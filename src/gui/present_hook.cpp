/*
 *  Copyright (C) 2002-2026  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/* Present hook (Phase 9A, DOSBox-X-AI project) -- see include/present_hook.h
 * and docs/phase9a-composite-capture-design.md for the design. */

#include "dosbox.h"
#include "sdlmain.h"
#include "render.h"
#include "present_hook.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

std::atomic<bool> g_presentHookArmed{false};

static std::atomic<uint64_t> g_renderSeq{0};
static std::atomic<bool>     g_compositeWantsFullFrame{false};

static std::mutex                                     g_compositeMutex;
static std::condition_variable                        g_compositeCv;
static std::deque<std::shared_ptr<CompositeRequest>>  g_compositeRequests;

/* Recomputes both hot-path flags from g_compositeRequests. Caller holds
 * g_compositeMutex. */
static void RefreshFlagsLocked(void) {
    bool anyPending = false, anyArmed = false;
    for (const auto &r : g_compositeRequests) {
        if (r->state == CompositeRequest::State::Pending) anyPending = true;
        else if (r->state == CompositeRequest::State::Armed) anyArmed = true;
    }
    g_compositeWantsFullFrame.store(anyPending, std::memory_order_relaxed);
    g_presentHookArmed.store(anyArmed, std::memory_order_relaxed);
}

/* Caller holds g_compositeMutex. Removes finished requests from the queue. */
static void PruneDoneLocked(void) {
    g_compositeRequests.erase(
        std::remove_if(g_compositeRequests.begin(), g_compositeRequests.end(),
            [](const std::shared_ptr<CompositeRequest> &r) { return r->state == CompositeRequest::State::Done; }),
        g_compositeRequests.end());
}

/* ---- present thread ---------------------------------------------------- */

void PRESENT_Hook_BeforePresent(const PresentContext &ctx, IPresentReadback &rb) {
    std::vector<std::shared_ptr<CompositeRequest>> due;
    {
        std::lock_guard<std::mutex> lk(g_compositeMutex);
        for (const auto &r : g_compositeRequests)
            if (r->state == CompositeRequest::State::Armed && ctx.render_seq >= r->target_render_seq)
                due.push_back(r);
    }
    if (due.empty()) return;

    /* One readback serves every request due at this present. Done outside
     * the lock: GetRenderTargetData() stalls on the GPU. */
    std::vector<uint8_t> rgba;
    std::string errCode, errMsg;
    bool ok = ctx.bb_width > 0 && ctx.bb_height > 0 &&
        rb.ReadRGBA8888(0, 0, (int)ctx.bb_width, (int)ctx.bb_height, rgba, errCode, errMsg);
    if (!ok && errCode.empty()) {
        errCode = "INTERNAL_ERROR";
        errMsg = "back buffer readback failed";
    }

    {
        std::lock_guard<std::mutex> lk(g_compositeMutex);
        for (size_t i = 0; i < due.size(); i++) {
            CompositeRequest &r = *due[i];
            if (r.state != CompositeRequest::State::Armed) continue; /* cancelled meanwhile */
            r.ctx = ctx;
            r.ok = ok;
            if (ok) {
                if (i + 1 == due.size()) r.rgba = std::move(rgba);
                else r.rgba = rgba;
            } else {
                r.error_code = errCode;
                r.error_message = errMsg;
            }
            r.state = CompositeRequest::State::Done;
        }
        PruneDoneLocked();
        RefreshFlagsLocked();
    }
    g_compositeCv.notify_all();
}

void PRESENT_Hook_NoteDeviceLost(void) {
    std::lock_guard<std::mutex> lk(g_compositeMutex);
    for (const auto &r : g_compositeRequests)
        if (r->state == CompositeRequest::State::Armed) r->device_lost_seen = true;
}

/* ---- emulator thread ---------------------------------------------------- */

uint64_t PRESENT_OnRenderEndUpdate(void) {
    return g_renderSeq.fetch_add(1, std::memory_order_relaxed) + 1;
}

uint64_t PRESENT_GetRenderSeq(void) {
    return g_renderSeq.load(std::memory_order_relaxed);
}

bool PRESENT_Composite_WantsFullFrame(void) {
    return g_compositeWantsFullFrame.load(std::memory_order_relaxed);
}

void PRESENT_Composite_Arm(const CompositeArmInfo &info,
        const std::function<bool(std::vector<uint8_t> &, uint32_t &, uint32_t &)> &captureSource) {
    if (!g_compositeWantsFullFrame.load(std::memory_order_relaxed)) return;

    std::vector<std::shared_ptr<CompositeRequest>> toArm;
    bool wantSource = false;
    {
        std::lock_guard<std::mutex> lk(g_compositeMutex);
        for (const auto &r : g_compositeRequests)
            if (r->state == CompositeRequest::State::Pending) {
                toArm.push_back(r);
                wantSource |= r->include_source;
            }
    }
    if (toArm.empty()) return;

    /* The source frame is unpacked outside the lock -- it is the same
     * work video.frame.capture does and can take a moment at high res. */
    std::vector<uint8_t> src;
    uint32_t srcW = 0, srcH = 0;
    bool haveSource = wantSource && captureSource && captureSource(src, srcW, srcH);

    const bool supported = PRESENT_CurrentBackendSupported();
    const uint64_t seq = PRESENT_GetRenderSeq();

    {
        std::lock_guard<std::mutex> lk(g_compositeMutex);
        for (const auto &rp : toArm) {
            CompositeRequest &r = *rp;
            if (r.state != CompositeRequest::State::Pending) continue;
            r.target_render_seq = seq;
            r.captured_at_emulated_ms = info.emulated_ms;
            r.render_src_w = info.render_src_w;
            r.render_src_h = info.render_src_h;
            r.guest_native_w = info.guest_native_w;
            r.guest_native_h = info.guest_native_h;
            r.aspect_correction = info.aspect_correction;
            r.fullscreen = sdl.desktop.fullscreen;
            if (r.include_source && haveSource) {
                r.source_rgba = src;
                r.source_w = srcW;
                r.source_h = srcH;
            }
            if (!supported) {
                /* Backend changed after the request was accepted. Never
                 * fall back to the source frame -- see design doc 5.4. */
                r.ok = false;
                r.error_code = "COMPOSITE_UNSUPPORTED_BACKEND";
                r.error_message = std::string("output backend \"") + PRESENT_CurrentBackendName() +
                    "\" does not support composite capture (supported: " + PRESENT_SupportedBackendList() + ")";
                r.state = CompositeRequest::State::Done;
            } else {
                r.state = CompositeRequest::State::Armed;
            }
        }
        PruneDoneLocked();
        RefreshFlagsLocked();
    }
    if (!supported) g_compositeCv.notify_all();
}

/* ---- socket thread ---------------------------------------------------- */

/* TTF output keeps sdl.desktop.type at the backend it switched from, but
 * while ttf.inUse GFX_EndUpdate() hands frames to the TTF text renderer
 * instead of that backend -- so it is the one actually presenting. */
static bool TtfPresenting(void) {
#if defined(USE_TTF)
    return ttf.inUse;
#else
    return false;
#endif
}

const char *PRESENT_CurrentBackendName(void) {
    if (TtfPresenting()) return "ttf";
    switch (sdl.desktop.type) {
        case SCREEN_SURFACE:    return "surface";
        case SCREEN_OPENGL:     return "opengl";
#if C_DIRECT3D
        case SCREEN_DIRECT3D:   return "direct3d";
#endif
        case SCREEN_TTF:        return "ttf";
        case SCREEN_GAMELINK:   return "gamelink";
#if C_DIRECT3D && defined(C_SDL2)
        case SCREEN_DIRECT3D11: return "direct3d11";
#endif
#if defined(MACOSX) && defined(C_SDL2) && C_METAL
        case SCREEN_METAL:      return "metal";
#endif
        default:                return "unknown";
    }
}

bool PRESENT_CurrentBackendSupported(void) {
    if (TtfPresenting()) return false;
    switch (sdl.desktop.type) {
        case SCREEN_SURFACE:
#if C_DIRECT3D
        case SCREEN_DIRECT3D:
#endif
            return true;
        default:
            return false;
    }
}

const char *PRESENT_SupportedBackendList(void) {
#if C_DIRECT3D
    return "direct3d, surface";
#else
    return "surface";
#endif
}

void PRESENT_Composite_Submit(const std::shared_ptr<CompositeRequest> &req) {
    std::lock_guard<std::mutex> lk(g_compositeMutex);
    req->state = CompositeRequest::State::Pending;
    g_compositeRequests.push_back(req);
    RefreshFlagsLocked();
}

bool PRESENT_Composite_Wait(const std::shared_ptr<CompositeRequest> &req, unsigned int timeoutMs) {
    std::unique_lock<std::mutex> lk(g_compositeMutex);
    return g_compositeCv.wait_for(lk, std::chrono::milliseconds(timeoutMs),
        [&] { return req->state == CompositeRequest::State::Done; });
}

bool PRESENT_Composite_Cancel(const std::shared_ptr<CompositeRequest> &req) {
    std::lock_guard<std::mutex> lk(g_compositeMutex);
    if (req->state == CompositeRequest::State::Done) return false;
    g_compositeRequests.erase(
        std::remove(g_compositeRequests.begin(), g_compositeRequests.end(), req),
        g_compositeRequests.end());
    req->state = CompositeRequest::State::Done;
    RefreshFlagsLocked();
    return true;
}

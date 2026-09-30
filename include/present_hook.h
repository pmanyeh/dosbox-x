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

/*
 * Present hook (Phase 9A, DOSBox-X-AI project).
 *
 * A backend-agnostic point at which each output backend hands over its
 * FINAL composed image -- after scaling, filtering, pixel shaders and
 * letterboxing, immediately before it is presented to the screen. Phase 9A
 * uses it for the AI bridge's video.composite.capture method (see
 * src/debug/debug_ai.cpp and docs/phase9a-composite-capture-design.md);
 * later Modern Runtime overlays are meant to draw at the same point,
 * BEFORE the capture, so a capture always shows what the player sees.
 *
 * Threads:
 *   - Socket thread: PRESENT_Composite_Submit()/_Cancel()/_Wait().
 *   - Emulator thread: PRESENT_OnRenderEndUpdate(), PRESENT_Composite_Arm(),
 *     PRESENT_Composite_WantsFullFrame() (all from src/gui/render.cpp).
 *   - Present thread (the D3D9 worker thread for direct3d; the emulator
 *     thread for surface): PRESENT_Hook_BeforePresent(). It only copies
 *     pixels -- never PNG-encodes -- and never touches emulator state.
 *
 * Idle cost: PRESENT_Hook_Armed() and PRESENT_Composite_WantsFullFrame()
 * are single relaxed atomic loads.
 */

#ifndef DOSBOX_PRESENT_HOOK_H
#define DOSBOX_PRESENT_HOOK_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

/* Filled in by the backend at the moment it is about to present. */
struct PresentContext {
    const char *backend = "unknown";      /* "direct3d" | "surface" */
    uint64_t    render_seq = 0;           /* RENDER_EndUpdate() sequence this present belongs to */
    uint32_t    bb_width = 0, bb_height = 0;             /* back buffer size */
    int32_t     clip_x = 0, clip_y = 0, clip_w = 0, clip_h = 0; /* viewport (sdl.clip) */
    uint32_t    draw_width = 0, draw_height = 0;         /* texture/draw size handed to the backend */
    std::string pixel_shader = "none";
};

/* Implemented by each backend: reads a rectangle of the current back
 * buffer into tightly packed, top-to-bottom RGBA8888 (alpha always 255).
 * On failure returns false and sets errCode to one of the bridge's error
 * codes (e.g. "COMPOSITE_UNSUPPORTED_FORMAT") plus a human message. */
struct IPresentReadback {
    virtual ~IPresentReadback() {}
    virtual bool ReadRGBA8888(int x, int y, int w, int h, std::vector<uint8_t> &out,
                              std::string &errCode, std::string &errMsg) = 0;
};

/* One video.composite.capture request, shared between the three threads
 * above. Every field below "state" is guarded by the present hook's
 * internal mutex until state == Done, after which only the submitting
 * socket thread touches it. */
struct CompositeRequest {
    bool include_source = false;

    enum class State { Pending, Armed, Done } state = State::Pending;

    /* Arm (emulator thread) */
    uint64_t target_render_seq = 0;
    uint64_t captured_at_emulated_ms = 0;
    uint32_t render_src_w = 0, render_src_h = 0;     /* same size capture_frame reports */
    uint32_t guest_native_w = 0, guest_native_h = 0; /* e.g. 320x200 for mode 13h */
    bool aspect_correction = false;
    bool fullscreen = false;
    std::vector<uint8_t> source_rgba;
    uint32_t source_w = 0, source_h = 0;

    /* Present (present thread) */
    PresentContext ctx;
    std::vector<uint8_t> rgba;                        /* full back buffer, bb_width x bb_height */
    bool device_lost_seen = false;

    /* Outcome */
    bool ok = false;
    std::string error_code, error_message;
};

/* ---- present thread ---------------------------------------------------- */

extern std::atomic<bool> g_presentHookArmed;
static inline bool PRESENT_Hook_Armed(void) {
    return g_presentHookArmed.load(std::memory_order_relaxed);
}

/* Call only when PRESENT_Hook_Armed() is true, after the frame is fully
 * composed and before it is presented. */
void PRESENT_Hook_BeforePresent(const PresentContext &ctx, IPresentReadback &rb);

/* Called by a backend that skipped a present because its device is lost. */
void PRESENT_Hook_NoteDeviceLost(void);

/* ---- emulator thread (render.cpp) ------------------------------------- */

/* Increments and returns the RENDER_EndUpdate() sequence number. */
uint64_t PRESENT_OnRenderEndUpdate(void);
uint64_t PRESENT_GetRenderSeq(void);

/* True while any request is waiting to be armed: RENDER_StartUpdate()
 * then takes its full-redraw path so the next frame is guaranteed to be
 * handed to the backend even if the guest screen is static. */
bool PRESENT_Composite_WantsFullFrame(void);

struct CompositeArmInfo {
    uint64_t emulated_ms = 0;
    uint32_t render_src_w = 0, render_src_h = 0;
    uint32_t guest_native_w = 0, guest_native_h = 0;
    bool aspect_correction = false;
};

/* Arms every pending request against the frame whose sequence number is
 * PRESENT_GetRenderSeq(). Only call when this frame WILL be presented.
 * captureSource is invoked at most once, and only if some request wants
 * include_source. */
void PRESENT_Composite_Arm(const CompositeArmInfo &info,
    const std::function<bool(std::vector<uint8_t> &, uint32_t &, uint32_t &)> &captureSource);

/* ---- socket thread ---------------------------------------------------- */

/* The name of the backend currently presenting ("direct3d", "surface",
 * "opengl", "ttf", ...), and whether composite capture supports it. */
const char *PRESENT_CurrentBackendName(void);
bool PRESENT_CurrentBackendSupported(void);
const char *PRESENT_SupportedBackendList(void);

void PRESENT_Composite_Submit(const std::shared_ptr<CompositeRequest> &req);
/* Waits up to timeoutMs for req to reach Done. Returns true if it did. */
bool PRESENT_Composite_Wait(const std::shared_ptr<CompositeRequest> &req, unsigned int timeoutMs);
/* Withdraws a request that timed out. Returns true if it was still queued
 * (false if it completed in the meantime -- the caller may then use it). */
bool PRESENT_Composite_Cancel(const std::shared_ptr<CompositeRequest> &req);

#endif /* DOSBOX_PRESENT_HOOK_H */

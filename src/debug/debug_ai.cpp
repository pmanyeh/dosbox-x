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

#include "dosbox.h"
#if C_DEBUG

#include "debug_ai.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>
#include <map>
#include <deque>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <set>
#include <utility>
#include <cmath>

#include "regs.h"
#include "cpu.h"
#include "mem.h"
#include "paging.h"
#include "keyboard.h"
#include "mouse.h"
#include "logging.h"
#include "pic.h"
#include "hardware.h"
#include "render.h"
#include "video.h"

#if (C_SSHOT)
#include <zlib.h>
#include <png.h>
#endif

#if defined(WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET ai_socket_t;
#define AI_INVALID_SOCKET INVALID_SOCKET
#define ai_close closesocket
typedef int ai_socklen_t;
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
typedef int ai_socket_t;
#define AI_INVALID_SOCKET (-1)
#define ai_close close
typedef socklen_t ai_socklen_t;
#endif

/* Reused, unmodified, from debug.cpp -- NOT redeclared/duplicated logic,
 * just the existing external-linkage declarations debug.cpp already has.
 * debug_inc.h is deliberately not included here: it pulls in curses.h,
 * which this headless bridge has no need of. */
extern uint64_t GetAddress(uint16_t seg, uint32_t offset);
extern Bitu DasmI386(char* buffer, PhysPt pc, uint32_t cur_ip, bool bit32);
/* Phase 4D: repaints the curses debugger console (declared in
 * include/debug.h, defined in debug.cpp). Called after an
 * execution.step_over() that took the async call/int/loop/rep path
 * finishes (DEBUG_AI_CompletePendingSteps() below) so the human GUI
 * reflects the new position immediately, the same guarantee
 * DEBUG_AI_DoStepInto() (debug.cpp) already gives the synchronous case. */
extern void DEBUG_DrawScreen(void);

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

static const uint16_t AI_BRIDGE_PORT = 9876;
static const char *AI_BRIDGE_BIND_ADDR = "127.0.0.1";
static const size_t   MAX_LINE_LENGTH        = 8192;
static const long long MAX_READ_LENGTH       = 65536;
static const long long MAX_WRITE_LENGTH      = 65536;
static const long long MAX_DISASSEMBLE_COUNT = 100;
static const size_t    MAX_FRAME_PAYLOAD_BYTES = 8 * 1024 * 1024;
static const int       REQUEST_TIMEOUT_SECONDS = 5;

/* Phase 4A: registers writable through register.write. EIP, all segment
 * registers, ESP, and EFLAGS are deliberately excluded -- writing them
 * could desync the debugger/CPU (e.g. jump execution elsewhere, corrupt
 * the stack) and is out of scope until explicitly approved. */
static const char *WRITABLE_REGISTERS[] = { "eax", "ebx", "ecx", "edx", "esi", "edi", "ebp" };
static const char *BLOCKED_REGISTERS[]  = { "eip", "cs", "ds", "es", "ss", "fs", "gs", "esp", "eflags" };

/* ------------------------------------------------------------------ */
/* Minimal JSON value + parser                                         */
/*                                                                      */
/* This is intentionally not a general-purpose JSON library: it only   */
/* supports what the AI bridge protocol actually needs (Phase3.md      */
/* section 8) -- flat objects containing strings/numbers, with at most */
/* one level of nesting for "params", and flat arrays of strings/numbers */
/* (used only by memory.write's params.data, an array of byte values).  */
/* ------------------------------------------------------------------ */

struct JsonValue {
    enum class Type { Null, Bool, Number, String, Object, Array } type = Type::Null;
    double number = 0;
    std::string str;
    std::map<std::string, JsonValue> object;
    std::vector<JsonValue> array;
};

class JsonParser {
public:
    explicit JsonParser(const std::string &text) : s(text), i(0), n(text.size()) {}

    bool Parse(JsonValue &out) {
        SkipWs();
        if (!ParseValue(out)) return false;
        SkipWs();
        return i == n;
    }

private:
    const std::string &s;
    size_t i, n;

    void SkipWs() {
        while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
    }

    bool ParseValue(JsonValue &v) {
        SkipWs();
        if (i >= n) return false;
        char c = s[i];
        if (c == '{') return ParseObject(v);
        if (c == '[') return ParseArray(v);
        if (c == '"') return ParseString(v);
        if (c == '-' || (c >= '0' && c <= '9')) return ParseNumber(v);
        if (s.compare(i, 4, "true") == 0)  { v.type = JsonValue::Type::Bool; v.number = 1; i += 4; return true; }
        if (s.compare(i, 5, "false") == 0) { v.type = JsonValue::Type::Bool; v.number = 0; i += 5; return true; }
        if (s.compare(i, 4, "null") == 0)  { v.type = JsonValue::Type::Null; i += 4; return true; }
        return false; /* anything else is not part of this protocol */
    }

    bool ParseArray(JsonValue &v) {
        v.type = JsonValue::Type::Array;
        i++; /* consume '[' */
        SkipWs();
        if (i < n && s[i] == ']') { i++; return true; }
        for (;;) {
            JsonValue elem;
            if (!ParseValue(elem)) return false;
            v.array.push_back(elem);
            SkipWs();
            if (i >= n) return false;
            if (s[i] == ',') { i++; continue; }
            if (s[i] == ']') { i++; break; }
            return false;
        }
        return true;
    }

    bool ParseObject(JsonValue &v) {
        v.type = JsonValue::Type::Object;
        i++; /* consume '{' */
        SkipWs();
        if (i < n && s[i] == '}') { i++; return true; }
        for (;;) {
            SkipWs();
            JsonValue key;
            if (i >= n || s[i] != '"' || !ParseString(key)) return false;
            SkipWs();
            if (i >= n || s[i] != ':') return false;
            i++;
            JsonValue val;
            if (!ParseValue(val)) return false;
            v.object[key.str] = val;
            SkipWs();
            if (i >= n) return false;
            if (s[i] == ',') { i++; continue; }
            if (s[i] == '}') { i++; break; }
            return false;
        }
        return true;
    }

    bool ParseString(JsonValue &v) {
        v.type = JsonValue::Type::String;
        if (i >= n || s[i] != '"') return false;
        i++;
        std::string out;
        while (i < n && s[i] != '"') {
            char c = s[i];
            if (c == '\\') {
                i++;
                if (i >= n) return false;
                switch (s[i]) {
                    case '"':  out += '"';  break;
                    case '\\': out += '\\'; break;
                    case '/':  out += '/';  break;
                    case 'n':  out += '\n'; break;
                    case 't':  out += '\t'; break;
                    case 'r':  out += '\r'; break;
                    case 'b':  out += '\b'; break;
                    case 'f':  out += '\f'; break;
                    case 'u':
                        if (i + 4 >= n) return false;
                        out += '?'; /* no request field needs non-ASCII; skip the codepoint */
                        i += 4;
                        break;
                    default: return false;
                }
                i++;
            } else {
                out += c;
                i++;
            }
        }
        if (i >= n) return false; /* unterminated string */
        i++; /* consume closing quote */
        v.str = out;
        return true;
    }

    bool ParseNumber(JsonValue &v) {
        size_t start = i;
        if (i < n && s[i] == '-') i++;
        if (i >= n || !isdigit((unsigned char)s[i])) return false;
        while (i < n && isdigit((unsigned char)s[i])) i++;
        if (i < n && s[i] == '.') {
            i++;
            while (i < n && isdigit((unsigned char)s[i])) i++;
        }
        if (i < n && (s[i] == 'e' || s[i] == 'E')) {
            i++;
            if (i < n && (s[i] == '+' || s[i] == '-')) i++;
            while (i < n && isdigit((unsigned char)s[i])) i++;
        }
        v.type = JsonValue::Type::Number;
        v.number = strtod(s.substr(start, i - start).c_str(), nullptr);
        return true;
    }
};

static std::string JsonEscape(const std::string &in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (unsigned char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += (char)c;
                }
        }
    }
    return out;
}

/* SEG:OFF, e.g. "1234:0100" -- 1-4 hex digits per side, each in 0..0xFFFF. */
static bool ParseAddress(const std::string &s, uint16_t &seg, uint32_t &off) {
    size_t colon = s.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= s.size()) return false;
    std::string segStr = s.substr(0, colon);
    std::string offStr = s.substr(colon + 1);
    if (segStr.empty() || segStr.size() > 4 || offStr.empty() || offStr.size() > 4) return false;
    for (char c : segStr) if (!isxdigit((unsigned char)c)) return false;
    for (char c : offStr) if (!isxdigit((unsigned char)c)) return false;
    unsigned long segVal = strtoul(segStr.c_str(), nullptr, 16);
    unsigned long offVal = strtoul(offStr.c_str(), nullptr, 16);
    if (segVal > 0xFFFFu || offVal > 0xFFFFu) return false;
    seg = (uint16_t)segVal;
    off = (uint32_t)offVal;
    return true;
}

/* ------------------------------------------------------------------ */
/* Connection + request queue plumbing                                 */
/* ------------------------------------------------------------------ */

struct AIConnection {
    ai_socket_t sock = AI_INVALID_SOCKET;
    std::mutex mtx;
    std::condition_variable cv;
    bool responseReady = false;
    std::string responseLine;
    std::atomic<bool> stopping{false};
};

enum class AIMethod {
    DebugStatus, CpuGet, MemoryRead, CodeCurrent, CodeDisassemble,
    MemoryWrite, RegisterWrite,
    BreakpointSet, ProtectedMemoryBreakpointSet, RealMemoryBreakpointSet, BreakpointDelete, BreakpointList,
    ExecutionContinue,
    ExecutionStepInto, ExecutionStepOver,
    /* Phase 7B: the "stopped" half of input.mouse.capture.get/.set's dual
     * route -- see "Mouse capture & absolute input (Phase 7B)" below. */
    MouseCaptureGet, MouseCaptureSet
};

struct AIRequestItem {
    std::shared_ptr<AIConnection> conn;
    long long id = 0;
    AIMethod method = AIMethod::DebugStatus;
    uint16_t seg = 0;
    uint32_t off = 0;
    long long length = 0;
    long long count = 0;
    std::vector<uint8_t> writeBytes;  /* memory.write */
    std::string regName;              /* register.write -- already whitelist-checked */
    uint32_t regValue = 0;            /* register.write */
    long long breakpointIndex = -1;   /* breakpoint.delete */
    bool mouseCaptureDesired = false; /* input.mouse.capture.set, stopped route */
};

static std::atomic<bool> g_bridgeRunning{false};
static ai_socket_t       g_listenSocket = AI_INVALID_SOCKET;
static std::thread       g_acceptThread;

static std::mutex g_connsMutex;
static std::vector<std::shared_ptr<AIConnection>> g_connections;

static std::mutex g_queueMutex;
static std::deque<AIRequestItem> g_requestQueue;

/* ------------------------------------------------------------------ */
/* Execution-control state (Phase 4C)                                  */
/*                                                                      */
/* g_debuggerActive mirrors whether DEBUG_Loop() is currently the       */
/* active main-loop handler -- i.e. whether the debugger is genuinely   */
/* stopped right now. It is written ONLY from the emulator thread       */
/* (DEBUG_AI_SetDebuggerActive(), called from DEBUG_Loop()/              */
/* DEBUG_AI_DoContinue() in debug.cpp and from Normal_Loop() in          */
/* dosbox.cpp) and read from socket threads via std::atomic -- unlike a */
/* bare bool, this has well-defined cross-thread visibility, and it is  */
/* bridge-level metadata, not emulator state, so reading it from a      */
/* socket thread does not violate the "never touch cpu_regs/Segs/guest  */
/* memory off the emulator thread" rule.                                */
/*                                                                      */
/* g_pendingPauses is the "request queue" for pause_execution(): unlike  */
/* every other AI bridge method, pause is only meaningful while the     */
/* debugger is NOT stopped, i.e. while DEBUG_Loop()/DEBUG_AI_Poll() are  */
/* not running at all -- there is nothing here for the existing         */
/* g_requestQueue to be drained by. Pending pause requests are instead   */
/* consumed from Normal_Loop's existing per-iteration DEBUG_ExitLoop()   */
/* hook (dosbox.cpp) via DEBUG_AI_CheckPauseRequest() (debug.cpp), which */
/* still executes only on the emulator thread.                          */
/* ------------------------------------------------------------------ */

static std::atomic<bool> g_debuggerActive{false};

void DEBUG_AI_SetDebuggerActive(bool active) {
    g_debuggerActive.store(active);
}

struct PendingPause {
    std::shared_ptr<AIConnection> conn;
    long long id;
};

static std::mutex g_pauseMutex;
static std::vector<PendingPause> g_pendingPauses;

static void DEBUG_AI_RequestPause(const std::shared_ptr<AIConnection> &conn, long long id) {
    std::lock_guard<std::mutex> lk(g_pauseMutex);
    g_pendingPauses.push_back({conn, id});
}

/* Removes conn's pending pause request, if any -- called by the socket
 * thread after its wait times out, so a late DEBUG_AI_CompletePendingPauses()
 * doesn't write into a response slot nobody is waiting on anymore. */
static void DEBUG_AI_CancelPause(const std::shared_ptr<AIConnection> &conn) {
    std::lock_guard<std::mutex> lk(g_pauseMutex);
    g_pendingPauses.erase(
        std::remove_if(g_pendingPauses.begin(), g_pendingPauses.end(),
                        [&](const PendingPause &p) { return p.conn == conn; }),
        g_pendingPauses.end());
}

bool DEBUG_AI_HasPendingPause(void) {
    std::lock_guard<std::mutex> lk(g_pauseMutex);
    return !g_pendingPauses.empty();
}

/* ------------------------------------------------------------------ */
/* Single-step control (Phase 4D)                                      */
/*                                                                      */
/* g_pendingSteps mirrors g_pendingPauses above, but for the ASYNC half */
/* of execution.step_over(): DEBUG_AI_Poll() (below) calls               */
/* DEBUG_AI_DoStepOver() directly (it's another item in the SAME          */
/* g_requestQueue everything else uses -- unlike pause, kicking off a     */
/* step requires emulator-thread work, so it cannot skip the queue the   */
/* way pause does), and if THAT reports becameAsync (the current          */
/* instruction was call/int/loop/rep, so DEBUG_Run(1,false) just handed   */
/* control to Normal_Loop(), exactly like execution.continue), the        */
/* request is parked here instead of being completed immediately.        */
/* DEBUG_AI_CompletePendingSteps() (called from DEBUG_Loop(), debug.cpp) */
/* drains it once the debugger regains control. */
/* ------------------------------------------------------------------ */

static std::mutex g_stepMutex;
static std::vector<PendingPause> g_pendingSteps;  /* same {conn,id} shape as PendingPause */

static void DEBUG_AI_RequestStepCompletion(const std::shared_ptr<AIConnection> &conn, long long id) {
    std::lock_guard<std::mutex> lk(g_stepMutex);
    g_pendingSteps.push_back({conn, id});
}

/* Mirrors DEBUG_AI_CancelPause() -- called by the socket thread after its
 * wait times out, so a late DEBUG_AI_CompletePendingSteps() doesn't write
 * into a response slot nobody is waiting on anymore. */
static void DEBUG_AI_CancelStep(const std::shared_ptr<AIConnection> &conn) {
    std::lock_guard<std::mutex> lk(g_stepMutex);
    g_pendingSteps.erase(
        std::remove_if(g_pendingSteps.begin(), g_pendingSteps.end(),
                        [&](const PendingPause &p) { return p.conn == conn; }),
        g_pendingSteps.end());
}

/* ------------------------------------------------------------------ */
/* Result builders -- these run ONLY from DEBUG_AI_Poll(), i.e. only    */
/* on the DOSBox-X emulator/debugger thread. This is the sole place    */
/* in this file that touches cpu_regs / Segs / guest memory.           */
/* ------------------------------------------------------------------ */

static std::string ReadInstructionBytesHex(uint32_t physStart, Bitu length) {
    std::string out;
    for (Bitu k = 0; k < length; k++) {
        uint8_t value = 0;
        bool fault = mem_readb_checked((LinearPt)(physStart + (uint32_t)k), &value);
        char b[8];
        if (fault) snprintf(b, sizeof(b), "%s??", k ? " " : "");
        else       snprintf(b, sizeof(b), "%s%02X", k ? " " : "", value);
        out += b;
    }
    return out;
}

static std::string BuildRegistersJson() {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"eax\":\"%08X\",\"ebx\":\"%08X\",\"ecx\":\"%08X\",\"edx\":\"%08X\","
        "\"esi\":\"%08X\",\"edi\":\"%08X\",\"ebp\":\"%08X\",\"esp\":\"%08X\"}",
        (unsigned)reg_eax, (unsigned)reg_ebx, (unsigned)reg_ecx, (unsigned)reg_edx,
        (unsigned)reg_esi, (unsigned)reg_edi, (unsigned)reg_ebp, (unsigned)reg_esp);
    return std::string(buf);
}

static std::string BuildSegmentsJson() {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"cs\":\"%04X\",\"ds\":\"%04X\",\"es\":\"%04X\",\"ss\":\"%04X\"}",
        (unsigned)SegValue(cs), (unsigned)SegValue(ds), (unsigned)SegValue(es), (unsigned)SegValue(ss));
    return std::string(buf);
}

static std::string ExecDebugStatus(long long id) {
    uint16_t curCS  = SegValue(cs);
    uint32_t curEIP = reg_eip;
    uint32_t physAddr = (uint32_t)GetAddress(curCS, curEIP);

    char dline[256];
    dline[0] = 0;
    Bitu size = DasmI386(dline, (PhysPt)physAddr, curEIP, cpu.code.big);
    if (size == 0) size = 1;
    std::string bytesHex = ReadInstructionBytesHex(physAddr, size);

    /* "stopped" reflects debug_running (Phase 3B semantics: false only in
     * RUNWATCH mode, where the debugger loop keeps running but the CPU is
     * executing semi-freely) -- unchanged from Phase 3B/4A/4B. "running" is
     * its Phase 4C addition: the plain inverse, added for symmetry with the
     * new execution.continue/execution.pause state model rather than
     * replacing the existing field (every prior test/consumer keys off
     * "stopped"). Both are always true/false together here because
     * ExecDebugStatus() can only ever run from DEBUG_AI_Poll(), i.e. only
     * while the debugger IS the active loop handler -- see
     * DEBUG_AI_SetDebuggerActive() for the separate, coarser "is the
     * debugger the active loop at all" signal used by the debug.status
     * fast path in HandleLine() for when it is NOT. */
    bool stopped = !DEBUG_AI_IsDebugRunning();

    char buf[2048];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{"
        "\"stopped\":%s,"
        "\"running\":%s,"
        "\"location\":{\"cs\":\"%04X\",\"eip\":\"%04X\"},"
        "\"instruction\":{\"bytes\":\"%s\",\"text\":\"%s\"},"
        "\"registers\":%s,"
        "\"segments\":%s,"
        "\"flags\":{\"eflags\":\"%08X\"}"
        "}}",
        id,
        stopped ? "true" : "false",
        stopped ? "false" : "true",
        (unsigned)curCS, (unsigned)(curEIP & 0xFFFF),
        bytesHex.c_str(), JsonEscape(dline).c_str(),
        BuildRegistersJson().c_str(),
        BuildSegmentsJson().c_str(),
        (unsigned)reg_flags);
    return std::string(buf);
}

/* Phase 4C: consumes every pending pause_execution() request (see
 * DEBUG_AI_RequestPause()) and answers each with a real debug.status
 * snapshot -- reusing ExecDebugStatus() rather than building a second,
 * parallel "here is the current state" payload. Called ONLY from
 * DEBUG_AI_CheckPauseRequest() (debug.cpp), i.e. only on the emulator
 * thread, and only immediately after DEBUG_Enable_Handler() has already
 * completed the real "enter debugger" transition -- so the state read
 * here (via ExecDebugStatus()) is genuine, current, post-pause state,
 * never fabricated or cached. */
void DEBUG_AI_CompletePendingPauses(void) {
    std::vector<PendingPause> pending;
    {
        std::lock_guard<std::mutex> lk(g_pauseMutex);
        pending.swap(g_pendingPauses);
    }
    for (const PendingPause &p : pending) {
        std::string resp = ExecDebugStatus(p.id);
        {
            std::lock_guard<std::mutex> lk(p.conn->mtx);
            p.conn->responseLine = resp;
            p.conn->responseReady = true;
        }
        p.conn->cv.notify_all();
    }
}

/* Phase 4D: completes every execution.step_over() that took the
 * asynchronous call/int/loop/rep path (see DEBUG_AI_RequestStepCompletion()
 * and "Single-step control" above) -- mirrors
 * DEBUG_AI_CompletePendingPauses() exactly (same ExecDebugStatus() reuse,
 * same real-post-transition-state guarantee), just called from a different
 * site (DEBUG_Loop() itself, debug.cpp, rather than
 * DEBUG_AI_CheckPauseRequest()) because "the debugger regained control" is
 * inherently a DEBUG_Loop()-side event for step_over, not a Normal_Loop()-
 * side one. A no-op whenever nothing is pending. */
void DEBUG_AI_CompletePendingSteps(void) {
    std::vector<PendingPause> pending;
    {
        std::lock_guard<std::mutex> lk(g_stepMutex);
        pending.swap(g_pendingSteps);
    }
    if (pending.empty()) return;

    DEBUG_DrawScreen();

    for (const PendingPause &p : pending) {
        std::string resp = ExecDebugStatus(p.id);
        {
            std::lock_guard<std::mutex> lk(p.conn->mtx);
            p.conn->responseLine = resp;
            p.conn->responseReady = true;
        }
        p.conn->cv.notify_all();
    }
}

/* ------------------------------------------------------------------ */
/* Input injection (Phase 6B)                                          */
/*                                                                      */
/* key_down/key_up/key_tap and the mouse.* methods inject guest input   */
/* through the SAME internal entry points the real SDL keyboard/mouse   */
/* event handlers already use (KEYBOARD_AddKey() in sdlmain.cpp's key   */
/* handler; Mouse_CursorMoved()/Mouse_ButtonPressed()/                  */
/* Mouse_ButtonReleased() in its mouse handler) -- there is no Win32     */
/* SendKeys/window-handle/focus-stealing/GUI-automation involved         */
/* anywhere in this path, only the guest-facing emulation DOSBox-X's own */
/* input handling already funnels every real keypress/click through.    */
/*                                                                       */
/* Like pause_execution(), these are only meaningful while guest code is */
/* actually running under Normal_Loop() (dosbox.cpp) -- KEYBOARD_AddKey()*/
/* and the Mouse_* functions are not safe to call from any other thread, */
/* and the debugger being stopped means Normal_Loop() does not have      */
/* control to run the per-iteration hook that drains this queue. So this */
/* reuses the exact same pattern as g_pendingPauses/                     */
/* DEBUG_AI_CheckPauseRequest(): requests are recorded here by a socket   */
/* thread and drained ONLY from DEBUG_AI_CheckPendingInput(), called from */
/* the SAME Normal_Loop() hook (dosbox.cpp) right next to                */
/* DEBUG_AI_CheckPauseRequest() -- see include/debug.h.                  */
/*                                                                       */
/* Stuck-input cleanup: g_heldKeys/g_heldButtons record, per connection,  */
/* which keys/buttons a key_down/mouse.button.set(pressed=true) left held */
/* down. If a connection disconnects (socket EOF, client crash, or a     */
/* session timeout on the AI harness side) while keys/buttons are still  */
/* held, ConnectionThreadFunc's disconnect path below enqueues a          */
/* ReleaseAll request for that connection -- with conn set but             */
/* expectsResponse=false, since nobody is waiting on a response by then -- */
/* so the guest never sees a key or button permanently stuck down because  */
/* the far end went away mid-session. input.release_all exposes the same   */
/* mechanism directly, for a well-behaved agent to call proactively at the  */
/* end of a session instead of relying on disconnect alone. */
/* ------------------------------------------------------------------ */

enum class AIInputOp {
    KeyDown, KeyUp, KeyTap,
    MouseMoveRelative, MouseButtonSet, MouseButtonClick,
    ReleaseAll,
    /* Phase 7B: the "running" half of input.mouse.capture.get/.set's dual
     * route, plus input.mouse.move_absolute/click_at (always running-only,
     * like every other guest-input op above) -- see "Mouse capture &
     * absolute input (Phase 7B)" below. */
    MouseCaptureGet, MouseCaptureSet, MouseMoveAbsolute, MouseClickAt
};

struct PendingInputRequest {
    AIInputOp op = AIInputOp::KeyTap;
    std::shared_ptr<AIConnection> conn;
    long long id = 0;
    bool expectsResponse = true;
    KBD_KEYS key = KBD_NONE;
    uint8_t button = 0;
    bool pressed = false;
    float dx = 0.0f;
    float dy = 0.0f;
    double x = 0.0;              /* move_absolute/click_at */
    double y = 0.0;              /* move_absolute/click_at */
    bool normalized = false;     /* move_absolute/click_at: x/y space */
    bool clamp = false;          /* move_absolute/click_at */
    bool captureDesired = false; /* MouseCaptureSet, running route */
};

static std::mutex g_inputMutex;
static std::deque<PendingInputRequest> g_pendingInputs;

/* Touched ONLY from the emulator thread, inside DEBUG_AI_CheckPendingInput()
 * -- both maps are private to that function's single caller, so no separate
 * lock is needed for them beyond g_inputMutex already protecting the queue
 * that feeds it. */
static std::map<std::shared_ptr<AIConnection>, std::set<KBD_KEYS>> g_heldKeys;
static std::map<std::shared_ptr<AIConnection>, std::set<uint8_t>> g_heldButtons;

static void DEBUG_AI_RequestInput(PendingInputRequest req) {
    std::lock_guard<std::mutex> lk(g_inputMutex);
    g_pendingInputs.push_back(std::move(req));
}

/* Mirrors DEBUG_AI_CancelPause(): called by the socket thread after its
 * wait times out, so a late DEBUG_AI_CheckPendingInput() completion doesn't
 * write into a response slot a SUBSEQUENT request on the same (now
 * re-used) connection is waiting on instead. */
static void DEBUG_AI_CancelInput(const std::shared_ptr<AIConnection> &conn, long long id) {
    std::lock_guard<std::mutex> lk(g_inputMutex);
    g_pendingInputs.erase(
        std::remove_if(g_pendingInputs.begin(), g_pendingInputs.end(),
                        [&](const PendingInputRequest &r) { return r.conn == conn && r.id == id; }),
        g_pendingInputs.end());
}

/* US 104-key layout only for Phase 6B v1 -- Windows keys, F13-F24, and the
 * Japanese/Korean-specific KBD_KEYS entries are intentionally not mapped
 * yet (not needed by any currently supported guest workflow, and best
 * added deliberately, with real test coverage, rather than guessed at
 * here). Unrecognized names fail closed (INVALID_PARAMETER), never sent as
 * some best-guess fallback. */
static bool ParseKeyName(const std::string &name, KBD_KEYS &key) {
    static const std::pair<const char *, KBD_KEYS> table[] = {
        {"1",KBD_1},{"2",KBD_2},{"3",KBD_3},{"4",KBD_4},{"5",KBD_5},
        {"6",KBD_6},{"7",KBD_7},{"8",KBD_8},{"9",KBD_9},{"0",KBD_0},
        {"q",KBD_q},{"w",KBD_w},{"e",KBD_e},{"r",KBD_r},{"t",KBD_t},
        {"y",KBD_y},{"u",KBD_u},{"i",KBD_i},{"o",KBD_o},{"p",KBD_p},
        {"a",KBD_a},{"s",KBD_s},{"d",KBD_d},{"f",KBD_f},{"g",KBD_g},
        {"h",KBD_h},{"j",KBD_j},{"k",KBD_k},{"l",KBD_l},{"z",KBD_z},
        {"x",KBD_x},{"c",KBD_c},{"v",KBD_v},{"b",KBD_b},{"n",KBD_n},{"m",KBD_m},
        {"f1",KBD_f1},{"f2",KBD_f2},{"f3",KBD_f3},{"f4",KBD_f4},
        {"f5",KBD_f5},{"f6",KBD_f6},{"f7",KBD_f7},{"f8",KBD_f8},
        {"f9",KBD_f9},{"f10",KBD_f10},{"f11",KBD_f11},{"f12",KBD_f12},
        {"esc",KBD_esc},{"tab",KBD_tab},{"backspace",KBD_backspace},
        {"enter",KBD_enter},{"space",KBD_space},
        {"leftalt",KBD_leftalt},{"rightalt",KBD_rightalt},
        {"leftctrl",KBD_leftctrl},{"rightctrl",KBD_rightctrl},
        {"leftshift",KBD_leftshift},{"rightshift",KBD_rightshift},
        {"capslock",KBD_capslock},{"scrolllock",KBD_scrolllock},{"numlock",KBD_numlock},
        {"grave",KBD_grave},{"minus",KBD_minus},{"equals",KBD_equals},
        {"backslash",KBD_backslash},{"leftbracket",KBD_leftbracket},{"rightbracket",KBD_rightbracket},
        {"semicolon",KBD_semicolon},{"quote",KBD_quote},{"period",KBD_period},
        {"comma",KBD_comma},{"slash",KBD_slash},
        {"printscreen",KBD_printscreen},{"pause",KBD_pause},
        {"insert",KBD_insert},{"home",KBD_home},{"pageup",KBD_pageup},
        {"delete",KBD_delete},{"end",KBD_end},{"pagedown",KBD_pagedown},
        {"left",KBD_left},{"up",KBD_up},{"down",KBD_down},{"right",KBD_right},
        {"kp1",KBD_kp1},{"kp2",KBD_kp2},{"kp3",KBD_kp3},{"kp4",KBD_kp4},{"kp5",KBD_kp5},
        {"kp6",KBD_kp6},{"kp7",KBD_kp7},{"kp8",KBD_kp8},{"kp9",KBD_kp9},{"kp0",KBD_kp0},
        {"kpdivide",KBD_kpdivide},{"kpmultiply",KBD_kpmultiply},
        {"kpminus",KBD_kpminus},{"kpplus",KBD_kpplus},
        {"kpenter",KBD_kpenter},{"kpperiod",KBD_kpperiod},
    };
    for (const auto &e : table) {
        if (name == e.first) { key = e.second; return true; }
    }
    return false;
}

/* Erases (rather than merely clearing) conn's entries once released, so
 * g_heldKeys/g_heldButtons don't grow by one entry per connection for the
 * lifetime of the process -- every connection eventually reaches either an
 * explicit input.release_all or the disconnect-triggered ReleaseAll above. */
static void ReleaseAllHeldInput(const std::shared_ptr<AIConnection> &conn) {
    auto itK = g_heldKeys.find(conn);
    if (itK != g_heldKeys.end()) {
        for (KBD_KEYS k : itK->second) KEYBOARD_AddKey(k, false);
        g_heldKeys.erase(itK);
    }
    auto itB = g_heldButtons.find(conn);
    if (itB != g_heldButtons.end()) {
        for (uint8_t b : itB->second) Mouse_ButtonReleased(b);
        g_heldButtons.erase(itB);
    }
}

/* ------------------------------------------------------------------ */
/* Mouse capture & absolute input (Phase 7B)                           */
/*                                                                      */
/* input.mouse.capture.get/.set work whether the debugger is stopped or */
/* running (unlike every input.* method above) -- see docs/             */
/* phase7b-mouse-capture-and-absolute-input-design.md for the full      */
/* design. Both are serviced by the SAME two helpers below regardless   */
/* of which of the two existing drain mechanisms (g_requestQueue/       */
/* DEBUG_AI_Poll() while stopped, g_pendingInputs/                      */
/* DEBUG_AI_CheckPendingInput() while running) reached them, so the two  */
/* routes can never disagree. input.mouse.move_absolute/click_at, by     */
/* contrast, dispatch real guest input (Mouse_CursorMoved()/Mouse_*()),  */
/* so -- like every OTHER input.* method -- they only ever go through    */
/* the running-only route and are rejected with DEBUGGER_STOPPED while   */
/* stopped (see HandleLine()).                                          */
/*                                                                       */
/* last_guest_x/y and g_nextInputSequence are touched ONLY from the two  */
/* drain sites above (both exclusively main-thread), mirroring           */
/* g_heldKeys/g_heldButtons's existing "emulator-thread only" comment    */
/* above -- no lock needed.                                              */
/* ------------------------------------------------------------------ */

static std::atomic<uint64_t> g_nextInputSequence{1};
static bool     g_hasLastGuestXY = false;
static long long g_lastGuestX = 0;
static long long g_lastGuestY = 0;

/* ------------------------------------------------------------------ */
/* Input dispatch receipts (Phase 7C)                                  */
/*                                                                      */
/* Unlike capture.get/.set above, a receipt describes something that     */
/* ALREADY happened (a completed KEYBOARD_AddKey()/Mouse_*() dispatch),  */
/* so answering input.receipt.get needs no emulator-thread execution --  */
/* just safe concurrent access to already-computed history. Written      */
/* ONLY from DEBUG_AI_CheckPendingInput() (the emulator thread -- every  */
/* dispatching op, Phase 6B's and 7B's alike, goes through that one       */
/* function); read directly from whichever socket thread receives an     */
/* input.receipt.get, under the same mutex, with no queue involved at    */
/* all. See docs/phase7c-input-dispatch-receipts-design.md.              */
/* ------------------------------------------------------------------ */

struct InputReceipt {
    uint64_t sequence = 0;
    std::string device;                 /* "keyboard" | "mouse" */
    uint64_t dispatchedAtEmulatedMs = 0; /* PIC_FullIndex() at dispatch time */
    std::chrono::steady_clock::time_point recordedAt;
};

static std::mutex g_receiptMutex;
static std::deque<InputReceipt> g_receipts;
static const size_t MAX_RECEIPTS = 4096;
static const std::chrono::minutes RECEIPT_MAX_AGE{10};

/* Called ONLY from DEBUG_AI_CheckPendingInput(), immediately after a
 * dispatch actually happens -- never speculatively, never for a request
 * that failed validation (ABSOLUTE_MOUSE_UNAVAILABLE/INVALID_PARAMETER
 * cases below never reach this). Evicts by EITHER bound independently
 * (whichever an entry hits first), per the requirements draft section
 * 5.2. */
static void RecordInputReceipt(uint64_t seq, const char *device, uint64_t dispatchedAtMs) {
    std::lock_guard<std::mutex> lk(g_receiptMutex);
    auto now = std::chrono::steady_clock::now();
    while (!g_receipts.empty() && (now - g_receipts.front().recordedAt) > RECEIPT_MAX_AGE)
        g_receipts.pop_front();
    while (g_receipts.size() >= MAX_RECEIPTS)
        g_receipts.pop_front();

    InputReceipt r;
    r.sequence = seq;
    r.device = device;
    r.dispatchedAtEmulatedMs = dispatchedAtMs;
    r.recordedAt = now;
    g_receipts.push_back(std::move(r));
}

/* Called from a socket thread (HandleLine()). A stale (evicted)
 * input_sequence and one that was never issued are deliberately
 * indistinguishable -- this bridge does not separately track "highest
 * sequence ever issued" -- both simply report as not found
 * (INPUT_RECEIPT_EXPIRED). */
static bool LookupInputReceipt(uint64_t seq, InputReceipt &out) {
    std::lock_guard<std::mutex> lk(g_receiptMutex);
    for (const auto &r : g_receipts) {
        if (r.sequence == seq) { out = r; return true; }
    }
    return false;
}

/* Shared by every dispatching case in DEBUG_AI_CheckPendingInput()'s
 * switch below -- the five receipt fields (requirements draft section
 * 5.1), as a JSON fragment (no surrounding braces) to splice into each
 * op's existing result object. guest_observed is always
 * "not_supported" in this implementation -- see the design doc's
 * "Goal" section for why real guest-side observation is out of scope
 * for now, not merely unfinished. */
static std::string ReceiptFieldsJson(uint64_t seq, uint64_t dispatchedAtMs) {
    return "\"queued\":true,\"dispatched\":true,"
        "\"dispatched_at_emulated_ms\":" + std::to_string(dispatchedAtMs) + ","
        "\"input_sequence\":" + std::to_string(seq) + ","
        "\"guest_observed\":\"not_supported\"";
}

/* render.src.width/height doubled by dblw/dblh -- the SAME formula
 * UnpackFrameToRGBA8888() (Phase 7A, above) applies, so this always
 * agrees with video.frame.capture's own reported width/height for the
 * current video mode. Returns false (leaving outW/outH untouched) before
 * any video mode has been established. */
static bool ResolveGuestPixelSize(Bitu &outW, Bitu &outH) {
    Bitu w = render.src.width  * (render.src.dblw ? 2 : 1);
    Bitu h = render.src.height * (render.src.dblh ? 2 : 1);
    if (w == 0 || h == 0) return false;
    outW = w;
    outH = h;
    return true;
}

/* Shared by both drain routes -- see section comment above. */
static std::string BuildMouseCaptureStatusResult(long long id) {
    Bitu w = 0, h = 0;
    bool haveSize = ResolveGuestPixelSize(w, h);
    bool absoluteAvailable = Mouse_AbsolutePositioningAvailable();

    return "{\"id\":" + std::to_string(id) + ",\"ok\":true,\"result\":{"
        "\"captured\":" + (mouselocked ? "true" : "false") + ","
        "\"autolock\":" + (GFX_MouseAutoLockEnabled() ? "true" : "false") + ","
        "\"mode\":\"" + (absoluteAvailable ? "absolute" : "relative") + "\","
        "\"guest_width\":" + (haveSize ? std::to_string(w) : "null") + ","
        "\"guest_height\":" + (haveSize ? std::to_string(h) : "null") + ","
        "\"last_guest_x\":" + (g_hasLastGuestXY ? std::to_string(g_lastGuestX) : "null") + ","
        "\"last_guest_y\":" + (g_hasLastGuestXY ? std::to_string(g_lastGuestY) : "null") +
        "}}";
}

/* Shared by both drain routes -- see section comment above. GFX_CaptureMouse()
 * is the EXACT function Ctrl+F10 already calls (src/gui/sdlmain.cpp); no
 * window focus/host cursor manipulation happens here. */
static std::string ApplyMouseCaptureSet(long long id, bool desired) {
    GFX_CaptureMouse(desired);
    return BuildMouseCaptureStatusResult(id);
}

/* Resolves one move_absolute/click_at request's requested (x,y) -- in
 * either coordinate space -- into BOTH the normalized [0,1] coordinates
 * Mouse_CursorMoved()'s absolute branch consumes AND the resolved
 * guest-pixel coordinates the response reports (the requirements draft
 * requires the result to always read "guest_pixels", section 4.1,
 * regardless of which space the request used). Returns false (and leaves
 * every out-parameter untouched) if the target falls outside the guest's
 * pixel bounds and clamp=false -- never a partially-resolved position. */
static bool ResolveAbsoluteTarget(double reqX, double reqY, bool normalized, bool clamp,
        Bitu guestW, Bitu guestH,
        double &outNormX, double &outNormY, long long &outGuestX, long long &outGuestY,
        bool &outClamped) {
    const double maxX = guestW > 1 ? (double)(guestW - 1) : 0.0;
    const double maxY = guestH > 1 ? (double)(guestH - 1) : 0.0;

    double px = normalized ? reqX * maxX : reqX;
    double py = normalized ? reqY * maxY : reqY;

    outClamped = false;
    if (px < 0.0 || px > maxX || py < 0.0 || py > maxY) {
        if (!clamp) return false;
        px = std::min<double>(std::max<double>(px, 0.0), maxX);
        py = std::min<double>(std::max<double>(py, 0.0), maxY);
        outClamped = true;
    }

    outGuestX = (long long)std::lround(px);
    outGuestY = (long long)std::lround(py);
    outNormX = maxX > 0.0 ? px / maxX : 0.0;
    outNormY = maxY > 0.0 ? py / maxY : 0.0;
    return true;
}

/* Drains every queued key/mouse input request and executes it against the
 * SAME KEYBOARD_AddKey()/Mouse_*() entry points real SDL input events use.
 * MUST be called only from the emulator thread, from the Normal_Loop()
 * per-iteration hook (dosbox.cpp) -- see include/debug.h and the section
 * comment above. A no-op (and cheap: one lock+empty check) whenever
 * nothing is pending. */
void DEBUG_AI_CheckPendingInput(void) {
    std::deque<PendingInputRequest> pending;
    {
        std::lock_guard<std::mutex> lk(g_inputMutex);
        if (g_pendingInputs.empty()) return;
        pending.swap(g_pendingInputs);
    }

    for (PendingInputRequest &r : pending) {
        char buf[384];
        buf[0] = '\0';
        /* Phase 7B: MouseCaptureGet/Set build a response via
         * BuildMouseCaptureStatusResult()/ApplyMouseCaptureSet() (variable
         * length, unlike every fixed-format snprintf() case below) --
         * dynResp takes priority over buf when non-empty. Freshly
         * default-constructed (empty) each loop iteration. */
        std::string dynResp;

        switch (r.op) {
            case AIInputOp::KeyDown: {
                KEYBOARD_AddKey(r.key, true);
                g_heldKeys[r.conn].insert(r.key);
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "keyboard", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"pressed\":true," +
                    ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::KeyUp: {
                KEYBOARD_AddKey(r.key, false);
                g_heldKeys[r.conn].erase(r.key);
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "keyboard", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"pressed\":false," +
                    ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::KeyTap: {
                KEYBOARD_AddKey(r.key, true);
                KEYBOARD_AddKey(r.key, false);
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "keyboard", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"tapped\":true," +
                    ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::MouseMoveRelative: {
                Mouse_CursorMoved(r.dx, r.dy, 0.0f, 0.0f, true);
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "mouse", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"moved\":true," +
                    ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::MouseButtonSet: {
                if (r.pressed) {
                    Mouse_ButtonPressed(r.button);
                    g_heldButtons[r.conn].insert(r.button);
                } else {
                    Mouse_ButtonReleased(r.button);
                    g_heldButtons[r.conn].erase(r.button);
                }
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "mouse", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"pressed\":" +
                    std::string(r.pressed ? "true" : "false") + "," + ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::MouseButtonClick: {
                Mouse_ButtonPressed(r.button);
                Mouse_ButtonReleased(r.button);
                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "mouse", ms);
                dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{\"clicked\":true," +
                    ReceiptFieldsJson(seq, ms) + "}}";
                break;
            }
            case AIInputOp::ReleaseAll:
                /* No receipt: input.release_all is neither input.key.* nor
                 * input.mouse.* by name, and releases an arbitrary number
                 * of keys/buttons at once, so no single input_sequence
                 * would meaningfully describe it -- see the Phase 7C
                 * design doc's "What gets a receipt" section. */
                ReleaseAllHeldInput(r.conn);
                snprintf(buf, sizeof(buf), "{\"id\":%lld,\"ok\":true,\"result\":{\"released\":true}}", r.id);
                break;
            case AIInputOp::MouseCaptureGet:
                dynResp = BuildMouseCaptureStatusResult(r.id);
                break;
            case AIInputOp::MouseCaptureSet:
                dynResp = ApplyMouseCaptureSet(r.id, r.captureDesired);
                break;
            case AIInputOp::MouseMoveAbsolute:
            case AIInputOp::MouseClickAt: {
                if (!Mouse_AbsolutePositioningAvailable()) {
                    snprintf(buf, sizeof(buf),
                        "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"ABSOLUTE_MOUSE_UNAVAILABLE\","
                        "\"message\":\"absolute mouse positioning is not available in the guest's current mode\"}}",
                        r.id);
                    break;
                }
                Bitu guestW = 0, guestH = 0;
                if (!ResolveGuestPixelSize(guestW, guestH)) {
                    snprintf(buf, sizeof(buf),
                        "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"ABSOLUTE_MOUSE_UNAVAILABLE\","
                        "\"message\":\"guest video mode not yet established\"}}", r.id);
                    break;
                }

                double normX = 0.0, normY = 0.0;
                long long guestX = 0, guestY = 0;
                bool clamped = false;
                if (!ResolveAbsoluteTarget(r.x, r.y, r.normalized, r.clamp, guestW, guestH,
                        normX, normY, guestX, guestY, clamped)) {
                    snprintf(buf, sizeof(buf),
                        "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"INVALID_PARAMETER\","
                        "\"message\":\"requested coordinates are outside the guest's pixel bounds\"}}", r.id);
                    break;
                }

                /* Mouse_CursorMoved()'s emulate=false branch (src/ints/
                 * mouse.cpp) -- the SAME internal path DOSBox-X's own
                 * seamless/integrated mouse positioning already uses, not a
                 * bridge invention. r.op == MouseClickAt performs move +
                 * button down + button up in THIS single case, before the
                 * next queued item (if any) can run -- nothing else touches
                 * the emulator thread between them, satisfying the
                 * requirements draft's "same emulator-thread dispatch"
                 * requirement (section 4.1) without a new lock. */
                Mouse_CursorMoved(0.0f, 0.0f, (float)normX, (float)normY, false);
                g_lastGuestX = guestX;
                g_lastGuestY = guestY;
                g_hasLastGuestXY = true;

                uint64_t seq = g_nextInputSequence.fetch_add(1);
                uint64_t ms = (uint64_t)PIC_FullIndex();
                RecordInputReceipt(seq, "mouse", ms);

                std::string common = "\"guest_x\":" + std::to_string(guestX) +
                    ",\"guest_y\":" + std::to_string(guestY) +
                    ",\"coordinate_space\":\"guest_pixels\","
                    "\"clamped\":" + (clamped ? "true" : "false") + "," +
                    ReceiptFieldsJson(seq, ms);

                if (r.op == AIInputOp::MouseClickAt) {
                    Mouse_ButtonPressed(r.button);
                    Mouse_ButtonReleased(r.button);
                    dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{" +
                        common + ",\"clicked\":true}}";
                } else {
                    dynResp = "{\"id\":" + std::to_string(r.id) + ",\"ok\":true,\"result\":{" + common + "}}";
                }
                break;
            }
        }

        if (r.conn && r.expectsResponse) {
            std::string responseLine = dynResp.empty() ? std::string(buf) : dynResp;
            {
                std::lock_guard<std::mutex> lk(r.conn->mtx);
                r.conn->responseLine = responseLine;
                r.conn->responseReady = true;
            }
            r.conn->cv.notify_all();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Frame capture (Phase 7A)                                            */
/*                                                                      */
/* video.frame.capture returns a PNG or raw RGBA8888 snapshot of        */
/* exactly the guest's own rendered frame -- never the DOSBox-X window, */
/* the SDL/GL surface as presented, or the host desktop. It reuses      */
/* DOSBox-X's OWN existing screenshot/AVI mechanism's hook point rather  */
/* than inventing a new one: RENDER_EndUpdate() (src/gui/render.cpp)     */
/* already hands the pre-scaler, pre-backend frame                      */
/* (scalerSourceCacheBuffer) to CAPTURE_AddImage() (src/hardware/        */
/* hardware.cpp) whenever a Host+P screenshot or AVI recording is        */
/* pending -- confirmed backend-agnostic (the SAME call site regardless  */
/* of software/OpenGL/Direct3D/Voodoo output) by docs/                   */
/* phase7a-frame-capture-design.md's source investigation. This adds a   */
/* second, independent check at that exact same site, so a pending AI    */
/* bridge capture request is serviced the moment a frame is genuinely    */
/* rendered, without waiting for or depending on the user's own          */
/* screenshot feature.                                                   */
/*                                                                       */
/* Thread: RENDER_EndUpdate() runs on the emulator thread (PIC-event-    */
/* driven from the VGA draw routines, per the design doc's                */
/* investigation) -- the SAME thread Phase 6B's KEYBOARD_AddKey()/        */
/* Mouse_*() calls are already safe on. So this reuses the identical      */
/* "pending request recorded by a socket thread, drained on the emulator  */
/* thread" pattern as g_pendingPauses/g_pendingInputs, just with a NEW    */
/* drain site (DEBUG_AI_CheckPendingFrameCapture(), called from           */
/* RENDER_EndUpdate() -- see include/debug.h) instead of Normal_Loop()'s  */
/* existing hook, since RENDER_EndUpdate() is the only place              */
/* scalerSourceCacheBuffer is valid before the next frame overwrites it.  */
/*                                                                       */
/* Unlike a screenshot, nothing here is ever written to disk -- PNG       */
/* encoding goes through libpng's png_set_write_fn() into an in-memory    */
/* buffer, bypassing CAPTURE_AddImage()'s own OpenCaptureFile() entirely,  */
/* so an agent calling this repeatedly never fills the user's own          */
/* screenshot folder.                                                     */
/* ------------------------------------------------------------------ */

struct PendingFrameCapture {
    std::shared_ptr<AIConnection> conn;
    long long id = 0;
    bool wantPng = true;       /* false => raw rgba8888 */
    uint32_t maxWidth = 0;     /* 0 = no limit (native size) */
    uint32_t maxHeight = 0;
};

static std::mutex                        g_frameCaptureMutex;
static std::deque<PendingFrameCapture>    g_pendingFrameCaptures;
static std::atomic<bool>                  g_frameCapturePending{false};
static std::atomic<uint64_t>              g_nextFrameId{1};

static void DEBUG_AI_RequestFrameCapture(PendingFrameCapture req) {
    std::lock_guard<std::mutex> lk(g_frameCaptureMutex);
    g_pendingFrameCaptures.push_back(std::move(req));
    g_frameCapturePending.store(true, std::memory_order_release);
}

/* Mirrors DEBUG_AI_CancelPause()/DEBUG_AI_CancelInput() -- called by the
 * socket thread after its wait times out, so a late frame-capture
 * completion doesn't write into a response slot a SUBSEQUENT request on
 * the same (now re-used) connection is waiting on instead. */
static void DEBUG_AI_CancelFrameCapture(const std::shared_ptr<AIConnection> &conn, long long id) {
    std::lock_guard<std::mutex> lk(g_frameCaptureMutex);
    g_pendingFrameCaptures.erase(
        std::remove_if(g_pendingFrameCaptures.begin(), g_pendingFrameCaptures.end(),
                        [&](const PendingFrameCapture &r) { return r.conn == conn && r.id == id; }),
        g_pendingFrameCaptures.end());
    if (g_pendingFrameCaptures.empty())
        g_frameCapturePending.store(false, std::memory_order_release);
}

static std::string Base64Encode(const uint8_t *data, size_t len) {
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    size_t i = 0;
    for (; i + 3 <= len; i += 3) {
        uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | data[i + 2];
        out += table[(n >> 18) & 0x3F];
        out += table[(n >> 12) & 0x3F];
        out += table[(n >> 6) & 0x3F];
        out += table[n & 0x3F];
    }
    size_t rem = len - i;
    if (rem == 1) {
        uint32_t n = (uint32_t)data[i] << 16;
        out += table[(n >> 18) & 0x3F];
        out += table[(n >> 12) & 0x3F];
        out += "==";
    } else if (rem == 2) {
        uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
        out += table[(n >> 18) & 0x3F];
        out += table[(n >> 12) & 0x3F];
        out += table[(n >> 6) & 0x3F];
        out += "=";
    }
    return out;
}

/* Unpacks one frame's worth of guest pixel data (native bpp 8/15/16/24/32,
 * as produced by the VGA draw handlers into scalerSourceCacheBuffer) into
 * a freshly allocated, top-to-bottom RGBA8888 buffer, applying
 * CAPTURE_FLAG_DBLW/DBLH the same way CAPTURE_AddImage() does (Phase 7A
 * design doc). Unlike CAPTURE_AddImage()'s own scratch buffer, this
 * writes straight into R,G,B,A order throughout -- CAPTURE_AddImage()'s
 * BGR-looking byte order only comes out correct there because of a
 * downstream png_set_bgr() call this function does not have (see the
 * design doc's "byte-order note"). The alpha channel is always 0xFF --
 * the guest has no concept of transparency. */
static std::vector<uint8_t> UnpackFrameToRGBA8888(Bitu width, Bitu height, Bitu bpp,
        Bitu pitch, Bitu flags, const uint8_t *data, const uint8_t *pal,
        Bitu &outWidth, Bitu &outHeight) {
    const bool dblw = (flags & CAPTURE_FLAG_DBLW) != 0;
    const bool dblh = (flags & CAPTURE_FLAG_DBLH) != 0;
    Bitu outW = width * (dblw ? 2 : 1);
    Bitu outH = height * (dblh ? 2 : 1);
    outWidth = outW;
    outHeight = outH;

    std::vector<uint8_t> out((size_t)outW * (size_t)outH * 4);

    for (Bitu y = 0; y < outH; y++) {
        Bitu srcY = dblh ? (y >> 1) : y;
        const uint8_t *srcLine = data + srcY * pitch;
        uint8_t *dstLine = out.data() + (size_t)y * outW * 4;

        for (Bitu x = 0; x < width; x++) {
            uint8_t r = 0, g = 0, b = 0;
            switch (bpp) {
                case 8: {
                    uint8_t idx = srcLine[x];
                    r = pal[(size_t)idx * 4 + 0];
                    g = pal[(size_t)idx * 4 + 1];
                    b = pal[(size_t)idx * 4 + 2];
                    break;
                }
                case 15: {
                    uint16_t pixel = ((const uint16_t *)srcLine)[x];
                    r = (uint8_t)(((pixel & 0x7c00u) * 0x21u) >> 12);
                    g = (uint8_t)(((pixel & 0x03e0u) * 0x21u) >> 7);
                    b = (uint8_t)(((pixel & 0x001fu) * 0x21u) >> 2);
                    break;
                }
                case 16: {
                    uint16_t pixel = ((const uint16_t *)srcLine)[x];
                    r = (uint8_t)(((pixel & 0xf800u) * 0x21u) >> 13);
                    g = (uint8_t)(((pixel & 0x07e0u) * 0x41u) >> 9);
                    b = (uint8_t)(((pixel & 0x001fu) * 0x21u) >> 2);
                    break;
                }
                case 24: {
                    const uint8_t *p = srcLine + (size_t)x * 3;
                    b = p[0]; g = p[1]; r = p[2]; /* source stored B,G,R */
                    break;
                }
                case 32:
                default: {
                    const uint8_t *p = srcLine + (size_t)x * 4;
                    b = p[0]; g = p[1]; r = p[2]; /* source stored B,G,R,(unused) */
                    break;
                }
            }

            uint8_t *dst = dstLine + (size_t)x * 4;
            dst[0] = r; dst[1] = g; dst[2] = b; dst[3] = 0xFF;

            if (dblw) {
                uint8_t *dst2 = dstLine + ((size_t)x * 2 + 1) * 4;
                dst2[0] = r; dst2[1] = g; dst2[2] = b; dst2[3] = 0xFF;
            }
        }
    }

    return out;
}

/* Simple nearest-neighbor downscale for max_width/max_height, per the
 * Phase 7A design doc's "high quality or a clearly documented
 * nearest-neighbor rule" allowance. Preserves aspect ratio -- the
 * smaller of the two implied scale factors wins. A no-op (returns the
 * input unchanged) if the image already fits. */
static std::vector<uint8_t> ScaleRGBA8888NearestNeighbor(const std::vector<uint8_t> &src,
        Bitu srcW, Bitu srcH, uint32_t maxW, uint32_t maxH, Bitu &outW, Bitu &outH) {
    double scale = 1.0;
    if (maxW && srcW > maxW) scale = std::min<double>(scale, (double)maxW / (double)srcW);
    if (maxH && srcH > maxH) scale = std::min<double>(scale, (double)maxH / (double)srcH);

    if (scale >= 1.0) {
        outW = srcW;
        outH = srcH;
        return src;
    }

    outW = std::max<Bitu>(1, (Bitu)((double)srcW * scale));
    outH = std::max<Bitu>(1, (Bitu)((double)srcH * scale));

    std::vector<uint8_t> out((size_t)outW * (size_t)outH * 4);
    for (Bitu y = 0; y < outH; y++) {
        Bitu srcY = std::min<Bitu>(srcH - 1, (Bitu)((double)y * srcH / outH));
        for (Bitu x = 0; x < outW; x++) {
            Bitu srcX = std::min<Bitu>(srcW - 1, (Bitu)((double)x * srcW / outW));
            const uint8_t *s = src.data() + ((size_t)srcY * srcW + srcX) * 4;
            uint8_t *d = out.data() + ((size_t)y * outW + x) * 4;
            d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3];
        }
    }
    return out;
}

#if (C_SSHOT)
namespace {
struct PngMemoryWriter {
    std::vector<uint8_t> buffer;
};
}

static void PngWriteCallback(png_structp png_ptr, png_bytep data, png_size_t length) {
    PngMemoryWriter *writer = (PngMemoryWriter *)png_get_io_ptr(png_ptr);
    writer->buffer.insert(writer->buffer.end(), data, data + length);
}

static void PngFlushCallback(png_structp) { /* nothing to flush -- in-memory only */ }

/* Encodes an RGBA8888 buffer to PNG entirely in memory -- deliberately
 * NOT reusing CAPTURE_AddImage()'s OpenCaptureFile()-based path, so this
 * never touches the user's own screenshot directory (see the section
 * comment above). */
static bool EncodeRGBA8888AsPng(const std::vector<uint8_t> &rgba, Bitu width, Bitu height,
        std::vector<uint8_t> &outPng) {
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) return false;
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    PngMemoryWriter writer;
    png_set_write_fn(png_ptr, &writer, PngWriteCallback, PngFlushCallback);

    png_set_IHDR(png_ptr, info_ptr, (png_uint_32)width, (png_uint_32)height, 8,
        PNG_COLOR_TYPE_RGB_ALPHA, PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png_ptr, info_ptr);

    std::vector<png_bytep> rows(height);
    for (Bitu y = 0; y < height; y++)
        rows[y] = (png_bytep)(rgba.data() + (size_t)y * width * 4);
    png_write_image(png_ptr, rows.data());
    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    outPng = std::move(writer.buffer);
    return true;
}
#endif

/* Builds the response for exactly one pending capture request against an
 * already-unpacked (and possibly already-scaled) RGBA8888 buffer. Split
 * out of DEBUG_AI_CheckPendingFrameCapture() below purely so each of its
 * several failure/success outcomes can use a plain `return` instead of
 * threading extra state through the loop body. */
static std::string BuildFrameCaptureResponse(const PendingFrameCapture &req,
        const std::vector<uint8_t> &finalRgba, Bitu outW, Bitu outH,
        uint64_t frameId, uint64_t emulatedMs) {
    const std::vector<uint8_t> *encoded = &finalRgba;
    std::vector<uint8_t> pngBytes;
    const char *payloadField = req.wantPng ? "png_base64" : "rgba_base64";

#if (C_SSHOT)
    if (req.wantPng) {
        if (!EncodeRGBA8888AsPng(finalRgba, outW, outH, pngBytes)) {
            return "{\"id\":" + std::to_string(req.id) +
                ",\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"PNG encoding failed\"}}";
        }
        encoded = &pngBytes;
    }
#else
    if (req.wantPng) {
        return "{\"id\":" + std::to_string(req.id) +
            ",\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"this build has no PNG support (C_SSHOT=0); use format=\\\"rgba\\\"\"}}";
    }
#endif

    if (encoded->size() > MAX_FRAME_PAYLOAD_BYTES) {
        /* Byte count is exact for rgba, and a reasonable (never an
         * under-estimate) basis for png too -- PNG compresses, so this
         * suggestion errs toward "smaller than strictly necessary" rather
         * than one that could still exceed the cap. */
        double scale = std::sqrt((double)MAX_FRAME_PAYLOAD_BYTES / (double)encoded->size());
        Bitu suggestedW = std::max<Bitu>(1, (Bitu)((double)outW * scale));
        Bitu suggestedH = std::max<Bitu>(1, (Bitu)((double)outH * scale));
        return "{\"id\":" + std::to_string(req.id) +
            ",\"ok\":false,\"error\":{\"code\":\"FRAME_TOO_LARGE\",\"message\":\"encoded frame exceeds the " +
            std::to_string(MAX_FRAME_PAYLOAD_BYTES) + "-byte cap\",\"suggested_max_width\":" +
            std::to_string(suggestedW) + ",\"suggested_max_height\":" + std::to_string(suggestedH) + "}}";
    }

    return "{\"id\":" + std::to_string(req.id) + ",\"ok\":true,\"result\":{"
        "\"frame_id\":" + std::to_string(frameId) + ","
        "\"width\":" + std::to_string(outW) + ","
        "\"height\":" + std::to_string(outH) + ","
        "\"pixel_format\":\"rgba8888\","
        "\"cursor_included\":false,"
        "\"captured_at_emulated_ms\":" + std::to_string(emulatedMs) + ","
        "\"" + std::string(payloadField) + "\":\"" + Base64Encode(encoded->data(), encoded->size()) + "\"}}";
}

/* Drains every queued video.frame.capture request against THIS rendered
 * frame's data. MUST be called only from RENDER_EndUpdate()
 * (src/gui/render.cpp), i.e. only from the emulator thread -- see
 * include/debug.h and the section comment above. Cheap (a single relaxed
 * atomic load) whenever nothing is pending -- safe to call
 * unconditionally on every rendered frame, the same way
 * DEBUG_AI_CheckPendingInput() is called unconditionally every
 * Normal_Loop() iteration. */
void DEBUG_AI_CheckPendingFrameCapture(Bitu width, Bitu height, Bitu bpp, Bitu pitch,
        Bitu flags, const uint8_t *data, const uint8_t *pal) {
    if (!g_frameCapturePending.load(std::memory_order_acquire)) return;

    std::deque<PendingFrameCapture> pending;
    {
        std::lock_guard<std::mutex> lk(g_frameCaptureMutex);
        pending.swap(g_pendingFrameCaptures);
        g_frameCapturePending.store(false, std::memory_order_release);
    }
    if (pending.empty()) return;

    Bitu nativeW = 0, nativeH = 0;
    std::vector<uint8_t> rgba = UnpackFrameToRGBA8888(width, height, bpp, pitch, flags, data, pal, nativeW, nativeH);

    uint64_t frameId = g_nextFrameId.fetch_add(1);
    uint64_t emulatedMs = (uint64_t)(PIC_FullIndex());

    for (PendingFrameCapture &req : pending) {
        Bitu outW = nativeW, outH = nativeH;
        const std::vector<uint8_t> *finalRgba = &rgba;
        std::vector<uint8_t> scaled;
        if (req.maxWidth || req.maxHeight) {
            scaled = ScaleRGBA8888NearestNeighbor(rgba, nativeW, nativeH, req.maxWidth, req.maxHeight, outW, outH);
            finalRgba = &scaled;
        }

        std::string resp = BuildFrameCaptureResponse(req, *finalRgba, outW, outH, frameId, emulatedMs);

        if (req.conn) {
            {
                std::lock_guard<std::mutex> lk(req.conn->mtx);
                req.conn->responseLine = resp;
                req.conn->responseReady = true;
            }
            req.conn->cv.notify_all();
        }
    }
}

/* Phase 4C: execution.continue. DEBUG_AI_DoContinue() (debug.cpp) performs
 * the actual transition -- the EXACT SAME code path as the debugger GUI's
 * "RUN" command (ParseCommand() now calls the same function instead of
 * duplicating its body). By the time it returns, DOSBOX_SetNormalLoop()
 * has already been called, so guest execution resumes on the very next
 * DOSBOX_RunMachine() iteration; nothing here waits for that or reads
 * post-resume CPU state (which would be racy -- the CPU is now free to
 * change it at any time), matching Phase4C.md section 2's "do not merely
 * return success" -- the resume is real and already in effect, just not
 * separately re-verified inside this response the way pause's response is. */
static std::string ExecExecutionContinue(long long id) {
    DEBUG_AI_DoContinue();
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"stopped\":false,\"running\":true}}", id);
    return std::string(buf);
}

/* Phase 4D: execution.step_into. DEBUG_AI_DoStepInto() (debug.cpp) executes
 * exactly one guest instruction via the SAME DEBUG_Run(1,true) the debugger
 * GUI's F11 key already uses, and returns only once that instruction has
 * genuinely finished and the debugger is stopped again -- so, unlike
 * execution.continue, it's always safe to read fresh post-step state
 * immediately via ExecDebugStatus(), the SAME builder debug.status/
 * execution.pause already use, rather than a second "here is the new
 * state" payload. */
static std::string ExecExecutionStepInto(long long id) {
    DEBUG_AI_DoStepInto();
    return ExecDebugStatus(id);
}

/* Phase 4D: execution.step_over. DEBUG_AI_DoStepOver() (debug.cpp) performs
 * the actual step -- the EXACT SAME StepOver()+DEBUG_Run() the debugger
 * GUI's F10 key already uses. For an ordinary instruction (becameAsync ==
 * false), it already fell through to a synchronous step-into by the time
 * this returns, so the new state can be read immediately, same as
 * ExecExecutionStepInto() above. For a call/int/loop/rep instruction
 * (becameAsync == true), DEBUG_Run(1,false) already handed control to
 * Normal_Loop() -- exactly like execution.continue -- so there is no new
 * "stopped" state to read yet; this registers the request with
 * DEBUG_AI_RequestStepCompletion() (see "Single-step control" above,
 * debug_ai.h) instead of returning a response string, signalled to
 * DEBUG_AI_Poll() by returning an EMPTY string (never a valid response
 * body otherwise) so it knows not to mark this connection's response ready
 * yet -- DEBUG_AI_CompletePendingSteps() will do that once the debugger
 * regains control. */
static std::string ExecExecutionStepOver(const AIRequestItem &item) {
    bool becameAsync = false;
    DEBUG_AI_DoStepOver(becameAsync);
    if (!becameAsync)
        return ExecDebugStatus(item.id);

    DEBUG_AI_RequestStepCompletion(item.conn, item.id);
    return std::string();
}

/* Phase 7B: the "stopped" half of input.mouse.capture.get/.set's dual
 * route -- see "Mouse capture & absolute input (Phase 7B)" above and
 * docs/phase7b-mouse-capture-and-absolute-input-design.md. Both just call
 * the SAME shared helpers the "running" route (DEBUG_AI_CheckPendingInput())
 * uses, so the two routes can never disagree. */
static std::string ExecMouseCaptureGet(long long id) {
    return BuildMouseCaptureStatusResult(id);
}

static std::string ExecMouseCaptureSet(long long id, bool desired) {
    return ApplyMouseCaptureSet(id, desired);
}

static std::string ExecCpuGet(long long id) {
    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{"
        "\"eax\":\"%08X\",\"ebx\":\"%08X\",\"ecx\":\"%08X\",\"edx\":\"%08X\","
        "\"esi\":\"%08X\",\"edi\":\"%08X\",\"ebp\":\"%08X\",\"esp\":\"%08X\","
        "\"cs\":\"%04X\",\"ds\":\"%04X\",\"es\":\"%04X\",\"ss\":\"%04X\",\"fs\":\"%04X\",\"gs\":\"%04X\","
        "\"eip\":\"%04X\",\"eflags\":\"%08X\""
        "}}",
        id,
        (unsigned)reg_eax, (unsigned)reg_ebx, (unsigned)reg_ecx, (unsigned)reg_edx,
        (unsigned)reg_esi, (unsigned)reg_edi, (unsigned)reg_ebp, (unsigned)reg_esp,
        (unsigned)SegValue(cs), (unsigned)SegValue(ds), (unsigned)SegValue(es), (unsigned)SegValue(ss),
        (unsigned)SegValue(fs), (unsigned)SegValue(gs),
        (unsigned)(reg_eip & 0xFFFF), (unsigned)reg_flags);
    return std::string(buf);
}

static std::string ExecCodeCurrent(long long id) {
    uint16_t curCS  = SegValue(cs);
    uint32_t curEIP = reg_eip;
    uint32_t physAddr = (uint32_t)GetAddress(curCS, curEIP);

    char dline[256];
    dline[0] = 0;
    Bitu size = DasmI386(dline, (PhysPt)physAddr, curEIP, cpu.code.big);
    if (size == 0) size = 1;
    std::string bytesHex = ReadInstructionBytesHex(physAddr, size);

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"address\":\"%04X:%04X\",\"bytes\":\"%s\",\"instruction\":\"%s\"}}",
        id, (unsigned)curCS, (unsigned)(curEIP & 0xFFFF), bytesHex.c_str(), JsonEscape(dline).c_str());
    return std::string(buf);
}

static std::string ExecMemoryRead(long long id, uint16_t seg, uint32_t off, long long length) {
    uint32_t physAddr = (uint32_t)GetAddress(seg, off);

    std::string bytesArray = "[";
    for (long long k = 0; k < length; k++) {
        uint8_t value = 0;
        bool fault = mem_readb_checked((LinearPt)(physAddr + (uint32_t)k), &value);
        if (fault) {
            char buf[320];
            snprintf(buf, sizeof(buf),
                "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"MEMORY_ERROR\","
                "\"message\":\"unmapped or inaccessible memory at offset %lld of the requested read\"}}",
                id, k);
            return std::string(buf);
        }
        char b[8];
        snprintf(b, sizeof(b), "%s\"%02X\"", k ? "," : "", value);
        bytesArray += b;
    }
    bytesArray += "]";

    char head[128];
    snprintf(head, sizeof(head),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"address\":\"%04X:%04X\",\"length\":%lld,\"bytes\":",
        id, (unsigned)seg, (unsigned)(off & 0xFFFF), length);

    std::string result;
    result.reserve(bytesArray.size() + 128);
    result += head;
    result += bytesArray;
    result += "}}";
    return result;
}

static std::string ExecMemoryWrite(long long id, uint16_t seg, uint32_t off, const std::vector<uint8_t> &data) {
    uint32_t physAddr = (uint32_t)GetAddress(seg, off);

    for (size_t k = 0; k < data.size(); k++) {
        bool fault = mem_writeb_checked((LinearPt)(physAddr + (uint32_t)k), data[k]);
        if (fault) {
            char buf[320];
            snprintf(buf, sizeof(buf),
                "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"MEMORY_ERROR\","
                "\"message\":\"unmapped or inaccessible memory at offset %lld of the requested write\"}}",
                id, (long long)k);
            return std::string(buf);
        }
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"address\":\"%04X:%04X\",\"length\":%lld}}",
        id, (unsigned)seg, (unsigned)(off & 0xFFFF), (long long)data.size());
    return std::string(buf);
}

static std::string ExecRegisterWrite(long long id, const std::string &regName, uint32_t value) {
    /* regName has already been checked against WRITABLE_REGISTERS by
     * HandleLine() (socket thread) before this ever reaches the queue --
     * this is the actual write, on the emulator thread, via the same
     * reg_* macros the rest of src/debug/ uses. */
    if (regName == "eax") reg_eax = value;
    else if (regName == "ebx") reg_ebx = value;
    else if (regName == "ecx") reg_ecx = value;
    else if (regName == "edx") reg_edx = value;
    else if (regName == "esi") reg_esi = value;
    else if (regName == "edi") reg_edi = value;
    else if (regName == "ebp") reg_ebp = value;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"register\":\"%s\",\"value\":\"%08X\"}}",
        id, regName.c_str(), (unsigned)value);
    return std::string(buf);
}

static std::string ExecCodeDisassemble(long long id, uint16_t seg, uint32_t off, long long count) {
    std::string arr = "[";
    uint32_t curOff = off;
    for (long long k = 0; k < count; k++) {
        uint32_t physAddr = (uint32_t)GetAddress(seg, curOff);

        char dline[256];
        dline[0] = 0;
        Bitu size = DasmI386(dline, (PhysPt)physAddr, curOff, cpu.code.big);
        if (size == 0) size = 1;
        std::string bytesHex = ReadInstructionBytesHex(physAddr, size);

        char item[512];
        snprintf(item, sizeof(item), "%s{\"address\":\"%04X:%04X\",\"bytes\":\"%s\",\"instruction\":\"%s\"}",
            k ? "," : "", (unsigned)seg, (unsigned)(curOff & 0xFFFF), bytesHex.c_str(), JsonEscape(dline).c_str());
        arr += item;

        curOff = (curOff + (uint32_t)size) & 0xFFFF;
    }
    arr += "]";

    std::string result;
    char head[64];
    snprintf(head, sizeof(head), "{\"id\":%lld,\"ok\":true,\"result\":", id);
    result += head;
    result += arr;
    result += "}";
    return result;
}

static std::string ExecBreakpointSet(long long id, uint16_t seg, uint32_t off) {
    /* Bridge-level policy on top of the existing mechanism: reject a
     * second breakpoint at an address that already has one, using the
     * existing IsBreakpoint() check. (The GUI's own "BP" command has no
     * such guard and will happily stack duplicates -- this is a
     * deliberate, documented AI-facing API choice, not a change to how
     * the debugger itself behaves; see docs/dosbox-ai-bridge.md.) */
    if (DEBUG_AI_BreakpointExists(seg, off)) {
        char buf[320];
        snprintf(buf, sizeof(buf),
            "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"BREAKPOINT_ALREADY_EXISTS\","
            "\"message\":\"a breakpoint already exists at %04X:%04X\"}}",
            id, (unsigned)seg, (unsigned)(off & 0xFFFF));
        return std::string(buf);
    }

    int bpIndex = DEBUG_AI_BreakpointAdd(seg, off);
    if (bpIndex < 0) {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"failed to create breakpoint\"}}",
            id);
        return std::string(buf);
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"id\":%d,\"address\":\"%04X:%04X\",\"enabled\":true}}",
        id, bpIndex, (unsigned)seg, (unsigned)(off & 0xFFFF));
    return std::string(buf);
}

static std::string ExecProtectedMemoryBreakpointSet(long long id, uint16_t selector, uint32_t off) {
    int bpIndex = DEBUG_AI_ProtectedMemoryBreakpointAdd(selector, off);
    if (bpIndex < 0) {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"failed to create protected-memory breakpoint\"}}",
            id);
        return std::string(buf);
    }

    char buf[288];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"id\":%d,\"address\":\"%04X:%04X\",\"type\":\"protected_memory\",\"enabled\":true}}",
        id, bpIndex, (unsigned)selector, (unsigned)(off & 0xFFFF));
    return std::string(buf);
}

static std::string ExecRealMemoryBreakpointSet(long long id, uint16_t seg, uint32_t off) {
    int bpIndex = DEBUG_AI_RealMemoryBreakpointAdd(seg, off);
    if (bpIndex < 0) {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"failed to create real-mode memory breakpoint\"}}",
            id);
        return std::string(buf);
    }
    char buf[272];
    snprintf(buf, sizeof(buf),
        "{\"id\":%lld,\"ok\":true,\"result\":{\"id\":%d,\"address\":\"%04X:%04X\",\"type\":\"memory\",\"enabled\":true}}",
        id, bpIndex, (unsigned)seg, (unsigned)(off & 0xFFFF));
    return std::string(buf);
}

static std::string ExecBreakpointDelete(long long id, long long index) {
    bool ok = (index >= 0 && index <= 0xFFFF) && DEBUG_AI_BreakpointDelete((uint16_t)index);
    if (!ok) {
        char buf[256];
        snprintf(buf, sizeof(buf),
            "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"BREAKPOINT_NOT_FOUND\",\"message\":\"no breakpoint with id %lld\"}}",
            id, index);
        return std::string(buf);
    }
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"id\":%lld,\"ok\":true,\"result\":{\"id\":%lld,\"deleted\":true}}", id, index);
    return std::string(buf);
}

static std::string ExecBreakpointList(long long id) {
    /* enabled is always true: DOSBox-X's debugger has no enable/disable-
     * without-delete concept for breakpoints (no BPENA/BPDIS command
     * exists) -- a breakpoint either exists in BPoints or it doesn't.
     * CBreakpoint::IsActive() reflects a transient, low-level detail (is
     * the 0xCC currently patched into guest memory right now) that is
     * re-synchronized around each RUN/continue and would be misleading
     * exposed here as "enabled". */
    uint16_t count = DEBUG_AI_BreakpointCount();
    std::string arr = "[";
    bool first = true;
    for (uint16_t i = 0; i < count; i++) {
        bool isPhysical = false;
        bool isRealMemory = false;
        bool isProtectedMemory = false;
        uint16_t seg = 0;
        uint32_t off = 0;
        if (!DEBUG_AI_BreakpointInfo(i, isPhysical, isRealMemory,
                                     isProtectedMemory, seg, off)) continue;
        if (!isPhysical && !isRealMemory && !isProtectedMemory) continue;

        char item[176];
        snprintf(item, sizeof(item), "%s{\"id\":%u,\"address\":\"%04X:%04X\",\"type\":\"%s\",\"enabled\":true}",
            first ? "" : ",", (unsigned)i, (unsigned)seg, (unsigned)(off & 0xFFFF),
            isProtectedMemory ? "protected_memory" : (isRealMemory ? "memory" : "code"));
        arr += item;
        first = false;
    }
    arr += "]";

    std::string result;
    char head[64];
    snprintf(head, sizeof(head), "{\"id\":%lld,\"ok\":true,\"result\":{\"breakpoints\":", id);
    result += head;
    result += arr;
    result += "}}";
    return result;
}

static std::string ExecuteRequest(const AIRequestItem &item) {
    switch (item.method) {
        case AIMethod::DebugStatus:      return ExecDebugStatus(item.id);
        case AIMethod::CpuGet:           return ExecCpuGet(item.id);
        case AIMethod::CodeCurrent:      return ExecCodeCurrent(item.id);
        case AIMethod::MemoryRead:       return ExecMemoryRead(item.id, item.seg, item.off, item.length);
        case AIMethod::CodeDisassemble:  return ExecCodeDisassemble(item.id, item.seg, item.off, item.count);
        case AIMethod::MemoryWrite:      return ExecMemoryWrite(item.id, item.seg, item.off, item.writeBytes);
        case AIMethod::RegisterWrite:    return ExecRegisterWrite(item.id, item.regName, item.regValue);
        case AIMethod::BreakpointSet:     return ExecBreakpointSet(item.id, item.seg, item.off);
        case AIMethod::ProtectedMemoryBreakpointSet: return ExecProtectedMemoryBreakpointSet(item.id, item.seg, item.off);
        case AIMethod::RealMemoryBreakpointSet: return ExecRealMemoryBreakpointSet(item.id, item.seg, item.off);
        case AIMethod::BreakpointDelete:  return ExecBreakpointDelete(item.id, item.breakpointIndex);
        case AIMethod::BreakpointList:    return ExecBreakpointList(item.id);
        case AIMethod::ExecutionContinue: return ExecExecutionContinue(item.id);
        case AIMethod::ExecutionStepInto: return ExecExecutionStepInto(item.id);
        case AIMethod::ExecutionStepOver: return ExecExecutionStepOver(item);
        case AIMethod::MouseCaptureGet:   return ExecMouseCaptureGet(item.id);
        case AIMethod::MouseCaptureSet:   return ExecMouseCaptureSet(item.id, item.mouseCaptureDesired);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"INTERNAL_ERROR\",\"message\":\"unhandled method\"}}", item.id);
    return std::string(buf);
}

void DEBUG_AI_Poll(void) {
    for (;;) {
        AIRequestItem item;
        {
            std::lock_guard<std::mutex> lk(g_queueMutex);
            if (g_requestQueue.empty()) break;
            item = g_requestQueue.front();
            g_requestQueue.pop_front();
        }

        std::string responseLine = ExecuteRequest(item);

        if (responseLine.empty()) {
            /* Phase 4D: ExecExecutionStepOver() returns empty ONLY when it
             * took the async call/int/loop/rep path and has already
             * registered this connection with DEBUG_AI_RequestStepCompletion()
             * -- DEBUG_AI_CompletePendingSteps() (called from DEBUG_Loop(),
             * once the debugger regains control) completes it later. Do not
             * mark the response ready here. */
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(item.conn->mtx);
            item.conn->responseLine = responseLine;
            item.conn->responseReady = true;
        }
        item.conn->cv.notify_all();
    }
}

/* ------------------------------------------------------------------ */
/* Socket-thread side: accept / read / parse / validate / send only.   */
/* Nothing below this line touches cpu_regs, Segs, guest memory, or    */
/* any other emulator/debugger state.                                  */
/* ------------------------------------------------------------------ */

static void SendLine(const std::shared_ptr<AIConnection> &conn, const std::string &line) {
    if (conn->sock == AI_INVALID_SOCKET) return;
    std::string out = line;
    out += "\n";
    size_t sent = 0;
    while (sent < out.size()) {
        int n = send(conn->sock, out.c_str() + sent, (int)(out.size() - sent), 0);
        if (n <= 0) return;
        sent += (size_t)n;
    }
}

static void HandleLine(const std::shared_ptr<AIConnection> &conn, const std::string &line) {
    if (line.size() > MAX_LINE_LENGTH) {
        LOG(LOG_MISC, LOG_WARN)("AI bridge: rejected oversized request (%u bytes)", (unsigned)line.size());
        SendLine(conn, "{\"id\":null,\"ok\":false,\"error\":{\"code\":\"INVALID_REQUEST\",\"message\":\"request exceeds maximum size\"}}");
        return;
    }

    JsonValue root;
    JsonParser parser(line);
    if (!parser.Parse(root) || root.type != JsonValue::Type::Object) {
        LOG(LOG_MISC, LOG_WARN)("AI bridge: malformed JSON request");
        SendLine(conn, "{\"id\":null,\"ok\":false,\"error\":{\"code\":\"INVALID_JSON\",\"message\":\"malformed JSON request\"}}");
        return;
    }

    long long id = 0;
    bool hasId = false;
    auto itId = root.object.find("id");
    if (itId != root.object.end() && itId->second.type == JsonValue::Type::Number) {
        id = (long long)itId->second.number;
        hasId = true;
    }

    auto RespondError = [&](const char *code, const std::string &msg) {
        char buf[512];
        if (hasId)
            snprintf(buf, sizeof(buf), "{\"id\":%lld,\"ok\":false,\"error\":{\"code\":\"%s\",\"message\":\"%s\"}}",
                     id, code, JsonEscape(msg).c_str());
        else
            snprintf(buf, sizeof(buf), "{\"id\":null,\"ok\":false,\"error\":{\"code\":\"%s\",\"message\":\"%s\"}}",
                     code, JsonEscape(msg).c_str());
        LOG(LOG_MISC, LOG_WARN)("AI bridge: %s: %s", code, msg.c_str());
        SendLine(conn, buf);
    };

    if (!hasId) { RespondError("INVALID_REQUEST", "missing or non-numeric \"id\""); return; }

    auto itMethod = root.object.find("method");
    if (itMethod == root.object.end() || itMethod->second.type != JsonValue::Type::String) {
        RespondError("INVALID_REQUEST", "missing \"method\"");
        return;
    }
    const std::string &method = itMethod->second.str;

    const JsonValue *params = nullptr;
    auto itParams = root.object.find("params");
    if (itParams != root.object.end() && itParams->second.type == JsonValue::Type::Object)
        params = &itParams->second;

    AIRequestItem item;
    item.conn = conn;
    item.id = id;

    if (method == "debug.status") {
        if (!g_debuggerActive.load()) {
            /* Guest code is currently running freely under Normal_Loop()
             * (dosbox.cpp) -- DEBUG_Loop()/DEBUG_AI_Poll() are not running
             * at all right now, so there is no live register/instruction
             * snapshot to safely read (doing so from this socket thread
             * would touch cpu_regs/Segs directly, exactly what the
             * request-queue design exists to avoid). Rather than queueing
             * this and waiting up to REQUEST_TIMEOUT_SECONDS only to time
             * out, answer immediately using ONLY the atomic activity flag
             * (bridge-level metadata, not emulator state) -- Phase 4C's
             * "the reported state must originate from the actual DOSBox-X
             * execution/debugger state" is satisfied because this IS that
             * state (the real loop-handler identity), just without the
             * fields that require the debugger to be stopped to read
             * safely. */
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"id\":%lld,\"ok\":true,\"result\":{\"stopped\":false,\"running\":true}}", id);
            SendLine(conn, buf);
            return;
        }
        item.method = AIMethod::DebugStatus;
    } else if (method == "execution.continue") {
        if (!g_debuggerActive.load()) {
            RespondError("ALREADY_RUNNING",
                "guest execution is already running (the debugger is not currently stopped)");
            return;
        }
        item.method = AIMethod::ExecutionContinue;
    } else if (method == "execution.step_into") {
        if (!g_debuggerActive.load()) {
            RespondError("ALREADY_RUNNING",
                "cannot step: guest execution is already running (the debugger is not currently stopped)");
            return;
        }
        item.method = AIMethod::ExecutionStepInto;
    } else if (method == "execution.step_over") {
        if (!g_debuggerActive.load()) {
            RespondError("ALREADY_RUNNING",
                "cannot step: guest execution is already running (the debugger is not currently stopped)");
            return;
        }
        /* Unlike execution.pause, kicking off a step requires real
         * emulator-thread work (DEBUG_AI_DoStepOver()), so this goes
         * through the SAME g_requestQueue/DEBUG_AI_Poll() mechanism every
         * other method here uses (falls through to the shared enqueue+wait
         * tail below) rather than being handled entirely on this socket
         * thread the way execution.pause is. If the current instruction is
         * call/int/loop/rep, DEBUG_AI_Poll() will not complete this
         * connection's response immediately -- see ExecExecutionStepOver()
         * and DEBUG_AI_CompletePendingSteps() -- so the wait below may run
         * the full REQUEST_TIMEOUT_SECONDS if the stepped-over call takes
         * that long to return; the shared tail's timeout handling sends
         * EXECUTION_TIMEOUT (not DEBUGGER_NOT_STOPPED) for this method,
         * since the debugger genuinely was entered and a step genuinely
         * was kicked off, it just hasn't completed yet. */
        item.method = AIMethod::ExecutionStepOver;
    } else if (method == "execution.pause") {
        if (g_debuggerActive.load()) {
            RespondError("ALREADY_STOPPED", "the DOSBox-X debugger is already stopped");
            return;
        }
        /* Cannot go through g_requestQueue/DEBUG_AI_Poll() -- that only
         * drains while DEBUG_Loop() is the active loop, which is exactly
         * the state we don't have. Register instead with the pending-pause
         * list Normal_Loop's existing per-iteration hook consumes (see
         * DEBUG_AI_CheckPauseRequest(), debug.cpp) and block on this same
         * connection's response slot -- the same wait pattern as every
         * other method below, just fed from a different producer. */
        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
        {
            std::lock_guard<std::mutex> lk(conn->mtx);
            conn->responseReady = false;
        }
        DEBUG_AI_RequestPause(conn, id);

        std::unique_lock<std::mutex> lk(conn->mtx);
        conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                           [&] { return conn->responseReady || conn->stopping.load(); });

        if (conn->responseReady) {
            std::string resp = conn->responseLine;
            lk.unlock();
            LOG(LOG_MISC, LOG_DEBUG)("AI bridge: response id=%lld", id);
            SendLine(conn, resp);
        } else {
            lk.unlock();
            DEBUG_AI_CancelPause(conn);
            if (!conn->stopping.load())
                RespondError("EXECUTION_TIMEOUT",
                    "pause_execution did not complete within the timeout");
        }
        return;
    } else if (method == "cpu.get") {
        item.method = AIMethod::CpuGet;
    } else if (method == "code.current") {
        item.method = AIMethod::CodeCurrent;
    } else if (method == "memory.read") {
        item.method = AIMethod::MemoryRead;
        if (!params) { RespondError("INVALID_PARAMETER", "memory.read requires \"params\""); return; }
        auto itAddr = params->object.find("address");
        auto itLen  = params->object.find("length");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (itLen == params->object.end() || itLen->second.type != JsonValue::Type::Number) {
            RespondError("INVALID_PARAMETER", "params.length must be a number"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_PARAMETER", "malformed address, expected \"SEG:OFF\" hex"); return;
        }
        long long len = (long long)itLen->second.number;
        if (len <= 0 || len > MAX_READ_LENGTH) {
            RespondError("INVALID_PARAMETER", "length out of range"); return;
        }
        item.length = len;
    } else if (method == "code.disassemble") {
        item.method = AIMethod::CodeDisassemble;
        if (!params) { RespondError("INVALID_PARAMETER", "code.disassemble requires \"params\""); return; }
        auto itAddr  = params->object.find("address");
        auto itCount = params->object.find("count");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (itCount == params->object.end() || itCount->second.type != JsonValue::Type::Number) {
            RespondError("INVALID_PARAMETER", "params.count must be a number"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_PARAMETER", "malformed address, expected \"SEG:OFF\" hex"); return;
        }
        long long cnt = (long long)itCount->second.number;
        if (cnt <= 0 || cnt > MAX_DISASSEMBLE_COUNT) {
            RespondError("INVALID_PARAMETER", "count out of range"); return;
        }
        item.count = cnt;
    } else if (method == "memory.write") {
        item.method = AIMethod::MemoryWrite;
        if (!params) { RespondError("INVALID_PARAMETER", "memory.write requires \"params\""); return; }
        auto itAddr = params->object.find("address");
        auto itData = params->object.find("data");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (itData == params->object.end() || itData->second.type != JsonValue::Type::Array) {
            RespondError("INVALID_PARAMETER", "params.data must be an array of byte values"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_PARAMETER", "malformed address, expected \"SEG:OFF\" hex"); return;
        }
        const std::vector<JsonValue> &arr = itData->second.array;
        if (arr.empty() || (long long)arr.size() > MAX_WRITE_LENGTH) {
            RespondError("INVALID_PARAMETER", "data length out of range"); return;
        }
        std::vector<uint8_t> bytes;
        bytes.reserve(arr.size());
        for (const JsonValue &elem : arr) {
            long parsedByte = -1;
            if (elem.type == JsonValue::Type::Number) {
                parsedByte = (long)elem.number;
            } else if (elem.type == JsonValue::Type::String) {
                char *endp = nullptr;
                parsedByte = strtol(elem.str.c_str(), &endp, 16);
                if (endp == elem.str.c_str() || *endp != '\0') parsedByte = -1;
            }
            if (parsedByte < 0 || parsedByte > 0xFF) {
                RespondError("INVALID_PARAMETER", "data must be byte values 0-255 (numbers or hex strings)");
                return;
            }
            bytes.push_back((uint8_t)parsedByte);
        }
        item.writeBytes = std::move(bytes);
    } else if (method == "register.write") {
        item.method = AIMethod::RegisterWrite;
        if (!params) { RespondError("INVALID_PARAMETER", "register.write requires \"params\""); return; }
        auto itReg = params->object.find("register");
        auto itVal = params->object.find("value");
        if (itReg == params->object.end() || itReg->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.register must be a string"); return;
        }
        if (itVal == params->object.end() || itVal->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.value must be a hex string"); return;
        }

        std::string regName = itReg->second.str;
        std::transform(regName.begin(), regName.end(), regName.begin(),
                        [](unsigned char c) { return (char)tolower(c); });

        bool isWritable = false;
        for (const char *w : WRITABLE_REGISTERS) if (regName == w) { isWritable = true; break; }
        if (!isWritable) {
            bool isBlocked = false;
            for (const char *b : BLOCKED_REGISTERS) if (regName == b) { isBlocked = true; break; }
            if (isBlocked)
                RespondError("REGISTER_NOT_WRITABLE",
                    "writing \"" + regName + "\" is not permitted (EIP/segment registers/ESP/EFLAGS are protected)");
            else
                RespondError("INVALID_PARAMETER", "unknown register \"" + regName + "\"");
            return;
        }

        const std::string &valStr = itVal->second.str;
        if (valStr.empty() || valStr.size() > 8) {
            RespondError("INVALID_PARAMETER", "params.value must be 1-8 hex digits"); return;
        }
        for (char c : valStr) {
            if (!isxdigit((unsigned char)c)) {
                RespondError("INVALID_PARAMETER", "params.value must be a hex string"); return;
            }
        }

        item.regName = regName;
        item.regValue = (uint32_t)strtoul(valStr.c_str(), nullptr, 16);
    } else if (method == "breakpoint.set") {
        item.method = AIMethod::BreakpointSet;
        if (!params) { RespondError("INVALID_PARAMETER", "breakpoint.set requires \"params\""); return; }
        auto itAddr = params->object.find("address");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_ADDRESS", "malformed address, expected \"SEG:OFF\" hex"); return;
        }
    } else if (method == "breakpoint.memory.set") {
        item.method = AIMethod::ProtectedMemoryBreakpointSet;
        if (!params) { RespondError("INVALID_PARAMETER", "breakpoint.memory.set requires \"params\""); return; }
        auto itAddr = params->object.find("address");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_ADDRESS", "malformed address, expected \"SELECTOR:OFFSET\" hex"); return;
        }
    } else if (method == "breakpoint.memory.real.set") {
        item.method = AIMethod::RealMemoryBreakpointSet;
        if (!params) { RespondError("INVALID_PARAMETER", "breakpoint.memory.real.set requires \"params\""); return; }
        auto itAddr = params->object.find("address");
        if (itAddr == params->object.end() || itAddr->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.address must be a string"); return;
        }
        if (!ParseAddress(itAddr->second.str, item.seg, item.off)) {
            RespondError("INVALID_ADDRESS", "malformed address, expected \"SEG:OFFSET\" hex"); return;
        }
    } else if (method == "breakpoint.delete") {
        item.method = AIMethod::BreakpointDelete;
        if (!params) { RespondError("INVALID_PARAMETER", "breakpoint.delete requires \"params\""); return; }
        auto itBpId = params->object.find("id");
        if (itBpId == params->object.end() || itBpId->second.type != JsonValue::Type::Number) {
            RespondError("INVALID_PARAMETER", "params.id must be a number"); return;
        }
        long long bpIndex = (long long)itBpId->second.number;
        if (bpIndex < 0 || bpIndex > 0xFFFF) {
            RespondError("INVALID_PARAMETER", "params.id out of range"); return;
        }
        item.breakpointIndex = bpIndex;
    } else if (method == "breakpoint.list") {
        item.method = AIMethod::BreakpointList;
    } else if (method == "input.key.down" || method == "input.key.up" || method == "input.key.tap") {
        /* Cannot go through g_requestQueue/DEBUG_AI_Poll() -- KEYBOARD_AddKey()
         * must run on the emulator thread, which only reaches the hook that
         * drains this (DEBUG_AI_CheckPendingInput(), called from
         * Normal_Loop(), dosbox.cpp) while guest code is actually running,
         * i.e. exactly while the debugger is NOT stopped -- the opposite
         * precondition of every g_requestQueue method above. Same wait
         * pattern as execution.pause, just fed from g_pendingInputs. */
        if (g_debuggerActive.load()) {
            RespondError("DEBUGGER_STOPPED",
                "cannot inject input while the DOSBox-X debugger is stopped; call execution.continue first");
            return;
        }
        if (!params) { RespondError("INVALID_PARAMETER", method + " requires \"params\""); return; }
        auto itKey = params->object.find("key");
        if (itKey == params->object.end() || itKey->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.key must be a string"); return;
        }
        KBD_KEYS key;
        if (!ParseKeyName(itKey->second.str, key)) {
            RespondError("INVALID_PARAMETER", "unrecognized params.key \"" + itKey->second.str + "\"");
            return;
        }

        PendingInputRequest req;
        req.op  = (method == "input.key.down") ? AIInputOp::KeyDown
                : (method == "input.key.up")   ? AIInputOp::KeyUp
                                                : AIInputOp::KeyTap;
        req.conn = conn;
        req.id = id;
        req.key = key;

        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
        {
            std::lock_guard<std::mutex> lk(conn->mtx);
            conn->responseReady = false;
        }
        DEBUG_AI_RequestInput(req);

        std::unique_lock<std::mutex> lk(conn->mtx);
        conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                           [&] { return conn->responseReady || conn->stopping.load(); });
        if (conn->responseReady) {
            std::string resp = conn->responseLine;
            lk.unlock();
            SendLine(conn, resp);
        } else {
            lk.unlock();
            DEBUG_AI_CancelInput(conn, id);
            if (!conn->stopping.load())
                RespondError("EXECUTION_TIMEOUT", method + " did not complete within the timeout");
        }
        return;
    } else if (method == "input.mouse.move_relative" || method == "input.mouse.button.set" ||
               method == "input.mouse.button.click" || method == "input.release_all") {
        if (g_debuggerActive.load()) {
            RespondError("DEBUGGER_STOPPED",
                "cannot inject input while the DOSBox-X debugger is stopped; call execution.continue first");
            return;
        }

        PendingInputRequest req;
        req.conn = conn;
        req.id = id;

        if (method == "input.mouse.move_relative") {
            if (!params) { RespondError("INVALID_PARAMETER", method + " requires \"params\""); return; }
            auto itDx = params->object.find("dx");
            auto itDy = params->object.find("dy");
            if (itDx == params->object.end() || itDx->second.type != JsonValue::Type::Number ||
                itDy == params->object.end() || itDy->second.type != JsonValue::Type::Number) {
                RespondError("INVALID_PARAMETER", "params.dx and params.dy must both be numbers"); return;
            }
            req.op = AIInputOp::MouseMoveRelative;
            req.dx = (float)itDx->second.number;
            req.dy = (float)itDy->second.number;
        } else if (method == "input.release_all") {
            req.op = AIInputOp::ReleaseAll;
        } else {
            if (!params) { RespondError("INVALID_PARAMETER", method + " requires \"params\""); return; }
            auto itButton = params->object.find("button");
            if (itButton == params->object.end() || itButton->second.type != JsonValue::Type::Number) {
                RespondError("INVALID_PARAMETER", "params.button must be a number"); return;
            }
            long long button = (long long)itButton->second.number;
            if (button < 0 || button > 2) {
                RespondError("INVALID_PARAMETER", "params.button must be 0 (left), 1 (right), or 2 (middle)");
                return;
            }
            req.button = (uint8_t)button;

            if (method == "input.mouse.button.set") {
                auto itPressed = params->object.find("pressed");
                if (itPressed == params->object.end() || itPressed->second.type != JsonValue::Type::Bool) {
                    RespondError("INVALID_PARAMETER", "params.pressed must be a boolean"); return;
                }
                req.op = AIInputOp::MouseButtonSet;
                req.pressed = itPressed->second.number != 0;
            } else {
                req.op = AIInputOp::MouseButtonClick;
            }
        }

        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
        {
            std::lock_guard<std::mutex> lk(conn->mtx);
            conn->responseReady = false;
        }
        DEBUG_AI_RequestInput(req);

        std::unique_lock<std::mutex> lk(conn->mtx);
        conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                           [&] { return conn->responseReady || conn->stopping.load(); });
        if (conn->responseReady) {
            std::string resp = conn->responseLine;
            lk.unlock();
            SendLine(conn, resp);
        } else {
            lk.unlock();
            DEBUG_AI_CancelInput(conn, id);
            if (!conn->stopping.load())
                RespondError("EXECUTION_TIMEOUT", method + " did not complete within the timeout");
        }
        return;
    } else if (method == "input.mouse.capture.get" || method == "input.mouse.capture.set") {
        /* Phase 7B: unlike every OTHER input.* method, capture status/
         * toggle is meaningful whether the debugger is stopped or running
         * -- GFX_CaptureMouse() only touches SDL/window state on the main
         * thread, never cpu_regs/Segs/guest memory, so there is nothing
         * unsafe about servicing it while stopped. This branch therefore
         * picks ONE of two different routes at runtime instead of always
         * doing one or the other like every branch around it -- see
         * "Mouse capture & absolute input (Phase 7B)" above and docs/
         * phase7b-mouse-capture-and-absolute-input-design.md. */
        bool wantSet = (method == "input.mouse.capture.set");
        bool desired = false;
        if (wantSet) {
            if (!params) { RespondError("INVALID_PARAMETER", "input.mouse.capture.set requires \"params\""); return; }
            auto itCaptured = params->object.find("captured");
            if (itCaptured == params->object.end() || itCaptured->second.type != JsonValue::Type::Bool) {
                RespondError("INVALID_PARAMETER", "params.captured must be a boolean"); return;
            }
            desired = itCaptured->second.number != 0;
        }

        if (g_debuggerActive.load()) {
            /* Stopped: fall through to the common g_requestQueue push
             * below, the SAME route debug.status/cpu.get/etc. already
             * use -- mirrors how the "breakpoint.list" branch above just
             * sets item.method and falls through rather than returning. */
            item.method = wantSet ? AIMethod::MouseCaptureSet : AIMethod::MouseCaptureGet;
            item.mouseCaptureDesired = desired;
        } else {
            /* Running: Normal_Loop() never calls DEBUG_AI_Poll(), so route
             * through g_pendingInputs/DEBUG_AI_CheckPendingInput() instead,
             * the SAME mechanism input.mouse.move_relative etc. use. */
            PendingInputRequest req;
            req.op = wantSet ? AIInputOp::MouseCaptureSet : AIInputOp::MouseCaptureGet;
            req.conn = conn;
            req.id = id;
            req.captureDesired = desired;

            LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
            {
                std::lock_guard<std::mutex> lk(conn->mtx);
                conn->responseReady = false;
            }
            DEBUG_AI_RequestInput(req);

            std::unique_lock<std::mutex> lk(conn->mtx);
            conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                               [&] { return conn->responseReady || conn->stopping.load(); });
            if (conn->responseReady) {
                std::string resp = conn->responseLine;
                lk.unlock();
                SendLine(conn, resp);
            } else {
                lk.unlock();
                DEBUG_AI_CancelInput(conn, id);
                if (!conn->stopping.load())
                    RespondError("EXECUTION_TIMEOUT", method + " did not complete within the timeout");
            }
            return;
        }
    } else if (method == "input.mouse.move_absolute" || method == "input.mouse.click_at") {
        /* Unlike capture.get/.set above, these dispatch real guest input
         * (Mouse_CursorMoved()/Mouse_ButtonPressed()/Mouse_ButtonReleased()),
         * so -- like every OTHER input.* method -- they only work while
         * running, never while stopped. */
        if (g_debuggerActive.load()) {
            RespondError("DEBUGGER_STOPPED",
                "cannot inject input while the DOSBox-X debugger is stopped; call execution.continue first");
            return;
        }
        if (!params) { RespondError("INVALID_PARAMETER", method + " requires \"params\""); return; }

        auto itX = params->object.find("x");
        auto itY = params->object.find("y");
        if (itX == params->object.end() || itX->second.type != JsonValue::Type::Number ||
            itY == params->object.end() || itY->second.type != JsonValue::Type::Number) {
            RespondError("INVALID_PARAMETER", "params.x and params.y must both be numbers"); return;
        }

        auto itSpace = params->object.find("coordinate_space");
        if (itSpace == params->object.end() || itSpace->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.coordinate_space is required"); return;
        }
        bool normalized;
        if (itSpace->second.str == "normalized") normalized = true;
        else if (itSpace->second.str == "guest_pixels") normalized = false;
        else { RespondError("INVALID_PARAMETER", "params.coordinate_space must be \"guest_pixels\" or \"normalized\""); return; }

        bool clamp = false;
        auto itClamp = params->object.find("clamp");
        if (itClamp != params->object.end() && itClamp->second.type == JsonValue::Type::Bool)
            clamp = itClamp->second.number != 0;

        PendingInputRequest req;
        req.op = (method == "input.mouse.click_at") ? AIInputOp::MouseClickAt : AIInputOp::MouseMoveAbsolute;
        req.conn = conn;
        req.id = id;
        req.x = itX->second.number;
        req.y = itY->second.number;
        req.normalized = normalized;
        req.clamp = clamp;

        if (method == "input.mouse.click_at") {
            auto itButton = params->object.find("button");
            if (itButton == params->object.end() || itButton->second.type != JsonValue::Type::Number) {
                RespondError("INVALID_PARAMETER", "params.button must be a number"); return;
            }
            long long button = (long long)itButton->second.number;
            if (button < 0 || button > 2) {
                RespondError("INVALID_PARAMETER", "params.button must be 0 (left), 1 (right), or 2 (middle)");
                return;
            }
            req.button = (uint8_t)button;
        }

        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
        {
            std::lock_guard<std::mutex> lk(conn->mtx);
            conn->responseReady = false;
        }
        DEBUG_AI_RequestInput(req);

        std::unique_lock<std::mutex> lk(conn->mtx);
        conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                           [&] { return conn->responseReady || conn->stopping.load(); });
        if (conn->responseReady) {
            std::string resp = conn->responseLine;
            lk.unlock();
            SendLine(conn, resp);
        } else {
            lk.unlock();
            DEBUG_AI_CancelInput(conn, id);
            if (!conn->stopping.load())
                RespondError("EXECUTION_TIMEOUT", method + " did not complete within the timeout");
        }
        return;
    } else if (method == "input.receipt.get") {
        /* Phase 7C: answered directly here, never queued -- a receipt
         * describes something that already happened, so this needs only
         * safe concurrent access to LookupInputReceipt()'s mutex-guarded
         * history, not emulator-thread execution. Meaningful whether the
         * debugger is stopped or running. */
        if (!params) { RespondError("INVALID_PARAMETER", "input.receipt.get requires \"params\""); return; }
        auto itSeq = params->object.find("input_sequence");
        if (itSeq == params->object.end() || itSeq->second.type != JsonValue::Type::Number) {
            RespondError("INVALID_PARAMETER", "params.input_sequence must be a number"); return;
        }
        uint64_t seq = (uint64_t)itSeq->second.number;

        InputReceipt receipt;
        if (!LookupInputReceipt(seq, receipt)) {
            RespondError("INPUT_RECEIPT_EXPIRED",
                "no receipt found for that input_sequence (evicted, or never issued)");
            return;
        }

        std::string resp = "{\"id\":" + std::to_string(id) + ",\"ok\":true,\"result\":{" +
            ReceiptFieldsJson(receipt.sequence, receipt.dispatchedAtEmulatedMs) + ","
            "\"device\":\"" + receipt.device + "\","
            "\"guest_observation\":{\"kind\":null,\"observed_at_emulated_ms\":null}"
            "}}";
        SendLine(conn, resp);
        return;
    } else if (method == "video.frame.capture") {
        /* Unlike input.*, this does NOT require the debugger to be
         * running: DEBUG_AI_CheckPendingFrameCapture() (debug_ai.cpp) is
         * drained from RENDER_EndUpdate() (render.cpp), which still runs
         * while the guest is stopped IF a frame happens to render before
         * the request times out -- but in practice a fully halted guest
         * (stopped at a breakpoint) is not producing new VGA frames via
         * PIC-driven vertical retrace, so a request issued while stopped
         * will typically wait out the full timeout unless execution
         * resumes in the meantime. That is expected, documented behavior
         * (see docs/phase7a-frame-capture-design.md), not a bug -- it is
         * intentionally NOT rejected outright the way input.* methods
         * reject DEBUGGER_STOPPED, since a capture is still meaningful
         * (if slow) around a resume. */
        PendingFrameCapture req;
        req.conn = conn;
        req.id = id;

        if (!params) { RespondError("INVALID_PARAMETER", "video.frame.capture requires \"params\""); return; }
        auto itFormat = params->object.find("format");
        if (itFormat == params->object.end() || itFormat->second.type != JsonValue::Type::String) {
            RespondError("INVALID_PARAMETER", "params.format must be a string"); return;
        }
        if (itFormat->second.str == "png") req.wantPng = true;
        else if (itFormat->second.str == "rgba") req.wantPng = false;
        else { RespondError("INVALID_PARAMETER", "params.format must be \"png\" or \"rgba\""); return; }

        auto itCursor = params->object.find("include_cursor");
        if (itCursor != params->object.end() && itCursor->second.type == JsonValue::Type::Bool &&
            itCursor->second.number != 0) {
            /* Phase 7A design doc section "Open questions": cursor
             * compositing isn't implemented yet -- fail closed rather
             * than silently ignoring the request or claiming a cursor
             * that isn't really there. */
            RespondError("INVALID_PARAMETER", "include_cursor=true is not yet supported"); return;
        }

        auto itMaxW = params->object.find("max_width");
        if (itMaxW != params->object.end() && itMaxW->second.type == JsonValue::Type::Number) {
            long long mw = (long long)itMaxW->second.number;
            if (mw <= 0 || mw > 0x7FFFFFFFLL) { RespondError("INVALID_PARAMETER", "params.max_width out of range"); return; }
            req.maxWidth = (uint32_t)mw;
        }
        auto itMaxH = params->object.find("max_height");
        if (itMaxH != params->object.end() && itMaxH->second.type == JsonValue::Type::Number) {
            long long mh = (long long)itMaxH->second.number;
            if (mh <= 0 || mh > 0x7FFFFFFFLL) { RespondError("INVALID_PARAMETER", "params.max_height out of range"); return; }
            req.maxHeight = (uint32_t)mh;
        }

        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());
        {
            std::lock_guard<std::mutex> lk(conn->mtx);
            conn->responseReady = false;
        }
        DEBUG_AI_RequestFrameCapture(req);

        std::unique_lock<std::mutex> lk(conn->mtx);
        conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                           [&] { return conn->responseReady || conn->stopping.load(); });
        if (conn->responseReady) {
            std::string resp = conn->responseLine;
            lk.unlock();
            SendLine(conn, resp);
        } else {
            lk.unlock();
            DEBUG_AI_CancelFrameCapture(conn, id);
            if (!conn->stopping.load())
                RespondError("EXECUTION_TIMEOUT",
                    "video.frame.capture did not complete within the timeout -- no frame was "
                    "rendered in time (is the guest currently running and producing video output?)");
        }
        return;
    } else {
        RespondError("UNKNOWN_METHOD", "unknown method \"" + method + "\"");
        return;
    }

    LOG(LOG_MISC, LOG_DEBUG)("AI bridge: request id=%lld method=%s", id, method.c_str());

    {
        std::lock_guard<std::mutex> lk(conn->mtx);
        conn->responseReady = false;
    }
    {
        std::lock_guard<std::mutex> lk(g_queueMutex);
        g_requestQueue.push_back(item);
    }

    std::unique_lock<std::mutex> lk(conn->mtx);
    conn->cv.wait_for(lk, std::chrono::seconds(REQUEST_TIMEOUT_SECONDS),
                       [&] { return conn->responseReady || conn->stopping.load(); });

    if (conn->responseReady) {
        std::string resp = conn->responseLine;
        lk.unlock();
        LOG(LOG_MISC, LOG_DEBUG)("AI bridge: response id=%lld", id);
        SendLine(conn, resp);
    } else if (!conn->stopping.load()) {
        lk.unlock();
        /* Phase 4D: an execution.step_over() that took the async path (see
         * ExecExecutionStepOver()) may still be genuinely in flight here --
         * DEBUG_AI_RequestStepCompletion() registered it, but the one-shot
         * breakpoint hasn't been hit yet. DEBUGGER_NOT_STOPPED would be
         * misleading (the debugger WAS entered and the step WAS kicked
         * off); EXECUTION_TIMEOUT (Phase4D.md section 8) is the accurate,
         * specific code, matching how execution.pause's own timeout is
         * reported above. Cancelling the pending registration here is a
         * no-op for every other method (it was never registered), and for
         * execution.step_over specifically avoids a late
         * DEBUG_AI_CompletePendingSteps() writing into a response slot
         * nobody is waiting on anymore. */
        DEBUG_AI_CancelStep(conn);
        if (item.method == AIMethod::ExecutionStepOver) {
            RespondError("EXECUTION_TIMEOUT",
                "step_over did not complete within the timeout");
        } else {
            RespondError("DEBUGGER_NOT_STOPPED",
                "the DOSBox-X debugger is not currently active (open it with Ctrl+Pause, "
                "or start with -break-start) so no request could be serviced in time");
        }
    }
}

static void ConnectionThreadFunc(std::shared_ptr<AIConnection> conn) {
    LOG(LOG_MISC, LOG_NORMAL)("AI bridge: client connected");

    std::string lineBuf;
    char buf[4096];

    while (!conn->stopping.load()) {
        int n = recv(conn->sock, buf, sizeof(buf), 0);
        if (n <= 0) break;

        lineBuf.append(buf, (size_t)n);
        if (lineBuf.size() > MAX_LINE_LENGTH * 4) {
            /* No newline for way too long -- refuse to keep buffering unbounded data. */
            LOG(LOG_MISC, LOG_WARN)("AI bridge: client sent unterminated oversized data, dropping connection");
            break;
        }

        size_t pos;
        while ((pos = lineBuf.find('\n')) != std::string::npos) {
            std::string line = lineBuf.substr(0, pos);
            lineBuf.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) HandleLine(conn, line);
            if (conn->stopping.load()) break;
        }
    }

    LOG(LOG_MISC, LOG_NORMAL)("AI bridge: client disconnected");

    /* Phase 6B: if this connection left any key/mouse button held down
     * (key_down/mouse.button.set without a matching release before the
     * client disconnected, crashed, or hit a session timeout), release it
     * now rather than leaving the guest with a permanently "stuck" input --
     * see the "Input injection (Phase 6B)" section above. This only
     * enqueues the release; expectsResponse=false because nobody is
     * waiting on conn's response slot by this point, and it still only
     * actually runs once DEBUG_AI_CheckPendingInput() next drains the
     * queue on the emulator thread. */
    {
        PendingInputRequest release;
        release.op = AIInputOp::ReleaseAll;
        release.conn = conn;
        release.expectsResponse = false;
        DEBUG_AI_RequestInput(release);
    }

    if (conn->sock != AI_INVALID_SOCKET) {
        ai_close(conn->sock);
        conn->sock = AI_INVALID_SOCKET;
    }
    {
        std::lock_guard<std::mutex> lk(g_connsMutex);
        g_connections.erase(std::remove(g_connections.begin(), g_connections.end(), conn), g_connections.end());
    }
}

static void AcceptThreadFunc() {
    while (g_bridgeRunning.load()) {
        sockaddr_in clientAddr;
        ai_socklen_t clientLen = sizeof(clientAddr);
        memset(&clientAddr, 0, sizeof(clientAddr));

        ai_socket_t clientSock = accept(g_listenSocket, (sockaddr *)&clientAddr, &clientLen);
        if (clientSock == AI_INVALID_SOCKET) {
            if (!g_bridgeRunning.load()) break;
            continue;
        }

        auto conn = std::make_shared<AIConnection>();
        conn->sock = clientSock;
        {
            std::lock_guard<std::mutex> lk(g_connsMutex);
            g_connections.push_back(conn);
        }
        std::thread(ConnectionThreadFunc, conn).detach();
    }
}

void DEBUG_AI_Init(void) {
    if (g_bridgeRunning.load()) return;

#if defined(WIN32)
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        LOG(LOG_MISC, LOG_ERROR)("AI bridge: WSAStartup failed, bridge disabled");
        return;
    }
#endif

    g_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_listenSocket == AI_INVALID_SOCKET) {
        LOG(LOG_MISC, LOG_ERROR)("AI bridge: socket() failed, bridge disabled");
        return;
    }

    /* POSIX-only: SO_REUSEADDR there just permits re-binding a port stuck
     * in TIME_WAIT, so a second listener while the first is still bound
     * and listening still correctly fails at bind() below. On Windows,
     * SO_REUSEADDR has different (and dangerous) semantics: it lets a
     * SECOND process successfully bind() and listen() on the SAME
     * 127.0.0.1:9876 that a first, still-running DOSBox-X-AI instance
     * already owns, with no error and no defined rule for which process
     * subsequently receives an incoming connection -- exactly the "two
     * Debugger GUIs open at once" scenario this bridge must fail loudly
     * on rather than silently double-bind. This project's own vendored
     * SDL_net (vs/sdlnet/SDLnetTCP.c, vs/sdl2net/SDLnetTCP.c) already
     * documents and works around the identical Windows pitfall by simply
     * not setting SO_REUSEADDR there; this mirrors that precedent instead
     * of introducing a new one. Leaving it unset on Windows restores the
     * platform's default exclusive-bind behavior, so a second instance's
     * bind() below correctly fails and that instance's bridge stays
     * disabled, per DEBUG_AI_Init()'s documented contract (debug_ai.h). */
#if !defined(WIN32)
    int reuse = 1;
    setsockopt(g_listenSocket, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));
#endif

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(AI_BRIDGE_PORT);
    /* Bind ONLY to loopback -- never 0.0.0.0 / INADDR_ANY. Hard security
     * requirement (AGENTS.md 2.4): the AI debugger control interface must
     * never be reachable from the LAN. Uses inet_addr() rather than
     * inet_pton() -- this project's Windows builds target an API level
     * older than Vista, where inet_pton() is not declared. */
    addr.sin_addr.s_addr = inet_addr(AI_BRIDGE_BIND_ADDR);
    if (addr.sin_addr.s_addr == INADDR_NONE) {
        LOG(LOG_MISC, LOG_ERROR)("AI bridge: inet_addr(127.0.0.1) failed, bridge disabled");
        ai_close(g_listenSocket);
        g_listenSocket = AI_INVALID_SOCKET;
        return;
    }

    if (bind(g_listenSocket, (sockaddr *)&addr, sizeof(addr)) != 0) {
        LOG(LOG_MISC, LOG_ERROR)("AI bridge: bind() to 127.0.0.1:%u failed, bridge disabled", (unsigned)AI_BRIDGE_PORT);
        ai_close(g_listenSocket);
        g_listenSocket = AI_INVALID_SOCKET;
        return;
    }

    if (listen(g_listenSocket, 4) != 0) {
        LOG(LOG_MISC, LOG_ERROR)("AI bridge: listen() failed, bridge disabled");
        ai_close(g_listenSocket);
        g_listenSocket = AI_INVALID_SOCKET;
        return;
    }

    g_bridgeRunning = true;
    g_acceptThread = std::thread(AcceptThreadFunc);
    /* Phase 4E fix: detach immediately, exactly like every per-connection
     * thread already is (see the comment above AcceptThreadFunc's own
     * connection-thread spawn) -- std::thread's destructor calls
     * std::terminate() if the thread is still joinable when it is
     * destroyed. g_acceptThread is a function-local... no, file-scope
     * static, so it is destroyed at C++ static-object teardown time; this
     * project's normal shutdown path (DEBUG_AI_ShutDown() below, invoked
     * via DOSBox-X's own AddExitFunction list) reliably joins it first,
     * but at least one other exit path this build can take (observed live:
     * dosbox-x.exe -break-start terminates with STATUS_STACK_BUFFER_OVERRUN
     * a few seconds after startup, with no AI bridge client ever
     * connecting) reaches static teardown WITHOUT running
     * DEBUG_AI_ShutDown() first, so g_acceptThread was still joinable and
     * its destructor called std::terminate(). Detaching here removes the
     * dependency on shutdown-path ordering entirely: ai_close(g_listenSocket)
     * in DEBUG_AI_ShutDown() still unblocks its accept() call and lets it
     * exit on its own, the same way detached connection threads already
     * rely on their own socket closing. */
    g_acceptThread.detach();
    LOG(LOG_MISC, LOG_NORMAL)("AI bridge: listening on 127.0.0.1:%u (Phase 3B, read-only)", (unsigned)AI_BRIDGE_PORT);
}

void DEBUG_AI_ShutDown(void) {
    if (!g_bridgeRunning.load()) return;
    g_bridgeRunning = false;

    if (g_listenSocket != AI_INVALID_SOCKET) {
        ai_close(g_listenSocket); /* unblocks accept() */
        g_listenSocket = AI_INVALID_SOCKET;
    }
    /* g_acceptThread is detached (see DEBUG_AI_Init()) -- never joinable
     * here, so nothing to join. Left as a no-op guard rather than removed
     * outright, in case a future change reintroduces a joinable thread. */
    if (g_acceptThread.joinable()) g_acceptThread.join();

    std::vector<std::shared_ptr<AIConnection>> conns;
    {
        std::lock_guard<std::mutex> lk(g_connsMutex);
        conns = g_connections;
    }
    for (auto &conn : conns) {
        conn->stopping = true;
        if (conn->sock != AI_INVALID_SOCKET) {
#if defined(WIN32)
            shutdown(conn->sock, SD_BOTH);
#else
            shutdown(conn->sock, SHUT_RDWR);
#endif
        }
        conn->cv.notify_all();
    }

    {
        std::lock_guard<std::mutex> lk(g_queueMutex);
        g_requestQueue.clear();
    }

#if defined(WIN32)
    WSACleanup();
#endif

    LOG(LOG_MISC, LOG_NORMAL)("AI bridge: shut down");
}

#endif /* C_DEBUG */

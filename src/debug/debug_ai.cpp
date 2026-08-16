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

#include "regs.h"
#include "cpu.h"
#include "mem.h"
#include "paging.h"
#include "logging.h"

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
    ExecutionStepInto, ExecutionStepOver
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

    int reuse = 1;
    setsockopt(g_listenSocket, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

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

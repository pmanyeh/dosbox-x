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
 * Native DOSBox-X AI Bridge (Phase 3B/4A/4B/4C/4D, DOSBox-X-AI project).
 *
 * A localhost-only (127.0.0.1:9876), newline-delimited-JSON TCP interface
 * exposing live debugger state to an external AI agent, without touching
 * the existing curses debugger UI or its command processing. Phase 3B
 * added read-only inspection (debug.status/cpu.get/memory.read/
 * code.current/code.disassemble); Phase 4A added real write access
 * (memory.write, and register.write for a whitelisted subset of GPRs --
 * see WRITABLE_REGISTERS in debug_ai.cpp); Phase 4B added breakpoint
 * management (breakpoint.set/delete/list) against the SAME CBreakpoint/
 * BPoints state the debugger GUI's BP/BPDEL/BPLIST commands use -- see
 * DEBUG_AI_Breakpoint* below. Phase 4C adds real execution control
 * (execution.continue/execution.pause) against the SAME RUN mechanism
 * (DEBUG_Run()/DOSBOX_SetNormalLoop()) and the SAME "enter debugger"
 * transition (DEBUG_Enable_Handler()) the GUI already uses for RUN and
 * Ctrl+Pause -- see "Execution control (Phase 4C)" below. Phase 4D adds
 * single-step control (execution.step_into/execution.step_over) against
 * the SAME mechanisms (DEBUG_Run()/StepOver()) the debugger GUI's own
 * F10/F11 keys already use -- see "Single-step control (Phase 4D)" below.
 *
 * Architecture (see docs/dosbox-ai-bridge.md for the full write-up):
 *
 *     socket thread(s)            <- accept/read/parse/validate/send only
 *         |  (thread-safe request queue)
 *         v
 *     DEBUG_AI_Poll(), called from DEBUG_Loop()   <- runs on the emulator/
 *         |                                          debugger thread
 *         v
 *     cpu_regs / Segs / guest memory / DasmI386()
 *         |  (thread-safe response handoff, per connection)
 *         v
 *     socket thread sends the response
 *
 * The socket threads never read or write cpu_regs, Segs, guest memory, or
 * any other emulator/debugger state directly -- see debug_ai.cpp and Gate A
 * analysis (docs/dosbox-debugger-analysis.md, section 5.9) for why that
 * would be unsafe: DOSBox-X's debugger and CPU emulation share one thread
 * with no locking of any kind.
 */

#ifndef DOSBOX_DEBUG_AI_H
#define DOSBOX_DEBUG_AI_H

#include "dosbox.h"

#if C_DEBUG

/* Starts the AI bridge's listener thread, bound only to 127.0.0.1:9876.
 * Call once, from DEBUG_Init(). If the socket cannot be created/bound
 * (e.g. the port is already in use), the bridge logs the failure and
 * stays disabled -- it never falls back to a wider bind address, and it
 * never prevents DOSBox-X or the normal debugger from starting. */
void DEBUG_AI_Init(void);

/* Stops the listener and signals every connection to close. Call once,
 * from DEBUG_ShutDown(). Non-blocking on slow/stuck clients: connection
 * threads are told to stop and left to exit on their own rather than
 * being joined here. */
void DEBUG_AI_ShutDown(void);

/* Drains any AI bridge requests queued by the socket thread(s) and
 * executes them against live debugger state (registers, guest memory,
 * disassembly). MUST be called only from the DOSBox-X emulator/debugger
 * thread -- the same thread that runs DEBUG_Loop() -- and never from a
 * bridge socket thread. This is what makes direct access to cpu_regs/
 * Segs/guest memory safe here: nothing else touches that state
 * concurrently, the same single-threaded contract the rest of
 * src/debug/ already relies on (see Gate A analysis, section 5.9). */
void DEBUG_AI_Poll(void);

/* Implemented in debug.cpp (debug_running is file-local there). Exposes
 * just enough of existing debugger state for debug.status's "stopped"
 * field, without duplicating or relocating that state. */
bool DEBUG_AI_IsDebugRunning(void);

/* Implemented in debug.cpp. Thin wrappers around the EXISTING
 * CBreakpoint class (debug.cpp) and its BPoints list -- Phase 4B. None
 * of these touch breakpoint state directly from debug_ai.cpp, and none
 * of them introduce a second/parallel breakpoint store; debug_ai.cpp
 * never sees the CBreakpoint type at all, only plain seg:off/index data.
 *
 * Breakpoint "id" is the 0-based position in the underlying BPoints list
 * -- the SAME identifier ShowList()/DeleteByIndex() (the BPLIST/BPDEL
 * debugger commands) already use, not a bridge-invented numbering
 * scheme. Like the GUI's own BPLIST/BPDEL, this id is a *position*, not
 * a stable per-breakpoint identity: adding a new breakpoint (which
 * front-inserts) shifts every existing id up by one, and deleting a
 * breakpoint shifts every later id down by one. Callers should treat an
 * id as valid only until the next breakpoint.set/breakpoint.delete. */
uint16_t DEBUG_AI_BreakpointCount(void);
bool DEBUG_AI_BreakpointInfo(uint16_t index, bool &isPhysical, bool &isRealMemory,
                             bool &isProtectedMemory, uint16_t &seg, uint32_t &off);
bool DEBUG_AI_BreakpointExists(uint16_t seg, uint32_t off);
int DEBUG_AI_BreakpointAdd(uint16_t seg, uint32_t off);
int DEBUG_AI_ProtectedMemoryBreakpointAdd(uint16_t selector, uint32_t off);
int DEBUG_AI_RealMemoryBreakpointAdd(uint16_t seg, uint32_t off);
bool DEBUG_AI_BreakpointDelete(uint16_t index);

/* ------------------------------------------------------------------ */
/* Execution control (Phase 4C)                                        */
/*                                                                      */
/* continue and pause need two DIFFERENT plumbing paths, because they  */
/* start from two different execution contexts:                       */
/*                                                                      */
/*  - continue_execution() is only meaningful while the debugger is    */
/*    already stopped (DEBUG_Loop() is the active loop, so             */
/*    DEBUG_AI_Poll() is running). It is therefore just another         */
/*    request in the SAME g_requestQueue/DEBUG_AI_Poll() mechanism      */
/*    everything else (Phase 3B/4A/4B) already uses -- see              */
/*    DEBUG_AI_DoContinue() below.                                      */
/*                                                                      */
/*  - pause_execution() is only meaningful while the debugger is NOT    */
/*    stopped -- guest code is running under Normal_Loop() (dosbox.cpp),*/
/*    which never calls DEBUG_Loop()/DEBUG_AI_Poll() at all. There is   */
/*    nothing for the existing request queue to be drained by. Instead, */
/*    a pause request is recorded in a small thread-safe pending list   */
/*    (DEBUG_AI_RequestPause()/DEBUG_AI_HasPendingPause(), both in       */
/*    debug_ai.cpp) that is checked from a NEW hook in Normal_Loop's     */
/*    existing per-iteration DEBUG_ExitLoop() check (dosbox.cpp) --      */
/*    DEBUG_AI_CheckPauseRequest(), declared in include/debug.h and      */
/*    defined in debug.cpp, which reuses DEBUG_Enable_Handler() (the     */
/*    SAME function Ctrl+Pause already calls) to perform the actual      */
/*    "enter debugger" transition, then calls                            */
/*    DEBUG_AI_CompletePendingPauses() to hand real, post-transition     */
/*    debugger state back to the waiting connection(s). */
/* ------------------------------------------------------------------ */

/* Mirrors whether DEBUG_Loop() is currently the active main-loop
 * handler (i.e. whether the debugger is genuinely stopped right now).
 * Written ONLY by the emulator/debugger thread (DEBUG_Loop() in
 * debug.cpp, DEBUG_AI_DoContinue() in debug.cpp, and Normal_Loop() in
 * dosbox.cpp); read by socket threads (via std::atomic, not a bare
 * bool) to answer debug.status/execution.continue/execution.pause
 * without ever touching cpu_regs/Segs/guest memory themselves. */
void DEBUG_AI_SetDebuggerActive(bool active);

/* Implemented in debug.cpp. Performs the EXACT SAME guest-resume
 * transition as the debugger GUI's "RUN" command (debug.cpp's
 * ParseCommand(), which now calls this same function instead of
 * duplicating its body) -- debug_running=false, debugging=false,
 * DEBUG_Run(1,false), DOSBOX_SetNormalLoop(). Callable ONLY from
 * DEBUG_AI_Poll() (i.e. only while the debugger is already stopped). */
void DEBUG_AI_DoContinue(void);

/* Implemented in debug_ai.cpp. Records/queries pending pause_execution()
 * requests. DEBUG_AI_RequestPause() is called from a socket thread
 * (HandleLine()); DEBUG_AI_HasPendingPause() and
 * DEBUG_AI_CompletePendingPauses() are called ONLY from the emulator
 * thread, from DEBUG_AI_CheckPauseRequest() (debug.cpp). */
bool DEBUG_AI_HasPendingPause(void);
void DEBUG_AI_CompletePendingPauses(void);

/* ------------------------------------------------------------------ */
/* Single-step control (Phase 4D)                                      */
/*                                                                      */
/* step_into() is always synchronous: DEBUG_AI_DoStepInto() (debug.cpp) */
/* executes exactly one guest instruction via the SAME DEBUG_Run(1,true) */
/* the debugger GUI's F11 key already uses, and control returns to the  */
/* debugger before the call returns -- so it is just another request in */
/* the SAME g_requestQueue/DEBUG_AI_Poll() mechanism everything else    */
/* (Phase 3B/4A/4B/4C) already uses.                                    */
/*                                                                      */
/* step_over() is USUALLY synchronous too (DEBUG_AI_DoStepOver() falls  */
/* through to DEBUG_AI_DoStepInto() for any instruction that isn't a    */
/* call/int/loop/rep), but for one of those it reuses the debugger's    */
/* OWN one-shot-breakpoint mechanism (StepOver(), debug.cpp) and hands   */
/* control to Normal_Loop() exactly like execution.continue does --     */
/* becameAsync (see DEBUG_AI_DoStepOver() below) tells the caller which */
/* happened. In the async case, DEBUG_AI_Poll() does NOT complete the   */
/* connection's response immediately; instead the request is recorded   */
/* in a pending-step list (mirroring g_pendingPauses/PendingPause above) */
/* that DEBUG_AI_CompletePendingSteps() drains once DEBUG_Loop() is the */
/* active loop again -- i.e. once the debugger has genuinely regained    */
/* control, whether because the one-shot breakpoint fired or for any     */
/* other reason (see the DEBUG_Loop() call site, debug.cpp, right next   */
/* to DEBUG_AI_SetDebuggerActive(true)). This deliberately reuses the    */
/* SAME "hand off to Normal_Loop(), observe the re-stop later" pattern   */
/* Phase 4C already established for execution.continue/pause rather than */
/* inventing a second async completion mechanism.                        */
/* ------------------------------------------------------------------ */

/* Implemented in debug.cpp. See "Single-step control" above. Callable
 * ONLY from the emulator/debugger thread (DEBUG_AI_Poll()). */
int32_t DEBUG_AI_DoStepInto(void);
int32_t DEBUG_AI_DoStepOver(bool &becameAsync);

/* Implemented in debug_ai.cpp. DEBUG_AI_CompletePendingSteps() is called
 * ONLY from the emulator thread, from DEBUG_Loop() (debug.cpp) -- see
 * "Single-step control" above. A no-op whenever no execution.step_over()
 * is currently pending completion. */
void DEBUG_AI_CompletePendingSteps(void);

/* ------------------------------------------------------------------ */
/* Input injection (Phase 6B)                                          */
/*                                                                      */
/* input.key.down/input.key.up/input.key.tap and input.mouse.* inject   */
/* guest keyboard/mouse input through the SAME internal entry points     */
/* the real SDL input handlers already use -- KEYBOARD_AddKey()          */
/* (include/keyboard.h) and Mouse_CursorMoved()/Mouse_ButtonPressed()/   */
/* Mouse_ButtonReleased() (include/mouse.h). There is no Win32 SendKeys,  */
/* window-handle/focus manipulation, or GUI automation of DOSBox-X's own  */
/* window anywhere in this path -- see debug_ai.cpp's "Input injection    */
/* (Phase 6B)" section for the full design.                               */
/*                                                                        */
/* Like pause_execution(), these are only meaningful while guest code is  */
/* actually running under Normal_Loop() (dosbox.cpp): KEYBOARD_AddKey()   */
/* and the Mouse_* functions are emulator-thread-only, and Normal_Loop()  */
/* only has control while the debugger is NOT stopped. So this mirrors    */
/* g_pendingPauses/DEBUG_AI_CheckPauseRequest() exactly: a socket thread   */
/* records a request in debug_ai.cpp's g_pendingInputs (entirely private   */
/* to that file), and DEBUG_AI_CheckPendingInput() -- called from the SAME */
/* Normal_Loop() hook (dosbox.cpp), right next to                         */
/* DEBUG_AI_CheckPauseRequest() -- drains it on the emulator thread.       */
/*                                                                        */
/* Unlike DEBUG_AI_CheckPauseRequest(), this never needs to enter the      */
/* debugger or hand off to a different main-loop handler, so it returns    */
/* void and dosbox.cpp calls it unconditionally every iteration rather     */
/* than branching on its result. */
void DEBUG_AI_CheckPendingInput(void);

/* ------------------------------------------------------------------ */
/* Frame capture (Phase 7A)                                            */
/*                                                                      */
/* video.frame.capture returns a PNG/RGBA8888 snapshot of exactly the   */
/* guest's own rendered frame -- never the DOSBox-X window, the SDL/GL   */
/* surface as presented, or the host desktop -- by reusing the SAME      */
/* internal hook point DOSBox-X's own Host+P screenshot and AVI          */
/* recording already use: RENDER_EndUpdate() (src/gui/render.cpp) hands  */
/* the pre-scaler, pre-backend frame (scalerSourceCacheBuffer) to        */
/* CAPTURE_AddImage() (src/hardware/hardware.cpp) whenever a screenshot   */
/* or recording is pending; this adds a second, independent check at      */
/* that exact same site, confirmed backend-agnostic (the SAME call site   */
/* regardless of active software/OpenGL/Direct3D/Voodoo output) by        */
/* docs/phase7a-frame-capture-design.md's source investigation. See       */
/* debug_ai.cpp's "Frame capture (Phase 7A)" section for the full         */
/* design, including why nothing here is ever written to disk.            */
/*                                                                        */
/* RENDER_EndUpdate() runs on the emulator thread (PIC-event-driven from  */
/* the VGA draw routines) -- the SAME thread Phase 6B's                   */
/* KEYBOARD_AddKey()/Mouse_*() calls are already safe on. So this mirrors */
/* g_pendingInputs/DEBUG_AI_CheckPendingInput() exactly: a socket thread   */
/* records a request in debug_ai.cpp's g_pendingFrameCaptures (entirely   */
/* private to that file), and DEBUG_AI_CheckPendingFrameCapture() --      */
/* called from RENDER_EndUpdate() (render.cpp), NOT from Normal_Loop()'s  */
/* hook, since RENDER_EndUpdate() is the only place                       */
/* scalerSourceCacheBuffer is valid for this frame -- drains it on the    */
/* emulator thread. Cheap (a single relaxed atomic load) whenever          */
/* nothing is pending, since it is called on every rendered frame,        */
/* far more often than Normal_Loop()'s own per-iteration hook. */
void DEBUG_AI_CheckPendingFrameCapture(Bitu width, Bitu height, Bitu bpp, Bitu pitch,
    Bitu flags, const uint8_t *data, const uint8_t *pal);

#endif

#endif

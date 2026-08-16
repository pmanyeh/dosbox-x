/*
 *  Copyright (C) 2002-2021  The DOSBox Team
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

void DEBUG_SetupConsole(void);
void DEBUG_DrawScreen(void);
bool DEBUG_Breakpoint(void);
bool DEBUG_IntBreakpoint(uint8_t intNum);
void DEBUG_Enable(bool pressed);
void DEBUG_CheckExecuteBreakpoint(uint16_t seg, uint32_t off);
bool DEBUG_ExitLoop(void);
void DEBUG_RefreshPage(char scroll);
Bitu DEBUG_EnableDebugger(void);

/* Phase 4C (DOSBox-X-AI project, src/debug/debug_ai.cpp): checks for a
 * pending AI bridge pause_execution() request and, if one exists, enters
 * the debugger via the SAME DEBUG_Enable_Handler() Ctrl+Pause already
 * uses. Defined in debug.cpp; called only from Normal_Loop() (dosbox.cpp),
 * on the emulator thread, right alongside the existing DEBUG_ExitLoop()
 * check. See src/debug/debug_ai.h for the full Phase 4C design. */
bool DEBUG_AI_CheckPauseRequest(void);

/* Phase 4C: defined in src/debug/debug_ai.cpp, declared again here (as
 * well as in debug_ai.h) so dosbox.cpp -- which already includes this
 * header, not debug_ai.h -- can mark every Normal_Loop() call as "the
 * debugger is not active" without a new include dependency. See
 * src/debug/debug_ai.h for the full Phase 4C design. */
void DEBUG_AI_SetDebuggerActive(bool active);

/* Phase 6B (DOSBox-X-AI project, src/debug/debug_ai.cpp): drains any
 * queued key/mouse input injection requests (the input.key.* and
 * input.mouse.* methods) and executes them via KEYBOARD_AddKey() and the
 * Mouse_* functions -- the SAME internal entry points real SDL input
 * events use. Defined in debug_ai.cpp;
 * declared again here (as well as in debug_ai.h) so dosbox.cpp -- which
 * already includes this header, not debug_ai.h -- can call it from
 * Normal_Loop() without a new include dependency, right alongside
 * DEBUG_AI_CheckPauseRequest() above. See src/debug/debug_ai.h for the
 * full Phase 6B design. */
void DEBUG_AI_CheckPendingInput(void);

extern Bitu cycle_count;
extern Bitu debugCallback;

#ifdef C_HEAVY_DEBUG
bool DEBUG_HeavyIsBreakpoint(void);
void DEBUG_HeavyWriteLogInstruction(void);
#endif

/**
 * \file main.h
 *
 * The PROS project header: competition entry-point prototypes plus the PROS
 * API. Rewritten at the C7 cutover (2026-08-10) when src/main.cpp was rewired
 * onto the shulib v2 core.
 *
 * Deliberately minimal — three things the February version did are gone:
 *   * `#include "shulib/api.hpp"` — the old umbrella header, deleted with the
 *     rest of the pre-rebuild tree at the C7 cutover. v2 policy is
 *     include-what-you-use: main.cpp includes the exact shulib headers it
 *     wires, nothing re-exports the library wholesale.
 *   * `using namespace pros; using namespace shulib;` — namespace pollution
 *     in a header, inherited by every includer. v2 code qualifies its names.
 *   * `#include "liblvgl/lvgl.h"` — nothing in the project touches LVGL
 *     directly (the one legacy GUI idea worth keeping is planned as the
 *     D-12 BrainHud; it will include what it needs, where it needs it).
 *
 * PROS-boundary note: this header and src/main.cpp are the ONLY project files
 * that may include <pros/...> / "api.h". Everything under include/shulib/ is
 * PROS-free by contract, and CI enforces that over the whole tree.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#include "api.h"

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#endif  // _PROS_MAIN_H_

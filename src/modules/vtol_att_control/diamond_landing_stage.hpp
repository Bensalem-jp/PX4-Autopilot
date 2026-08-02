/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file diamond_landing_stage.hpp
 *
 * Diamond VTOL: the landing / back-transition configuration, applied once at
 * front-transition completion.
 *
 * Eleven of these values differ between the calm and the windy mission and
 * cannot be chosen before takeoff -- they configure the back transition, the
 * MC brake and the descent, all of which happen after the aircraft is already
 * flying.  Until now a ground runner pushed them over MAVLink at FW entry,
 * which meant the aircraft was correctly configured for its landing only while
 * a laptop was talking to it: a link loss between takeoff and FW entry left it
 * descending on the boot (takeoff/climb) configuration, and nothing on board
 * noticed.  This table moves that push into the firmware.
 *
 * The values themselves are not restated here -- see
 * `bwb_vtol/px4_staging.py::build_landing_stage_params()` in the campaign repo
 * for the forensics behind each one.  `tests/test_diamond_landing_stage.py`
 * asserts the two agree entry for entry, in both regimes, so a value changed on
 * one side without the other fails a test rather than a flight.
 */

#pragma once

#include <parameters/param.h>

struct DiamondLandingStageParam {
	const char *name;
	param_type_t type;	/**< PARAM_TYPE_INT32 values are stored here as whole floats */
	float calm;		/**< VT_DIA_WX_MODE 0 */
	float wind;		/**< VT_DIA_WX_MODE 1 and 2 */
};

extern const DiamondLandingStageParam kDiamondLandingStage[];
extern const unsigned kDiamondLandingStageCount;

/**
 * Write the whole table for one regime.
 *
 * Entries whose live value already matches are still written; param_set()
 * compares before it stores, so an unchanged entry costs no autosave and no
 * notification.  Writing all of them (rather than a precomputed delta against
 * the airframe file) keeps this table a statement about the FW-entry
 * configuration itself, so editing a boot value cannot silently change what
 * the aircraft lands with.
 *
 * @param wind true for VT_DIA_WX_MODE 1 or 2
 * @return number of entries written, or -1 if any parameter name was unknown
 */
int applyDiamondLandingStage(bool wind);

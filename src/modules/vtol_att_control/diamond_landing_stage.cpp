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
 * @file diamond_landing_stage.cpp
 */

#include "diamond_landing_stage.hpp"

#include <px4_platform_common/log.h>

// Order and values mirror build_fw_entry_rate_caps() followed by
// build_landing_stage_params() in the campaign repo, entry for entry; the
// parity test compares them positionally.  Only the one-line "what it is for"
// is repeated here -- the measurements that chose each number live next to the
// Python, in one place, and are far longer than this file.
const DiamondLandingStageParam kDiamondLandingStage[] = {
	// FW-regime rate caps.  The airframe boots these at 12 deg/s, which is a
	// transition limit -- the FW attitude controller is live through P1/P2 and
	// the tight cap keeps the tilting vehicle from chasing attitude error with
	// the wing.  Cruise wants the full 25, in either regime.
	{"FW_P_RMAX_POS",	PARAM_TYPE_FLOAT,	25.f,	25.f},
	{"FW_P_RMAX_NEG",	PARAM_TYPE_FLOAT,	25.f,	25.f},
	{"FW_R_RMAX",		PARAM_TYPE_FLOAT,	25.f,	25.f},
	// Pusher assist during the landing phase: the wind drift brake.  In calm
	// it is orbital-trap fuel, so calm turns it off.
	{"VT_FWD_THRUST_EN",	PARAM_TYPE_INT32,	0.f,	2.f},
	// Release the >17 m/s takeoff pitch clamp: the descent is governed by
	// VT_LND_PITCH_MIN regardless, so both regimes land on the same value.
	{"VT_PITCH_MIN",	PARAM_TYPE_FLOAT,	-1.f,	-1.f},
	// Hold the descent to 2 m before the flare.
	{"MPC_LAND_ALT1",	PARAM_TYPE_FLOAT,	2.f,	2.f},
	// Give back the takeoff-punch climb cap: a gust must not balloon the
	// vehicle upward during the sub-ALT1 hold.
	{"MPC_Z_VEL_MAX_UP",	PARAM_TYPE_FLOAT,	1.f,	1.f},
	// MC brake attitude bound.  Calm is tighter because its killer is the
	// reposition-engage step transient, not the brake authority.
	{"MPC_TILTMAX_AIR",	PARAM_TYPE_FLOAT,	10.f,	20.f},
	// MC brake gentleness.  Calm brakes at 0.8 m/s^2 so the vehicle never
	// drifts backward into the reverse-flow escalation; wind is braked by
	// the wind itself.
	{"MPC_ACC_HOR",		PARAM_TYPE_FLOAT,	0.8f,	3.f},
	// Reposition speed: the land point is a creep away, and at cruise speed
	// the reversed airflow flips the directional statics.
	{"MPC_XY_CRUISE",	PARAM_TYPE_FLOAT,	1.5f,	1.5f},
	// Back-transition trigger margin.  Calm stops deterministically SHORT of
	// the point (the navigator's prediction misses the trigger/tilt latency);
	// the wind geometry is validated at 0.
	{"VT_DIA_BT_MGN",	PARAM_TYPE_FLOAT,	35.f,	0.f},
	// Flare-taper knee, paired with the short stop above.
	{"VT_DIA_BT_TPR",	PARAM_TYPE_FLOAT,	6.f,	0.f},
	// Trajectory-smoother horizontal error gate; wind needs the Z-ramp
	// unlock, calm keeps the vtol default.
	{"MPC_XY_ERR_MAX",	PARAM_TYPE_FLOAT,	5.f,	20.f},
	// Ground rotation is a takeoff device: off for the landing in both
	// regimes.
	{"VT_DIA_ROT_P",	PARAM_TYPE_FLOAT,	0.f,	0.f},
	// Pusher-assist scale, sized with VT_FWD_THRUST_EN above.
	{"VT_FWD_THRUST_SC",	PARAM_TYPE_FLOAT,	1.2f,	2.5f},
	// Waypoint acceptance radius for the landing legs.
	{"NAV_ACC_RAD",		PARAM_TYPE_FLOAT,	10.f,	100.f},
	// Descent wing-unload bias: the wing refuses the descent and the
	// vertical loop throttles to zero chasing it.
	{"VT_DIA_DST_P",	PARAM_TYPE_FLOAT,	3.f,	3.f},
	// False-land guard: at the back-transition apex vz, groundspeed and
	// thrust all fall to ~0 and the MC land detector once DISARMED at 33 m.
	{"LNDMC_TRIG_TIME",	PARAM_TYPE_FLOAT,	5.f,	5.f},
	// Heading-alignment gate for the landing legs.
	{"MIS_YAW_ERR",		PARAM_TYPE_FLOAT,	12.f,	12.f},
};

const unsigned kDiamondLandingStageCount =
	sizeof(kDiamondLandingStage) / sizeof(kDiamondLandingStage[0]);

int applyDiamondLandingStage(bool wind)
{
	param_t handles[kDiamondLandingStageCount];

	// Resolve everything before writing anything.  A name this firmware does
	// not have is a build error, not a flight condition, and half a table is
	// worse than either whole one: the aircraft would descend on a mixture of
	// the takeoff and landing configurations that was never flown.
	for (unsigned i = 0; i < kDiamondLandingStageCount; i++) {
		const DiamondLandingStageParam &entry = kDiamondLandingStage[i];
		handles[i] = param_find(entry.name);

		if (handles[i] == PARAM_INVALID) {
			PX4_ERR("landing stage: no parameter %s; table NOT applied", entry.name);
			return -1;
		}

		// A float written into an int32 parameter (or the reverse) is stored
		// as a reinterpreted bit pattern, i.e. a silently absurd value.
		if (param_type(handles[i]) != entry.type) {
			PX4_ERR("landing stage: %s is type %d, table says %d; table NOT applied",
				entry.name, (int)param_type(handles[i]), (int)entry.type);
			return -1;
		}
	}

	for (unsigned i = 0; i < kDiamondLandingStageCount; i++) {
		const DiamondLandingStageParam &entry = kDiamondLandingStage[i];
		const float value = wind ? entry.wind : entry.calm;

		if (entry.type == PARAM_TYPE_INT32) {
			const int32_t int_value = (int32_t)value;
			param_set(handles[i], &int_value);

		} else {
			param_set(handles[i], &value);
		}
	}

	return (int)kDiamondLandingStageCount;
}

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
 * @file pitch_quiet_gate.hpp
 *
 * Diamond front-transition P1 exit gate (VT_DIA_PQ_*), extracted from
 * Tiltrotor so it can be unit tested without the module. Header only, no
 * uORB: the caller selects the speed source and passes the samples.
 */

#pragma once

#include <cmath>
#include <cstdint>

class PitchQuietGate
{
public:
	using Timestamp = uint64_t; // microseconds, same base as hrt_abstime

	struct Params {
		float min_trans_time_s;     // getMinimumFrontTransitionTime()
		float transition_airspeed;  // getTransitionAirspeed()
		float tilt_trans;           // VT_TILT_TRANS
		float pq_angle_rad;         // VT_DIA_PQ_ANG
		float pq_rate_rad_s;        // VT_DIA_PQ_RATE
		float pq_window_s;          // VT_DIA_PQ_T (<= 0: gate disabled)
	};

	struct Inputs {
		Timestamp now;
		float time_since_trans_start_s;
		float speed;                // NaN when no valid speed source
		float tilt;
		float pitch_rad;            // vehicle_attitude theta
		Timestamp pitch_timestamp;
		float pitch_rate_rad_s;     // vehicle_angular_velocity xyz[1]
		Timestamp pitch_rate_timestamp;
	};

	/** Forget the quiet history (front-transition start). */
	void reset() { _quiet_since = 0; }

	/**
	 * P1 exit decision for one controller cycle.
	 *
	 * Behaviour of f959ecd94d, kept exactly: the minimum-time, speed and
	 * tilt conditions are evaluated first and short-circuit the quiet
	 * test, so the quiet timer is only updated (and only reset) on cycles
	 * where all three hold.
	 */
	bool update(const Inputs &in, const Params &p)
	{
		const bool minimum_trans_time_elapsed = in.time_since_trans_start_s > p.min_trans_time_s;
		const bool speed_condition = std::isfinite(in.speed) && in.speed >= p.transition_airspeed;
		const bool tilt_condition = in.tilt >= p.tilt_trans;

		if (!(minimum_trans_time_elapsed && speed_condition && tilt_condition)) {
			return false;
		}

		if (p.pq_window_s <= 0.f) {
			return true; // gate disabled: stock tiltrotor behaviour
		}

		const bool quiet = fabsf(in.pitch_rad) < p.pq_angle_rad
				   && fabsf(in.pitch_rate_rad_s) < p.pq_rate_rad_s;

		if (!quiet) {
			_quiet_since = 0;
			return false;
		}

		if (_quiet_since == 0) {
			_quiet_since = in.now;
		}

		// Same comparison as before: elapsed (integer us) against a float window.
		return static_cast<float>(in.now - _quiet_since) > p.pq_window_s * 1e6f;
	}

private:
	Timestamp _quiet_since{0};
};

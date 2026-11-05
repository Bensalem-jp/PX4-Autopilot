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
 * @file PitchQuietGateTest.cpp
 *
 * Front-transition P1 exit gate (VT_DIA_PQ_*). The tests state the required
 * behaviour: the gate passes only after quiet, speed, tilt and minimum time
 * have ALL held continuously for VT_DIA_PQ_T. On f959ecd94d the quiet timer
 * is updated only when the other three conditions hold (&& short-circuit),
 * so a timer started earlier survives non-quiet flight below the transition
 * airspeed (#394 Step 2, seed 20261106). Inputs are scripted at the 250 Hz
 * controller rate with the 22002 parameter values.
 */

#include "pitch_quiet_gate.hpp"
#include <gtest/gtest.h>

#include <vector>

namespace
{
constexpr float kDeg = 0.017453292f;
constexpr double kCycleS = 0.004; // 250 Hz
constexpr uint64_t kStartUs = 1000000000ull;

struct Segment {
	double duration_s;
	float speed;
	float tilt;
	float pitch_deg;
	float rate_deg_s;
	double sample_age_s; // age of the attitude / rate samples
};

PitchQuietGate::Params params22002(float window_s = 1.f)
{
	PitchQuietGate::Params p{};
	p.min_trans_time_s = 2.f;      // VT_TRANS_MIN_TM
	p.transition_airspeed = 14.f;  // VT_ARSP_TRANS (mode 1)
	p.tilt_trans = 0.35f;          // VT_TILT_TRANS
	p.pq_angle_rad = 5.f * kDeg;   // VT_DIA_PQ_ANG
	p.pq_rate_rad_s = 15.f * kDeg; // VT_DIA_PQ_RATE
	p.pq_window_s = window_s;      // VT_DIA_PQ_T
	return p;
}

// Runs the segments from t = 0 (transition start); returns the first time
// the gate passes, or -1 if it never does.
double firstPass(PitchQuietGate &gate, const std::vector<Segment> &segments,
		 const PitchQuietGate::Params &p = params22002(), double t0 = 0.0)
{
	double t = t0;

	for (const Segment &s : segments) {
		const int cycles = static_cast<int>(s.duration_s / kCycleS + 0.5);

		for (int k = 0; k < cycles; ++k) {
			PitchQuietGate::Inputs in{};
			in.now = kStartUs + static_cast<uint64_t>(t * 1e6 + 0.5);
			const uint64_t sample = in.now - static_cast<uint64_t>(s.sample_age_s * 1e6 + 0.5);
			in.time_since_trans_start_s = static_cast<float>(t);
			in.speed = s.speed;
			in.tilt = s.tilt;
			in.pitch_rad = s.pitch_deg * kDeg;
			in.pitch_timestamp = sample;
			in.pitch_rate_rad_s = s.rate_deg_s * kDeg;
			in.pitch_rate_timestamp = sample;

			if (gate.update(in, p)) {
				return t;
			}

			t += kCycleS;
		}
	}

	return -1.0;
}

// Not quiet, everything else true: the timer must not run.
const Segment kRinging{2.5, 15.f, 0.35f, 0.f, 30.f, 0.0};
} // namespace

// CE scenario: 0.2 s quiet at 14.2 m/s, 3 s at 12 m/s with q 30 deg/s, then
// quiet at 14.2 m/s. The 0.2 s must not count: the gate may pass only after a
// fresh, continuous 1.0 s (first possible pass > 6.7 s).
TEST(PitchQuietGateTest, StaleQuietTimerDoesNotSurviveASpeedDip)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {
		kRinging,
		{0.2, 14.2f, 0.35f, 1.f, 5.f, 0.0},
		{3.0, 12.f, 0.35f, 8.f, 30.f, 0.0},
		{2.0, 14.2f, 0.35f, 1.f, 5.f, 0.0},
	});
	EXPECT_GT(pass, 5.7 + 1.0 - kCycleS / 2);
}

// The short-circuit boundary: a dip to just below VT_ARSP_TRANS must reset
// the window like any failing sample.
TEST(PitchQuietGateTest, DipJustBelowTransitionAirspeedResetsTheWindow)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {
		kRinging,
		{0.2, 14.f, 0.35f, 1.f, 5.f, 0.0},
		{3.0, 13.99f, 0.35f, 8.f, 30.f, 0.0},
		{2.0, 14.f, 0.35f, 1.f, 5.f, 0.0},
	});
	EXPECT_GT(pass, 5.7 + 1.0 - kCycleS / 2);
}

// Exactly VT_ARSP_TRANS satisfies the speed condition (speed >= threshold).
TEST(PitchQuietGateTest, ExactlyTransitionAirspeedCountsAsFastEnough)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {kRinging, {2.0, 14.f, 0.35f, 1.f, 5.f, 0.0}});
	EXPECT_GT(pass, 2.5 + 1.0 - kCycleS / 2);
	EXPECT_LT(pass, 2.5 + 1.0 + 3 * kCycleS);
}

TEST(PitchQuietGateTest, BelowTransitionAirspeedNeverPasses)
{
	PitchQuietGate gate;
	EXPECT_LT(firstPass(gate, {kRinging, {5.0, 13.99f, 0.35f, 1.f, 5.f, 0.0}}), 0.0);
}

// The same retention through the tilt condition.
TEST(PitchQuietGateTest, StaleQuietTimerDoesNotSurviveATiltDip)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {
		kRinging,
		{0.2, 15.f, 0.35f, 1.f, 5.f, 0.0},
		{3.0, 15.f, 0.30f, 8.f, 30.f, 0.0},
		{2.0, 15.f, 0.35f, 1.f, 5.f, 0.0},
	});
	EXPECT_GT(pass, 5.7 + 1.0 - kCycleS / 2);
}

// A single failing sample restarts the window.
TEST(PitchQuietGateTest, OneNoisySampleRestartsTheWindow)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {
		kRinging,
		{0.9, 15.f, 0.35f, 1.f, 5.f, 0.0},
		{kCycleS, 15.f, 0.35f, 1.f, 20.f, 0.0},
		{2.0, 15.f, 0.35f, 1.f, 5.f, 0.0},
	});
	EXPECT_GT(pass, 2.5 + 0.9 + kCycleS + 1.0 - kCycleS / 2);
}

TEST(PitchQuietGateTest, NonFiniteInputsAreNotQuiet)
{
	for (const Segment &bad : {Segment{3.0, 15.f, 0.35f, NAN, 5.f, 0.0},
				   Segment{3.0, 15.f, 0.35f, 1.f, NAN, 0.0},
				   Segment{3.0, NAN, 0.35f, 1.f, 5.f, 0.0}}) {
		PitchQuietGate gate;
		EXPECT_LT(firstPass(gate, {kRinging, bad}), 0.0);
	}
}

// Attitude / rate samples 0.5 s old are stale and cannot prove quiet.
TEST(PitchQuietGateTest, StaleSamplesAreNotQuiet)
{
	PitchQuietGate gate;
	EXPECT_LT(firstPass(gate, {kRinging, {3.0, 15.f, 0.35f, 1.f, 5.f, 0.5}}), 0.0);
}

// The window does not open before the minimum transition time.
TEST(PitchQuietGateTest, WindowStartsAfterTheMinimumTransitionTime)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {{4.0, 15.f, 0.35f, 1.f, 5.f, 0.0}});
	EXPECT_GT(pass, 2.0 + 1.0 - kCycleS / 2);
}

// 0.8 s of window, reset, then 0.5 s more: without the reset the gate would
// pass after 0.2 s; with it, a full window is needed again.
TEST(PitchQuietGateTest, ResetForgetsTheWindow)
{
	PitchQuietGate gate;
	const Segment quiet{0.8, 15.f, 0.35f, 1.f, 5.f, 0.0};
	EXPECT_LT(firstPass(gate, {kRinging, quiet}), 0.0);
	gate.reset();
	EXPECT_LT(firstPass(gate, {{0.5, 15.f, 0.35f, 1.f, 5.f, 0.0}}, params22002(), 3.3), 0.0);
}

// VT_DIA_PQ_T <= 0: stock behaviour, pass as soon as time, speed and tilt hold.
TEST(PitchQuietGateTest, DisabledGateIsStockBehaviour)
{
	PitchQuietGate gate;
	const double pass = firstPass(gate, {{3.0, 15.f, 0.35f, 30.f, 90.f, 0.0}}, params22002(0.f));
	EXPECT_GT(pass, 2.0);
	EXPECT_LT(pass, 2.0 + 2 * kCycleS);
}

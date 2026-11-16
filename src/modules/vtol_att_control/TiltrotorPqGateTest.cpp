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
 * @file TiltrotorPqGateTest.cpp
 *
 * Module-level check of the Diamond P1 pitch-quiet gate inside the real
 * Tiltrotor::update_vtol_state(): the gate window is cleared on every cycle
 * outside P1 (P1 -> P2, P2, back transition, MC request, fixed-wing system
 * failure / quad-chute, re-entry into P1), and the gate is evaluated on every
 * P1 cycle (a failing speed or tilt sample closes the window; the old
 * short-circuit kept it). Uses the 22002 VT_DIA_PQ_* / transition values.
 */

#define MODULE_NAME "vtol_att_control"

#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

class TiltrotorPqGateTest : public ::testing::Test
{
public:
	using Mode = Tiltrotor::vtol_mode;

	static constexpr float kFast = 16.f;   // CAS above VT_ARSP_TRANS
	static constexpr float kSlow = 12.f;   // CAS below VT_ARSP_TRANS
	static constexpr float kTilt = 0.35f;  // = VT_TILT_TRANS

	void SetUp() override
	{
		param_control_autosave(false);
		setParam("VT_ARSP_TRANS", 14.f);
		setParam("VT_TILT_TRANS", 0.35f);
		setParam("VT_TILT_FW", 1.f);
		setParam("VT_TRANS_MIN_TM", 2.f);
		setParam("VT_DIA_PQ_ANG", 5.f);
		setParam("VT_DIA_PQ_RATE", 15.f);
		setParam("VT_DIA_PQ_T", 1.f);
		int32_t use_airspd = 1;
		param_set(param_find("FW_USE_AIRSPD"), &use_airspd);

		_attc = new VtolAttitudeControl();
		_tr = new Tiltrotor(_attc);
		_tr->parameters_update();

		// Armed and airborne: otherwise can_transition_on_ground() completes
		// P1 regardless of the gate.
		_attc->get_control_mode()->flag_armed = true;
		_attc->get_land_detected()->landed = false;
	}

	void TearDown() override
	{
		delete _tr;
		delete _attc;
	}

	static void setParam(const char *name, float value)
	{
		param_set(param_find(name), &value);
	}

	void requestFixedWing(bool fw)
	{
		_attc->_transition_command = fw ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
					     : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	}

	// Fresh inputs for one cycle: quiet attitude/rate unless told otherwise.
	void feed(float cas, float tilt, float pitch_deg = 1.f, float rate_deg_s = 5.f)
	{
		const hrt_abstime now = hrt_absolute_time();

		airspeed_validated_s *aspd = _attc->get_airspeed();
		aspd->timestamp = now;
		aspd->calibrated_airspeed_m_s = cas;

		vehicle_attitude_s *att = _attc->get_att();
		att->timestamp = now;
		const matrix::Quatf q(matrix::Eulerf(0.f, math::radians(pitch_deg), 0.f));
		q.copyTo(att->q);

		vehicle_angular_velocity_s rate{};
		rate.timestamp = now;
		rate.xyz[1] = math::radians(rate_deg_s);
		_rate_pub.publish(rate);

		_tr->_tilt_control = tilt;
		_tr->_time_since_trans_start = 10.f; // past VT_TRANS_MIN_TM
	}

	void cycle() { _tr->update_vtol_state(); }
	Mode mode() const { return _tr->_vtol_mode; }
	void setMode(Mode m) { _tr->_vtol_mode = m; }
	bool windowOpen() const { return _tr->_pq_gate.windowOpen(); }

	// P1 with a running (not yet complete) quiet window.
	void openWindowInP1()
	{
		setMode(Mode::TRANSITION_FRONT_P1);
		requestFixedWing(true);
		feed(kFast, kTilt);
		cycle();
		ASSERT_EQ(mode(), Mode::TRANSITION_FRONT_P1);
		ASSERT_TRUE(windowOpen());
	}

	VtolAttitudeControl *_attc{nullptr};
	Tiltrotor *_tr{nullptr};
	uORB::Publication<vehicle_angular_velocity_s> _rate_pub{ORB_ID(vehicle_angular_velocity)};
};

TEST_F(TiltrotorPqGateTest, PassingTheGateEntersP2AndClearsTheWindow)
{
	openWindowInP1();
	px4_usleep(1100000); // > VT_DIA_PQ_T with every observed cycle passing
	feed(kFast, kTilt);
	cycle();
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P2);
	EXPECT_FALSE(windowOpen());

	feed(kFast, kTilt); // P2 continues (tilt < VT_TILT_FW)
	cycle();
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P2);
	EXPECT_FALSE(windowOpen());
}

TEST_F(TiltrotorPqGateTest, McRequestInP1ClearsTheWindow)
{
	openWindowInP1();
	requestFixedWing(false);
	feed(kFast, kTilt);
	cycle();
	EXPECT_EQ(mode(), Mode::MC_MODE);
	EXPECT_FALSE(windowOpen());
}

TEST_F(TiltrotorPqGateTest, FixedWingSystemFailureClearsTheWindow)
{
	openWindowInP1();
	_attc->get_vtol_vehicle_status()->fixed_wing_system_failure = true; // quad-chute path
	feed(kFast, kTilt);
	cycle();
	EXPECT_EQ(mode(), Mode::MC_MODE);
	EXPECT_FALSE(windowOpen());
}

TEST_F(TiltrotorPqGateTest, BackTransitionCyclesKeepTheWindowClear)
{
	openWindowInP1();
	setMode(Mode::FW_MODE);
	requestFixedWing(false);
	feed(kFast, kTilt);
	cycle();
	EXPECT_EQ(mode(), Mode::TRANSITION_BACK);
	EXPECT_FALSE(windowOpen());
}

TEST_F(TiltrotorPqGateTest, ReenteringP1StartsFromAnEmptyWindow)
{
	openWindowInP1();
	requestFixedWing(false);
	cycle(); // P1 -> MC
	requestFixedWing(true);
	feed(kFast, kTilt);
	cycle(); // MC -> P1 entry
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P1);
	EXPECT_FALSE(windowOpen());
	feed(kFast, kTilt);
	cycle();
	EXPECT_TRUE(windowOpen());
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P1); // a fresh window, not yet 1 s
}

// The gate runs on every P1 cycle: a speed or tilt sample below its
// threshold closes the window even though the quiet test is not reached
// first (the f959ecd94d short-circuit kept the window open here).
TEST_F(TiltrotorPqGateTest, SlowSampleInP1ClosesTheWindow)
{
	openWindowInP1();
	feed(kSlow, kTilt);
	cycle();
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P1);
	EXPECT_FALSE(windowOpen());
}

TEST_F(TiltrotorPqGateTest, LowTiltSampleInP1ClosesTheWindow)
{
	openWindowInP1();
	feed(kFast, 0.30f);
	cycle();
	EXPECT_EQ(mode(), Mode::TRANSITION_FRONT_P1);
	EXPECT_FALSE(windowOpen());
}

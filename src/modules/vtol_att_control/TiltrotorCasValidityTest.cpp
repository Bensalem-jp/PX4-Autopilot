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
 * @file TiltrotorCasValidityTest.cpp
 *
 * Module-level test of the opt-in CAS validity in Tiltrotor (VT_DIA_CAS_*):
 * the default is the legacy speed selection, an enabled check fails closed,
 * and the configuration is latched for the whole P1 phase (CE411-L2).
 * Thresholds are 137 ms / 310 ms on purpose: no value here is adopted.
 */

#define MODULE_NAME "vtol_att_control"

#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

class TiltrotorCasValidityTest : public ::testing::Test
{
public:
	using Mode = Tiltrotor::vtol_mode;

	static constexpr float kFast = 16.f;        // CAS above VT_ARSP_TRANS
	static constexpr float kGroundspeed = 20.f; // above VT_ARSP_TRANS
	static constexpr float kTilt = 0.35f;       // = VT_TILT_TRANS
	static constexpr int32_t kTv = 137000;
	static constexpr int32_t kTr = 310000;
	static constexpr hrt_abstime kCycleUs = 20000;

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
		setInt("FW_USE_AIRSPD", 1);
		param_reset(param_find("VT_DIA_CAS_EN"));
		param_reset(param_find("VT_DIA_CAS_TV"));
		param_reset(param_find("VT_DIA_CAS_TR"));
		param_reset(param_find("VT_DIA_CAS_DIS"));

		_attc = new VtolAttitudeControl();
		_tr = new Tiltrotor(_attc);
		_tr->parameters_update();

		_attc->get_control_mode()->flag_armed = true;
		_attc->get_land_detected()->landed = false;

		// Advertise _raw_pub first so it is uORB instance 0 (sensor index 1)
		// and _raw2_pub, when used, instance 1.
		publishRaw(hrt_absolute_time(), kFast);
	}

	void TearDown() override
	{
		delete _tr;
		delete _attc;
	}

	static void setParam(const char *name, float value) { param_set(param_find(name), &value); }
	static void setInt(const char *name, int32_t value) { param_set(param_find(name), &value); }

	void enable(int32_t tv, int32_t tr)
	{
		setInt("VT_DIA_CAS_EN", 1);
		setInt("VT_DIA_CAS_TV", tv);
		setInt("VT_DIA_CAS_TR", tr);
		_tr->parameters_update();
	}

	void publishRaw(hrt_abstime timestamp_sample, float ias)
	{
		airspeed_s raw{};
		raw.timestamp = hrt_absolute_time();
		raw.timestamp_sample = timestamp_sample;
		raw.indicated_airspeed_m_s = ias;
		raw.true_airspeed_m_s = ias;
		raw.confidence = 1.f;
		_raw_pub.publish(raw);
	}

	void publishRaw2(hrt_abstime timestamp_sample, float ias)
	{
		airspeed_s raw{};
		raw.timestamp = hrt_absolute_time();
		raw.timestamp_sample = timestamp_sample;
		raw.indicated_airspeed_m_s = ias;
		raw.true_airspeed_m_s = ias;
		raw.confidence = 1.f;
		_raw2_pub.publish(raw);
	}

	// What the selector reports in airspeed_validated (besides its timestamp).
	struct Selector {
		float cas{kFast};
		bool measurement_valid{true};
		int8_t index{1};
	};

	// One cycle of inputs: fresh validated CAS, quiet attitude/rate, groundspeed.
	void feed(hrt_abstime validated_age_us = 0)
	{
		const hrt_abstime now = hrt_absolute_time();

		airspeed_validated_s *aspd = _attc->get_airspeed();
		aspd->timestamp = now - validated_age_us;
		aspd->calibrated_airspeed_m_s = _selector.cas;
		aspd->airspeed_sensor_measurement_valid = _selector.measurement_valid;
		aspd->selected_airspeed_index = _selector.index;

		vehicle_local_position_s *pos = _attc->get_local_pos();
		pos->v_xy_valid = true;
		pos->vx = kGroundspeed;
		pos->vy = 0.f;

		vehicle_attitude_s *att = _attc->get_att();
		att->timestamp = now;
		const matrix::Quatf q(matrix::Eulerf(0.f, math::radians(1.f), 0.f));
		q.copyTo(att->q);

		vehicle_angular_velocity_s rate{};
		rate.timestamp = now;
		rate.xyz[1] = math::radians(5.f);
		_rate_pub.publish(rate);

		_tr->_tilt_control = kTilt;
		_tr->_time_since_trans_start = 10.f;
	}

	void requestFixedWing(bool fw)
	{
		_attc->_transition_command = fw ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
					     : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	}

	// MC -> P1 through the real entry path (the configuration latches on
	// the MC cycles before it).
	void enterP1()
	{
		_tr->_vtol_mode = Mode::MC_MODE;
		requestFixedWing(false);
		feed();
		_tr->update_vtol_state();
		requestFixedWing(true);
		feed();
		_tr->update_vtol_state();
		ASSERT_EQ(_tr->_vtol_mode, Mode::TRANSITION_FRONT_P1);
	}

	// Run P1 cycles for duration_us; fresh_raw publishes a fresh raw sample
	// every cycle. Returns true if P1 -> P2 happened.
	bool runP1(hrt_abstime duration_us, bool fresh_raw, hrt_abstime validated_age_us = 0)
	{
		const hrt_abstime end = hrt_absolute_time() + duration_us;

		while (hrt_absolute_time() < end) {
			if (fresh_raw) {
				publishRaw(hrt_absolute_time(), kFast);
			}

			if (_raw2_ias.has_value) {
				publishRaw2(hrt_absolute_time(), _raw2_ias.value);
			}

			feed(validated_age_us);
			_tr->update_vtol_state();

			if (_tr->_vtol_mode == Mode::TRANSITION_FRONT_P2) {
				return true;
			}

			px4_usleep(kCycleUs);
		}

		return false;
	}

	void updateParams() { _tr->parameters_update(); }
	void setSelector(float cas, bool measurement_valid, int8_t index) { _selector = Selector{cas, measurement_valid, index}; }
	void setSecondPitot(float ias)
	{
		_raw2_ias = OptionalIas{true, ias};
		publishRaw2(hrt_absolute_time(), ias); // advertise instance 1 before P1
	}
	const cas_validity::Config &latched() const { return _tr->_cas_cfg_latched; }
	cas_validity::Reason reason() const { return _tr->_cas_evaluation.reason; }

	VtolAttitudeControl *_attc{nullptr};
	Tiltrotor *_tr{nullptr};
	uORB::Publication<vehicle_angular_velocity_s> _rate_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::PublicationMulti<airspeed_s> _raw_pub{ORB_ID(airspeed)};
	uORB::PublicationMulti<airspeed_s> _raw2_pub{ORB_ID(airspeed)};

	// A second pitot publishing every runP1() cycle when has_value is set.
	struct OptionalIas {
		bool has_value{false};
		float value{0.f};
	};

	Selector _selector{};
	OptionalIas _raw2_ias{};
};

TEST_F(TiltrotorCasValidityTest, DefaultsAreDisabledAndUnset)
{
	EXPECT_FALSE(latched().enabled);
	EXPECT_EQ(latched().max_validated_age_us, 0);
	EXPECT_EQ(latched().max_raw_sample_age_us, 0);
	EXPECT_FLOAT_EQ(latched().max_disagreement_m_s, 0.f);
}

// Disabled: the legacy selection, including its groundspeed fallback when
// airspeed_validated is older than 1 s, and no raw-sample check at all.
TEST_F(TiltrotorCasValidityTest, DisabledKeepsTheLegacyFallback)
{
	publishRaw(hrt_absolute_time(), kFast); // then never again
	enterP1();
	EXPECT_TRUE(runP1(2000000, false, 1500000));
}

TEST_F(TiltrotorCasValidityTest, DisabledIgnoresAStaleRawSample)
{
	publishRaw(hrt_absolute_time(), kFast);
	enterP1();
	EXPECT_TRUE(runP1(2000000, false));
}

TEST_F(TiltrotorCasValidityTest, EnabledWithoutThresholdsNeverCompletesP1)
{
	setInt("VT_DIA_CAS_EN", 1);
	updateParams();
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::ThresholdUnset);
}

TEST_F(TiltrotorCasValidityTest, EnabledWithFreshSensorCompletesP1)
{
	enable(kTv, kTr);
	enterP1();
	EXPECT_TRUE(runP1(2000000, true));
}

TEST_F(TiltrotorCasValidityTest, EnabledRawStreamStopBlocks)
{
	enable(kTv, kTr);
	publishRaw(hrt_absolute_time(), kFast); // the pitot stops here
	enterP1();
	EXPECT_FALSE(runP1(2500000, false));
	EXPECT_EQ(reason(), cas_validity::Reason::RawStale);
}

TEST_F(TiltrotorCasValidityTest, EnabledStaleValidatedHasNoGroundspeedFallback)
{
	enable(kTv, kTr);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true, 1500000));
	EXPECT_EQ(reason(), cas_validity::Reason::ValidatedStale);
}

// CE411-L2: disabling the check during P1 does not release the hold.
TEST_F(TiltrotorCasValidityTest, DisablingDuringP1IsLatched)
{
	enable(kTv, kTr);
	publishRaw(hrt_absolute_time(), kFast);
	enterP1();
	EXPECT_FALSE(runP1(500000, false));

	setInt("VT_DIA_CAS_EN", 0);
	updateParams();
	EXPECT_TRUE(latched().enabled);
	EXPECT_FALSE(runP1(1500000, false));

	// It applies from the next transition on (legacy: no raw check).
	enterP1();
	EXPECT_FALSE(latched().enabled);
	EXPECT_TRUE(runP1(2000000, false));
}

// CE411-L2: raising T_R during P1 does not unlock a dead pitot.
TEST_F(TiltrotorCasValidityTest, RaisingAThresholdDuringP1IsLatched)
{
	enable(kTv, kTr);
	publishRaw(hrt_absolute_time(), kFast);
	enterP1();
	EXPECT_FALSE(runP1(500000, false));

	setInt("VT_DIA_CAS_TR", 100000000);
	updateParams();
	EXPECT_EQ(latched().max_raw_sample_age_us, static_cast<int32_t>(kTr));
	EXPECT_FALSE(runP1(1500000, false));
}

// Enabling during P1 also waits for the next transition (symmetric latch).
TEST_F(TiltrotorCasValidityTest, EnablingDuringP1IsLatched)
{
	enterP1();
	enable(kTv, kTr);
	EXPECT_FALSE(latched().enabled);
}

// Sensor health at module level: the airspeed_validated flags are wired.
TEST_F(TiltrotorCasValidityTest, InvalidMeasurementFlagBlocks)
{
	enable(kTv, kTr);
	setSelector(kFast, false, 1);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::MeasurementInvalid);
}

TEST_F(TiltrotorCasValidityTest, GroundspeedMinusWindIndexBlocks)
{
	enable(kTv, kTr);
	setSelector(kFast, true, 0);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::IndexNotSensor);
}

TEST_F(TiltrotorCasValidityTest, NonFiniteCasBlocks)
{
	enable(kTv, kTr);
	setSelector(NAN, true, 1);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::NonFinite);
}

// Two pitots (uORB instances 0 and 1).
TEST_F(TiltrotorCasValidityTest, DisagreeingSecondPitotBlocks)
{
	enable(kTv, kTr);
	setParam("VT_DIA_CAS_DIS", 1.5f);
	updateParams();
	setSecondPitot(kFast + 5.f);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::Disagreement);
}

TEST_F(TiltrotorCasValidityTest, AgreeingSecondPitotCompletesP1)
{
	enable(kTv, kTr);
	setParam("VT_DIA_CAS_DIS", 1.5f);
	updateParams();
	setSecondPitot(kFast + 1.f);
	enterP1();
	EXPECT_TRUE(runP1(2000000, true));
}

TEST_F(TiltrotorCasValidityTest, SecondPitotWithoutDisagreementThresholdBlocks)
{
	enable(kTv, kTr);
	setSecondPitot(kFast);
	enterP1();
	EXPECT_FALSE(runP1(2000000, true));
	EXPECT_EQ(reason(), cas_validity::Reason::ThresholdUnset);
}

// Index 2 reads uORB instance 1: the second pitot alone is fresh.
TEST_F(TiltrotorCasValidityTest, IndexTwoReadsTheSecondInstance)
{
	enable(kTv, kTr);
	setSelector(kFast, true, 2);
	publishRaw(hrt_absolute_time(), kFast); // instance 0 then stops
	setSecondPitot(kFast);
	enterP1();
	EXPECT_TRUE(runP1(2000000, false));
}

// CE MED-2 at module level: one invalid-flag cycle mid-window.
TEST_F(TiltrotorCasValidityTest, HoldBridgesOneBadCycleOnlyWhenSet)
{
	for (const int32_t hold_us : {0, 60000}) {
		enable(kTv, kTr);
		setInt("VT_DIA_CAS_HLD", hold_us);
		updateParams();
		enterP1();

		const hrt_abstime start = hrt_absolute_time();
		EXPECT_FALSE(runP1(500000, true));
		setSelector(kFast, false, 1);
		EXPECT_FALSE(runP1(1, true)); // exactly one cycle
		setSelector(kFast, true, 1);
		EXPECT_TRUE(runP1(2000000, true));
		const hrt_abstime elapsed = hrt_absolute_time() - start;

		if (hold_us > 0) {
			EXPECT_LT(elapsed, 1300000u) << "hold " << hold_us; // window kept running

		} else {
			EXPECT_GT(elapsed, 1450000u) << "hold " << hold_us; // window restarted after 0.5 s
		}
	}
}

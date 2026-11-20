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
 * @file VtolCasArmingCheckTest.cpp
 *
 * Commander pre-arm check for the opt-in CAS validity (VT_DIA_CAS_*,
 * CE MED-1 / MED-2): VtolChecks with real uORB airspeed topics. The
 * thresholds 137 ms / 310 ms are test values, not proposals.
 */

#include <gtest/gtest.h>

#include "Common.hpp"
#include "checks/vtolCheck.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/event.h>

using namespace time_literals;

class VtolCasArmingCheckTest : public ::testing::Test
{
public:
	static constexpr int32_t kTv = 137000;
	static constexpr int32_t kTr = 310000;
	// Test publish period, shorter than the PX4 selector's 100 ms to leave
	// wall-clock margin below T_V (137 ms) on a loaded host.
	static constexpr hrt_abstime kSelectorPeriodUs = 50000;

	void SetUp() override
	{
		orb_advertise(ORB_ID(event), nullptr);
		param_control_autosave(false);
		setInt("VT_DIA_CAS_EN", 0);
		setInt("VT_DIA_CAS_TV", 0);
		setInt("VT_DIA_CAS_TR", 0);
		setFloat("VT_DIA_CAS_DIS", 0.f);
		setInt("VT_DIA_CAS_HLD", 0);
		setFloat("VT_DIA_PQ_T", 1.f);
		setInt("FW_USE_AIRSPD", 1);
		setInt("ASPD_PRIMARY", 1);
		setInt("SYS_HAS_NUM_ASPD", 1);

		_status.is_vtol = true;
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_checks = new VtolChecks();
	}

	void TearDown() override { delete _checks; }

	static void setInt(const char *name, int32_t value) { param_set(param_find(name), &value); }
	static void setFloat(const char *name, float value) { param_set(param_find(name), &value); }

	void configure(int32_t tv, int32_t tr)
	{
		setInt("VT_DIA_CAS_EN", 1);
		setInt("VT_DIA_CAS_TV", tv);
		setInt("VT_DIA_CAS_TR", tr);
	}

	// A healthy selector + pitot publishing every kSelectorPeriodUs, checked in between.
	bool runChecks(int periods, bool measurement_valid = true)
	{
		bool can_arm = false;

		for (int k = 0; k < periods; k++) {
			const hrt_abstime now = hrt_absolute_time();

			airspeed_s raw{};
			raw.timestamp_sample = now;
			raw.timestamp = now;
			raw.indicated_airspeed_m_s = 0.5f;
			raw.true_airspeed_m_s = 0.5f;
			raw.confidence = 1.f;
			_raw_pub.publish(raw);

			airspeed_validated_s validated{};
			validated.timestamp = now;
			validated.calibrated_airspeed_m_s = 0.5f;
			validated.airspeed_sensor_measurement_valid = measurement_valid;
			validated.selected_airspeed_index = 1;
			_validated_pub.publish(validated);

			can_arm = checkOnce();
			px4_usleep(kSelectorPeriodUs);
		}

		return can_arm;
	}

	bool checkOnce()
	{
		_checks->updateParams();
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0_s};
		reporter.reset();
		const Context context{_status};
		_checks->checkAndReport(context, reporter);
		reporter.finalize();
		reporter.report(false, false);
		return reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	}

	vehicle_status_s _status{};
	VtolChecks *_checks{nullptr};
	uORB::PublicationMulti<airspeed_s> _raw_pub{ORB_ID(airspeed)};
	uORB::Publication<airspeed_validated_s> _validated_pub{ORB_ID(airspeed_validated)};
};

TEST_F(VtolCasArmingCheckTest, DisabledNeverRefuses)
{
	EXPECT_TRUE(runChecks(5));
	setInt("VT_DIA_CAS_TV", 1); // garbage while disabled
	EXPECT_TRUE(runChecks(2, false));
}

TEST_F(VtolCasArmingCheckTest, HealthyConfigurationArms)
{
	configure(kTv, kTr);
	EXPECT_TRUE(runChecks(5));
}

TEST_F(VtolCasArmingCheckTest, UnsetThresholdRefuses)
{
	configure(kTv, 0);
	EXPECT_FALSE(runChecks(5));
}

TEST_F(VtolCasArmingCheckTest, WindowTooShortRefuses)
{
	configure(kTv, kTr);
	setFloat("VT_DIA_PQ_T", 0.6f); // 2 * 310 ms >= 0.6 s
	EXPECT_FALSE(runChecks(5));
	setFloat("VT_DIA_PQ_T", 1.f);
	setInt("VT_DIA_CAS_HLD", 380000); // 620 + 380 ms >= 1 s
	EXPECT_FALSE(runChecks(2));
}

TEST_F(VtolCasArmingCheckTest, UnhealthySensorRefuses)
{
	configure(kTv, kTr);
	EXPECT_FALSE(runChecks(5, false));
}

TEST_F(VtolCasArmingCheckTest, ThresholdNotAboveTheSelectorIntervalRefuses)
{
	configure(static_cast<int32_t>(kSelectorPeriodUs) - 20000, kTr); // T_V below the measured interval
	EXPECT_FALSE(runChecks(5));
}

TEST_F(VtolCasArmingCheckTest, NoIntervalMeasuredYetRefuses)
{
	configure(kTv, kTr);
	EXPECT_FALSE(checkOnce()); // nothing published to this check yet
}

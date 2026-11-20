/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "../Common.hpp"

#include <modules/vtol_att_control/cas_validity.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vtol_vehicle_status.h>

class VtolChecks : public HealthAndArmingCheckBase
{
public:
	VtolChecks() = default;
	~VtolChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	void checkWeatherModeUnchanged(const Context &context, Report &reporter);

	/**
	 * Opt-in CAS validity (VT_DIA_CAS_EN=1 with FW_USE_AIRSPD=1) pre-arm
	 * check (CE MED-1 / MED-2): thresholds set, 2 * max(T) + hold below
	 * VT_DIA_PQ_T, a healthy airspeed sensor now, and VT_DIA_CAS_TV above the
	 * measured airspeed_validated publish interval.
	 */
	void checkCasValidity(const Context &context, Report &reporter);

	/** Track the longest airspeed_validated publish interval (one-message steps only). */
	void updateSelectorInterval();

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::SubscriptionMultiArray<airspeed_s, cas_validity::kMaxRawInstances> _airspeed_subs{ORB_ID::airspeed};
	hrt_abstime _last_validated_timestamp{0};
	hrt_abstime _max_selector_interval_us{0};
	hrt_abstime _interval_window_start{0};
	hrt_abstime _max_interval_in_window_us{0};

	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};

	// VT_DIA_WX_MODE as it was the first time this check ran, which is the value
	// the airframe script read at boot. See checkWeatherModeUnchanged().
	int32_t _boot_weather_mode{0};
	bool _boot_weather_mode_latched{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::VT_DIA_WX_MODE>) _param_vt_dia_wx_mode
				       )
};

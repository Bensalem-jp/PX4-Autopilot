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

#include "vtolCheck.hpp"

using namespace time_literals;

void VtolChecks::checkWeatherModeUnchanged(const Context &context, Report &reporter)
{
	const int32_t weather_mode = _param_vt_dia_wx_mode.get();

	// Latched on the first run, i.e. at boot: commander starts after the airframe
	// script has read VT_DIA_WX_MODE and written the parameters that differ between
	// a calm and a windy mission. Nothing applies those again later, so a mode
	// changed during flight prep -- without the reboot the parameter's
	// @reboot_required asks for -- would fly the previous regime with no warning at
	// all, e.g. a calm configuration into 20 m/s of wind.
	//
	// The latch lives here rather than in vtol_att_control on purpose: this check
	// has to hold whenever commander can arm, and vtol_vehicle_status is only
	// published once the rate control loop is running, so a check keyed off that
	// topic would silently pass whenever the topic is absent.
	if (!_boot_weather_mode_latched) {
		_boot_weather_mode = weather_mode;
		_boot_weather_mode_latched = true;
	}

	// Not sticky on purpose: nothing in the firmware acts on VT_DIA_WX_MODE after
	// boot, so a mode set back to the booted one leaves the aircraft in exactly the
	// configuration the script applied, and refusing to arm then would be a false
	// alarm.
	if (!context.status().is_vtol || weather_mode == _boot_weather_mode) {
		return;
	}

	/* EVENT
	 */
	reporter.armingCheckFailure(NavModes::All, health_component_t::system,
				    events::ID("check_vtol_weather_mode_changed"),
				    events::Log::Error,
				    "Weather mode changed since boot. Reboot to apply it before flying");

	if (reporter.mavlink_log_pub()) {
		mavlink_log_info(reporter.mavlink_log_pub(), "Preflight Fail: weather mode changed, reboot required\t");
	}
}

// The CAS validity parameters live in vtol_att_control, airspeed_selector,
// sensors and fw_att_control; read them by name so this check builds on every
// target. A missing parameter reads as the fallback (VT_DIA_CAS_EN 0: off).
static int32_t paramInt(const char *name, int32_t fallback)
{
	int32_t value = fallback;
	const param_t handle = param_find(name);

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value;
}

static float paramFloat(const char *name, float fallback)
{
	float value = fallback;
	const param_t handle = param_find(name);

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value;
}

// The interval maximum is kept over two consecutive windows of this length,
// so an old long interval ages out (bookkeeping, not a threshold).
static constexpr hrt_abstime kSelectorIntervalWindowUs = 10_s;

void VtolChecks::updateSelectorInterval()
{
	const uint32_t generation_before = _airspeed_validated_sub.get_last_generation();
	airspeed_validated_s validated;

	if (!_airspeed_validated_sub.update(&validated)) {
		return;
	}

	const uint32_t published = _airspeed_validated_sub.get_last_generation() - generation_before;
	const hrt_abstime now = hrt_absolute_time();

	// Only a step of exactly one publication is an interval; a skipped
	// message would read as a long interval and a repeated one as zero.
	if (published == 1 && _last_validated_timestamp != 0 && validated.timestamp > _last_validated_timestamp) {
		const hrt_abstime interval = validated.timestamp - _last_validated_timestamp;

		if (_interval_window_start == 0 || now - _interval_window_start > kSelectorIntervalWindowUs) {
			_interval_window_start = now;
			_max_selector_interval_us = math::max(_max_interval_in_window_us, interval);
			_max_interval_in_window_us = interval;

		} else {
			_max_interval_in_window_us = math::max(_max_interval_in_window_us, interval);
			_max_selector_interval_us = math::max(_max_selector_interval_us, interval);
		}
	}

	_last_validated_timestamp = validated.timestamp;
}

void VtolChecks::checkCasValidity(const Context &context, Report &reporter)
{
	updateSelectorInterval();

	if (!context.status().is_vtol) {
		return;
	}

	cas_validity::ArmingInputs in{};
	in.cfg.enabled = paramInt("VT_DIA_CAS_EN", 0) != 0;
	in.cfg.max_validated_age_us = paramInt("VT_DIA_CAS_TV", 0);
	in.cfg.max_raw_sample_age_us = paramInt("VT_DIA_CAS_TR", 0);
	in.cfg.max_disagreement_m_s = paramFloat("VT_DIA_CAS_DIS", 0.f);
	in.cfg.max_unusable_hold_us = paramInt("VT_DIA_CAS_HLD", 0);
	in.fw_use_airspd = paramInt("FW_USE_AIRSPD", 0) != 0;
	in.pq_window_s = paramFloat("VT_DIA_PQ_T", 0.f);
	in.aspd_primary = paramInt("ASPD_PRIMARY", -1);
	in.sys_has_num_aspd = paramInt("SYS_HAS_NUM_ASPD", -1);
	in.max_selector_interval_us = _max_selector_interval_us;

	if (!in.cfg.enabled || !in.fw_use_airspd) {
		return;
	}

	cas_validity::Inputs now_in{};
	now_in.now = hrt_absolute_time();
	now_in.fw_use_airspd = true;

	airspeed_validated_s validated{};

	if (_airspeed_validated_sub.copy(&validated)) {
		now_in.validated_timestamp = validated.timestamp;
		now_in.calibrated_airspeed_m_s = validated.calibrated_airspeed_m_s;
		now_in.measurement_valid = validated.airspeed_sensor_measurement_valid;
		now_in.selected_index = validated.selected_airspeed_index;
	}

	for (int i = 0; i < cas_validity::kMaxRawInstances; i++) {
		now_in.raw[i].present = _airspeed_subs[i].advertised();

		airspeed_s raw{};

		if (now_in.raw[i].present && _airspeed_subs[i].copy(&raw)) {
			now_in.raw[i].timestamp = raw.timestamp;
			now_in.raw[i].timestamp_sample = raw.timestamp_sample;
			now_in.raw[i].indicated_airspeed_m_s = raw.indicated_airspeed_m_s;
		}

		if (now_in.raw[i].present) {
			in.raw_instances++;
		}
	}

	in.sensor = cas_validity::evaluate(now_in, in.cfg);

	const uint16_t failures = cas_validity::checkArming(in);

	if (failures == 0) {
		return;
	}

	/* EVENT
	 * @description
	 * VT_DIA_CAS_EN=1 with FW_USE_AIRSPD=1, and the configuration or the airspeed sensor would not let the
	 * front transition complete safely. Failure bits: 1 threshold unset, 2 disagreement threshold unset,
	 * 4 2*max(VT_DIA_CAS_TV, VT_DIA_CAS_TR)+VT_DIA_CAS_HLD reaches VT_DIA_PQ_T, 8 no airspeed sensor,
	 * 16 airspeed sensor unhealthy, 32 selector interval not measured yet, 64 VT_DIA_CAS_TV not above the
	 * measured airspeed_validated interval.
	 */
	reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::differential_pressure,
					      events::ID("check_vtol_cas_validity"),
					      events::Log::Error,
					      "CAS validity for the front transition not satisfied (failure bits {1})", failures);

	if (reporter.mavlink_log_pub()) {
		mavlink_log_info(reporter.mavlink_log_pub(), "Preflight Fail: CAS validity (bits %u)\t",
				 static_cast<unsigned>(failures));
	}
}

void VtolChecks::checkAndReport(const Context &context, Report &reporter)
{
	checkWeatherModeUnchanged(context, reporter);
	checkCasValidity(context, reporter);

	vtol_vehicle_status_s vtol_vehicle_status;

	if (_vtol_vehicle_status_sub.copy(&vtol_vehicle_status)) {
		reporter.failsafeFlags().vtol_fixed_wing_system_failure = vtol_vehicle_status.fixed_wing_system_failure;

		if (reporter.failsafeFlags().vtol_fixed_wing_system_failure) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_vtol_fixed_wing_system_failure"),
						    events::Log::Error,
						    "VTOL fixed-wing system failure detected. Verify reason for failure, and reboot the vehicle once confirmed safe");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_info(reporter.mavlink_log_pub(), "Preflight Fail: VTOL fixed-wing system failure detected\t");
			}
		}
	}
}

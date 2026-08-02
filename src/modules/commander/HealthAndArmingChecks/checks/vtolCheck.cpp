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

void VtolChecks::checkAndReport(const Context &context, Report &reporter)
{
	checkWeatherModeUnchanged(context, reporter);

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

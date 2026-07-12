/****************************************************************************
 *
 *   Copyright (c) 2026 Diamond VTOL Project. All rights reserved.
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
 * @file ActuatorEffectivenessDiamondVTOL.cpp
 *
 * Actuator effectiveness for Diamond BWB-FIW VTOL with louver yaw vectoring.
 *
 * @author Diamond VTOL Project
 */

#include "ActuatorEffectivenessDiamondVTOL.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

ActuatorEffectivenessDiamondVTOL::ActuatorEffectivenessDiamondVTOL(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _control_surfaces(this), _tilts(this)
{
	_param_handles.com_spoolup_time = param_find("COM_SPOOLUP_TIME");
	_param_handles.ca_diamond_louver_k = param_find("CA_DIA_LVR_K");
	_param_handles.ca_diamond_louver_max = param_find("CA_DIA_LVR_MAX");
	_param_handles.ca_diamond_fiw_arm = param_find("CA_DIA_FIW_ARM");
	_param_handles.ca_diamond_fiw_ct = param_find("CA_DIA_FIW_CT");

	updateParams();
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessDiamondVTOL::updateParams()
{
	ModuleParams::updateParams();

	param_get(_param_handles.com_spoolup_time, &_param_spoolup_time);

	if (_param_handles.ca_diamond_louver_k != PARAM_INVALID) {
		param_get(_param_handles.ca_diamond_louver_k, &_param_louver_k);
	}

	if (_param_handles.ca_diamond_louver_max != PARAM_INVALID) {
		param_get(_param_handles.ca_diamond_louver_max, &_param_louver_max_rad);
	}

	if (_param_handles.ca_diamond_fiw_arm != PARAM_INVALID) {
		param_get(_param_handles.ca_diamond_fiw_arm, &_param_fiw_arm);
	}

	if (_param_handles.ca_diamond_fiw_ct != PARAM_INVALID) {
		param_get(_param_handles.ca_diamond_fiw_ct, &_param_fiw_ct);
	}
}

bool
ActuatorEffectivenessDiamondVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!_collective_tilt_updated && external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors (matrix 0)
	configuration.selected_matrix = 0;
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	_mc_rotors.enableThreeDimensionalThrust(false);

	// Update matrix with tilts in vertical position when triggered by config change
	const float collective_tilt_control_applied = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) ?
			-1.f : _last_collective_tilt_control;
	_untiltable_motors = _mc_rotors.updateAxisFromTilts(_tilts, collective_tilt_control_applied)
			     << configuration.num_actuators[(int)ActuatorType::MOTORS];

	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);
	_motors = _mc_rotors.getMotors();

	// Tilts (matrix 0, after rotors)
	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	_tilts.updateTorqueSign(_mc_rotors.geometry(), true /* disable pitch */);
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Louver servos (matrix 0, after tilts) -- 2 columns for yaw vectoring
	// At this point the effectiveness is set to a NOMINAL value; the actual
	// thrust-dependent scaling happens in updateSetpoint() each cycle.
	_first_louver_idx = configuration.num_actuators_matrix[configuration.selected_matrix];

	// Louver L: positive deflection -> positive yaw moment (CW seen from top in FRD)
	// Nominal effectiveness: k * T_max * arm [Nm per normalized servo cmd]
	// The normalized servo command maps [-1, 1] to [-louver_max_rad, +louver_max_rad]
	const float nominal_yaw_eff = _param_louver_k * _param_fiw_ct * _param_fiw_arm * _param_louver_max_rad;

	{
		// Louver L (left FIW fan, at PY = -arm in FRD)
		// Positive louver L deflection -> side force in +Y (FRD) -> positive yaw (nose right)
		Vector3f louver_l_torque{0.f, 0.f, nominal_yaw_eff};
		Vector3f louver_l_thrust{0.f, 0.f, 0.f};  // no direct thrust contribution
		configuration.addActuator(ActuatorType::SERVOS, louver_l_torque, louver_l_thrust);
	}

	{
		// Louver R (right FIW fan, at PY = +arm in FRD)
		// Positive louver R deflection -> side force in +Y (FRD) -> negative yaw (nose left)
		// Differential: L positive + R negative = yaw CW
		Vector3f louver_r_torque{0.f, 0.f, -nominal_yaw_eff};
		Vector3f louver_r_thrust{0.f, 0.f, 0.f};
		configuration.addActuator(ActuatorType::SERVOS, louver_r_torque, louver_r_thrust);
	}

	// Control Surfaces (matrix 1)
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	// If config change, force re-update next cycle with correct tilt
	_collective_tilt_updated = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	return (mc_rotors_added_successfully && surfaces_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessDiamondVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 1) {
		// Apply flaps
		normalized_unsigned_setpoint_s flaps_setpoint;

		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// Apply spoilers
		normalized_unsigned_setpoint_s spoilers_setpoint;

		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

void ActuatorEffectivenessDiamondVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	if (matrix_index == 0) {
		// --- Tilt handling (same as TiltrotorVTOL) ---
		tiltrotor_extra_controls_s tiltrotor_extra_controls;

		if (_tiltrotor_extra_controls_sub.copy(&tiltrotor_extra_controls)) {
			float control_collective_tilt = tiltrotor_extra_controls.collective_tilt_normalized_setpoint * 2.f - 1.f;

			control_collective_tilt = control_collective_tilt < -0.99f ? -1.f : control_collective_tilt;
			control_collective_tilt = control_collective_tilt > 0.99f ? 1.f : control_collective_tilt;

			if (!PX4_ISFINITE(_last_collective_tilt_control)) {
				_last_collective_tilt_control = control_collective_tilt;

			} else if (fabsf(control_collective_tilt - _last_collective_tilt_control) > 0.01f) {
				_collective_tilt_updated = true;
				_last_collective_tilt_control = control_collective_tilt;
			}

			if (_flight_phase == FlightPhase::TRANSITION_HF_TO_FF) {
				_last_collective_tilt_control = math::constrain(_last_collective_tilt_control, -1.f, 0.f);

			} else if (_flight_phase == FlightPhase::TRANSITION_FF_TO_HF) {
				_last_collective_tilt_control = -1.f;
			}

			for (int i = 0; i < _tilts.count(); ++i) {
				if (_tilts.config(i).tilt_direction == ActuatorEffectivenessTilts::TiltDirection::TowardsFront) {
					if (throttleSpoolupFinished() || _flight_phase != FlightPhase::HOVER_FLIGHT) {
						actuator_sp(i + _first_tilt_idx) += control_collective_tilt;

					} else {
						actuator_sp(i + _first_tilt_idx) = NAN;
					}
				}
			}

			// In FW, directly use throttle sp for motors
			if (_flight_phase == FlightPhase::FORWARD_FLIGHT) {
				for (int i = 0; i < _first_tilt_idx; ++i) {
					actuator_sp(i) = tiltrotor_extra_controls.collective_thrust_normalized_setpoint;
				}
			}
		}

		if (_flight_phase == FlightPhase::FORWARD_FLIGHT) {
			stopMaskedMotorsWithZeroThrust(_motors & ~_untiltable_motors, actuator_sp);
		}

		// --- Louver thrust-dependent yaw scaling ---
		// The allocator has already placed a nominal yaw command into the louver
		// servo slots based on the static effectiveness matrix. We now SCALE the
		// louver commands by the ratio of current FIW thrust to max thrust.
		//
		// Rationale: the effectiveness matrix was built with nominal_yaw_eff =
		//   k * T_max * arm * max_rad. The allocator assumes this is constant.
		//   In reality, effectiveness is proportional to actual fan thrust:
		//   actual_eff = k * T_current * arm * max_rad
		//   So we need to scale the allocated command by T_max / T_current
		//   (inverse scaling: if thrust is low, we need MORE deflection).
		//
		// However, we must also CLAMP to [-1, 1] and handle zero-thrust gracefully.
		// When thrust is zero, louver has zero authority -> set to zero.

		if (_flight_phase == FlightPhase::HOVER_FLIGHT || _flight_phase == FlightPhase::TRANSITION_HF_TO_FF) {
			// Read current FIW motor setpoints (motors 0 and 1 are FIW L and R)
			// Motor setpoints are in [0, 1] for non-reversible motors
			const float motor_l_sp = math::max(actuator_sp(0), 0.f);
			const float motor_r_sp = math::max(actuator_sp(1), 0.f);

			// Average FIW thrust ratio (use average of both fans for yaw scaling)
			const float avg_thrust_ratio = (motor_l_sp + motor_r_sp) * 0.5f;

			// Minimum thrust ratio below which louver has negligible authority
			const float MIN_THRUST_RATIO = 0.05f;

			if (avg_thrust_ratio > MIN_THRUST_RATIO) {
				// Scale factor: effectiveness was computed at max thrust (ratio=1.0)
				// Current effectiveness is reduced by thrust ratio, so we need to
				// command MORE deflection to achieve the same yaw torque.
				// But we CLAMP the result, so if thrust is very low, we saturate.
				const float scale = 1.f / avg_thrust_ratio;

				// Apply scale to louver commands
				float louver_l = actuator_sp(_first_louver_idx) * scale;
				float louver_r = actuator_sp(_first_louver_idx + 1) * scale;

				// Clamp to servo limits [-1, 1]
				louver_l = math::constrain(louver_l, -1.f, 1.f);
				louver_r = math::constrain(louver_r, -1.f, 1.f);

				actuator_sp(_first_louver_idx) = louver_l;
				actuator_sp(_first_louver_idx + 1) = louver_r;

				// Track saturation for anti-windup feedback
				_louver_yaw_saturated_pos = (louver_l >= 0.99f) || (louver_r <= -0.99f);
				_louver_yaw_saturated_neg = (louver_l <= -0.99f) || (louver_r >= 0.99f);

			} else {
				// Zero or negligible thrust -- zero out louver commands
				actuator_sp(_first_louver_idx) = 0.f;
				actuator_sp(_first_louver_idx + 1) = 0.f;
				_louver_yaw_saturated_pos = true;
				_louver_yaw_saturated_neg = true;
			}
		}
	}
}

void ActuatorEffectivenessDiamondVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// Update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_stopped_motors_mask |= _untiltable_motors;
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_stopped_motors_mask = 0;
		break;
	}
}

void ActuatorEffectivenessDiamondVTOL::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	if (matrix_index == 1) {
		return;
	}

	// Report yaw saturation from both tilt and louver systems
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos || _louver_yaw_saturated_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg || _louver_yaw_saturated_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}

bool ActuatorEffectivenessDiamondVTOL::throttleSpoolupFinished()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;
	}

	return _armed && hrt_elapsed_time(&_armed_time) > _param_spoolup_time * 1_s;
}

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
 * @file ActuatorEffectivenessDiamondVTOL.hpp
 *
 * Actuator effectiveness for Diamond BWB-FIW VTOL with louver yaw vectoring.
 *
 * Extends the tiltrotor VTOL composition with 2 additional louver servo
 * columns whose yaw effectiveness is thrust-dependent:
 *   Mz_louver = k_louver * T_fiw_current * y_arm * deflection_normalized
 *
 * @author Diamond VTOL Project
 */

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessDiamondVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessDiamondVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessDiamondVTOL() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	int numMatrices() const override { return 2; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
		normalize[1] = false;
	}

	void setFlightPhase(const FlightPhase &flight_phase) override;

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "Diamond VTOL"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:
	bool _collective_tilt_updated{true};
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;
	ActuatorEffectivenessTilts _tilts;

	uint32_t _motors{};
	uint32_t _untiltable_motors{};

	int _first_control_surface_idx{0}; ///< applies to matrix 1
	int _first_tilt_idx{0}; ///< applies to matrix 0
	int _first_louver_idx{0}; ///< louver servo start index in matrix 0

	float _last_collective_tilt_control{NAN};

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

	uORB::Subscription _tiltrotor_extra_controls_sub{ORB_ID(tiltrotor_extra_controls)};

	/// Louver yaw saturation tracking
	bool _louver_yaw_saturated_pos{false};
	bool _louver_yaw_saturated_neg{false};

private:

	void updateParams() override;

	struct ParamHandles {
		param_t com_spoolup_time;
		param_t ca_diamond_louver_k;
		param_t ca_diamond_louver_max;
		param_t ca_diamond_fiw_arm;
		param_t ca_diamond_fiw_ct;
	};

	ParamHandles _param_handles{};

	float _param_spoolup_time{1.f};

	/// Louver effectiveness coefficient: side force per fan thrust per radian [N/(N*rad)]
	/// CFD-validated: 0.7-1.2 N/deg ~ 40-69 N/(N*rad) at max thrust
	/// Linearized: k = dF_side / (T_fan * d_angle_rad)
	/// Default 0.015: 0.015 * 80N * 0.325m = 0.39 Nm per rad of deflection per fan
	float _param_louver_k{0.015f};

	/// Maximum louver deflection [rad], default 20 deg
	float _param_louver_max_rad{0.349f};

	/// FIW y-arm: distance from CG to each FIW fan in body Y [m]
	float _param_fiw_arm{0.325f};

	/// FIW max thrust coefficient [N] used to denormalize motor setpoint
	float _param_fiw_ct{80.0f};

	// Tilt handling during motor spoolup
	bool throttleSpoolupFinished();
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};
};

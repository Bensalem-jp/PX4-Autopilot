/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/topics/vehicle_angular_velocity.h>

class Tiltrotor : public VtolType
{

public:

	Tiltrotor(VtolAttitudeControl *_att_controller);
	~Tiltrotor() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void fill_actuator_outputs() override;
	void update_mc_state() override;
	void update_fw_state() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:
	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	/**
	 * Specific to tiltrotor with vertical aligned rear engine/s.
	 * These engines need to be shut down in fw mode. During the back-transition
	 * they need to idle otherwise they need too much time to spin up for mc mode.
	 */

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	uORB::Publication<tiltrotor_extra_controls_s>	_tiltrotor_extra_controls_pub{ORB_ID(tiltrotor_extra_controls)};

	float _tilt_control{0.0f};		/**< actuator value for the tilt servo */

	void parameters_update() override;
	float timeUntilMotorsAreUp();
	float moveLinear(float start, float stop, float progress);

	void blendThrottleDuringBacktransition(const float scale, const float target_throttle);
	bool isFrontTransitionCompletedBase() override;

	/**
	 * Diamond pitch-quiet gate (T3-2): hold the P1 plateau (VT_TILT_TRANS)
	 * until |pitch| and |pitch rate| stay below thresholds for a sustained
	 * window. VT_DIA_PQ_T <= 0 disables the gate (stock behavior).
	 * T1 sim finding: advancing tilt at the transition airspeed while the
	 * pitch loop is still ringing tips the vehicle (run-25 signature).
	 */
	bool isPitchQuiet();

	// Physical height above ground (pusher_assist source ladder).
	// The land detector is NOT a ground oracle at high thrust: its
	// low-thrust test de-asserts landed during a pinned takeoff.
	float distToGround() const;

	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	hrt_abstime _pitch_quiet_since{0};
	// Heading captured at front-transition start (P1 yaw pin)
	float _transition_start_yaw{0.f};
	// Slewed descent wing-unload bias (T10); 0 when inactive.
	float _dst_bias_state{0.f};
	hrt_abstime _dst_bias_last_us{0};
	// Wind-descent mode latch (bias + collective floor share it).
	bool _dst_mode_active{false};
	// Set when the back transition completes; the wind unload runs
	// from the first post-BT MC cycle (racing the MPC thrust cut).
	bool _post_bt_mc{false};
	// Rate limiter for the BT exit-gate diagnostic print (1 Hz).
	hrt_abstime _bt_gate_log_ts{0};
	// Low-pass state for the landing-regime elevon pitch DC-trim
	// (0.5 s tau; kills the 12.5 Hz passthrough flap while the
	// surfaces carry the slow wing-moment trim).
	float _elev_trim_state{0.f};
	hrt_abstime _elev_trim_last_us{0};


	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::VT_DIA_ELEV_QB>) _param_vt_dia_elev_qbar,
					(ParamFloat<px4::params::VT_DIA_ELEV_QC>) _param_vt_dia_elev_qcap,
					(ParamFloat<px4::params::VT_DIA_ROT_P>) _param_vt_dia_rot_p,
					(ParamFloat<px4::params::VT_DIA_DST_P>) _param_vt_dia_dst_p,
					(ParamFloat<px4::params::VT_DIA_PQ_ANG>) _param_vt_dia_pq_ang,
					(ParamFloat<px4::params::VT_DIA_PQ_RATE>) _param_vt_dia_pq_rate,
					(ParamFloat<px4::params::VT_DIA_PQ_T>) _param_vt_dia_pq_t,
					(ParamFloat<px4::params::VT_TILT_MC>) _param_vt_tilt_mc,
					(ParamFloat<px4::params::VT_TILT_TRANS>) _param_vt_tilt_trans,
					(ParamFloat<px4::params::VT_TILT_FW>) _param_vt_tilt_fw,
					(ParamFloat<px4::params::VT_TRANS_P2_DUR>) _param_vt_trans_p2_dur,
					(ParamFloat<px4::params::VT_BT_TILT_DUR>) _param_vt_bt_tilt_dur
				       )

};
#endif

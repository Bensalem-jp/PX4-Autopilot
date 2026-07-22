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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;

#define FRONTTRANS_THR_MIN 0.25f
// Diamond wing constants for qbar-scaled attitude limits (T10).
// NOTE diamond-specific (like the effectiveness class); revisit at the
// NG swap (S 1.3035, CLa from the NG polar).
#define DIAMOND_SREF_M2 1.404f
#define DIAMOND_CLA_RAD 4.1f
#define DIAMOND_FLARE_CAP_N 40.f
#define DIAMOND_UNLOAD_TARGET_N 70.f
#define BACKTRANS_THROTTLE_DOWNRAMP_DUR_S 0.5f
#define BACKTRANS_THROTTLE_UPRAMP_DUR_S 0.5f

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tiltrotor::parameters_update()
{
	VtolType::updateParams();
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		_vtol_mode = vtol_mode::MC_MODE;

	} else 	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_transition_start_yaw = Eulerf(Quatf(_v_att->q)).psi();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			const bool exit_backtransition_tilt_condition = _tilt_control <= (_param_vt_tilt_mc.get() + 0.01f);

			// speed exit condition: use ground if valid, otherwise airspeed
			bool exit_backtransition_speed_condition = false;

			if (_local_pos->v_xy_valid) {
				const Dcmf R_to_body(Quatf(_v_att->q).inversed());
				const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);
				exit_backtransition_speed_condition = vel(0) < _param_mpc_xy_cruise.get() ;

			} else if (PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
				exit_backtransition_speed_condition = _airspeed_validated->calibrated_airspeed_m_s < _param_mpc_xy_cruise.get() ;
			}

			const bool exit_backtransition_time_condition = _time_since_trans_start > _param_vt_b_trans_dur.get() ;

			if (exit_backtransition_tilt_condition && (exit_backtransition_speed_condition || exit_backtransition_time_condition)) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			resetTransitionStates();
			_pitch_quiet_since = 0;
			_transition_start_yaw = Eulerf(Quatf(_v_att->q)).psi();
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::TRANSITION_FRONT_P2;
					_trans_finished_ts = hrt_absolute_time();
					resetTransitionStates();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _param_vt_tilt_fw.get()) {
				_vtol_mode = vtol_mode::FW_MODE;
				_tilt_control = _param_vt_tilt_fw.get();
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

float Tiltrotor::distToGround() const
{
	// Same source ladder as VtolType::pusher_assist(): range sensor
	// when valid, else local z relative to home, else raw -z.
	if (_local_pos->dist_bottom_valid) {
		return _local_pos->dist_bottom;
	}

	const float home_z = _attc->get_home_position_z();

	if (PX4_ISFINITE(home_z)) {
		return -(_local_pos->z - home_z);
	}

	return -_local_pos->z;
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	_tilt_control = VtolType::pusher_assist() + _param_vt_tilt_mc.get();
	_mc_yaw_weight = 1.0f;

	// Wind-takeoff rotation (T7). Parked nose-into-wind the stance
	// alpha is negative: the wind presses the airframe onto its gear
	// (~50 N at 245 Pa) and the ground-frozen climb integrator grinds
	// through a marginal liftoff (240+ s, 15 m/s staged runs). The
	// pusher-assist clamp cannot cover this: EN=1 is disabled by the
	// landing_or_descending clause during climb-rate-controlled
	// phases and EN=2 is altitude-gated above the ground. While
	// LANDED with real airflow, hold a nose-up attitude setpoint
	// (rotate on the gear -> the wing lifts instead of pressing) and
	// crack the diverters so drag is balanced from the first airborne
	// instant. VT_DIA_ROT_P = rotation pitch in deg, 0 disables
	// (calm boot state; the runner stages it for wind runs).
	// Altitude-gated, NOT land-detector-gated: at pinned full thrust
	// the detector de-asserts landed (its low-thrust test) while the
	// vehicle is physically on its gear — gating on ->landed left the
	// rotation dead exactly when it was needed (tilt 0.00 through the
	// 575 s pin).
	// Gate at MPC_LAND_ALT1 — the SAME altitude where the EN=2
	// pusher takes over — so the two regimes hand off with no gap.
	// The earlier 1 m gate left a 1-2 m band with neither rotation
	// nor pusher: at 245 Pa the lean downforce owns that band and
	// the vehicle limit-cycled 0-2 m for the whole 580 s budget.
	if (_param_vt_dia_rot_p.get() > FLT_EPSILON
	    && distToGround() < _param_mpc_land_alt1.get()
	    && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
		const float cas = _airspeed_validated->calibrated_airspeed_m_s;
		const float qbar = 0.5f * 1.225f * cas * cas;

		if (qbar > 20.f) {
			const float pitch_rot = math::radians(_param_vt_dia_rot_p.get());
			const Eulerf euler_sp(Quatf(_v_att_sp->q_d));

			if (euler_sp.theta() < pitch_rot) {
				const Quatf q_rot(Eulerf(euler_sp.phi(), pitch_rot, euler_sp.psi()));
				q_rot.copyTo(_v_att_sp->q_d);
				_v_att_sp->pitch_body = pitch_rot;
			}

			_tilt_control = math::max(_tilt_control, 0.15f);
		}
	}

	// Descent wing-unload bias (T10), the mirror of the takeoff
	// rotation. Descending through strong wind in MC the wing carries
	// the vehicle (245 Pa at 20 m/s): the vertical loop cannot push
	// DOWN, throttles to near zero chasing the descent and the
	// vehicle finally departs and free-falls (32.8 m drop, ulog
	// 04_23_48). Bias the attitude setpoint nose-DOWN so the wing
	// unloads (CL(-4 deg) ~ -60 N at 245 Pa) and the fans get their
	// descent margin back. Runs AFTER pusher_assist so it overrides
	// the VT_LND_PITCH_MIN clamp during the descent; the pusher tilt
	// keeps carrying the drag. VT_DIA_DST_P = nose-down bias in deg,
	// 0 disables (calm boot state; the runner stages it for wind).
	if (_param_vt_dia_dst_p.get() > FLT_EPSILON
	    && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
		const bool descending = _v_control_mode->flag_control_auto_enabled
					&& ((_attc->get_pos_sp_triplet()->current.valid
					     && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND)
					    || (_v_control_mode->flag_control_climb_rate_enabled
						&& !_v_control_mode->flag_control_altitude_enabled));
		const float cas_dst = _airspeed_validated->calibrated_airspeed_m_s;
		const float qbar_dst = 0.5f * 1.225f * cas_dst * cas_dst;

		if (descending && qbar_dst > 20.f && distToGround() > _param_mpc_land_alt1.get()) {
			// Force-scaled, not fixed-angle: -4 deg at 245 Pa is
			// -102 N of downforce (beyond the fan margin). Target
			// DIAMOND_UNLOAD_TARGET_N of wing unload; the param
			// caps the angle at low qbar.
			const float pitch_bias = -math::min(
					math::radians(_param_vt_dia_dst_p.get()),
					DIAMOND_UNLOAD_TARGET_N / (qbar_dst * DIAMOND_SREF_M2 * DIAMOND_CLA_RAD));
			const Eulerf euler_dst(Quatf(_v_att_sp->q_d));

			if (euler_dst.theta() > pitch_bias) {
				const Quatf q_dst(Eulerf(euler_dst.phi(), pitch_bias, euler_dst.psi()));
				q_dst.copyTo(_v_att_sp->q_d);
				_v_att_sp->pitch_body = pitch_bias;
			}
		}
	}
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// this is needed to avoid a race condition when entering backtransition when the mc rate controller publishes
	// a zero throttle value
	_v_att_sp->thrust_body[2] = -_v_att_sp->thrust_body[0];

	// make sure motors are tilted forward
	_tilt_control = _param_vt_tilt_fw.get();
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we get attitude setpoint from a multirotor flighttask if altitude is controlled.
	// in any other case the fixed wing attitude controller publishes attitude setpoint from manual stick input.
	if (_v_control_mode->flag_control_climb_rate_enabled) {
		// we need a recent incoming (virtual) MC attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

		// Diamond OFFBOARD transitions: roll_body stays MC (the memcpy
		// above already took it). Upstream blends the FW virtual roll
		// here -- but with no FW guidance in OFFBOARD, fw_att_control
		// publishes FRESH garbage setpoints: blending its roll banked
		// the vehicle 26 deg and flew it into the ground (run 21; the
		// runs-18-20 'yaw spiral' was the same bank read through the
		// roll-weight fade). Upstream also returned early on a stale
		// FW setpoint -- dropped for the same reason.

		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_thrust_transition = _fw_virtual_att_sp->thrust_body[0];
	}


	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition all rotors are enabled

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _param_vt_tilt_trans.get()) {
			const float ramped_up_tilt = _param_vt_tilt_mc.get() +
						     fabsf(_param_vt_tilt_trans.get() - _param_vt_tilt_mc.get()) *
						     _time_since_trans_start / _param_vt_f_trans_dur.get() ;

			// only allow increasing tilt (tilt in hover can already be non-zero)
			_tilt_control = math::max(_tilt_control, ramped_up_tilt);
		}

		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		// Diamond: MC keeps FULL roll authority through all of P1.
		// Upstream blends roll toward the FW controller (by airspeed,
		// or a timer without one) -- but the diamond P1 locks the
		// matrix-1 elevons (no FW guidance in OFFBOARD), so any fade
		// hands roll to a DEAD surface: observed as an uncontrolled
		// bank -> 32 m-radius coordinated circle read as a yaw spiral
		// (runs 18-20: yaw_rate = V/R at constant radius; the roll
		// fade began exactly at VT_ARSP_BLEND = 8 m/s). Roll and yaw
		// hand over only at P2/FW, past the groundspeed gate.
		// (_mc_roll_weight stays at the 1.0 set above.)

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

		// Diamond: PIN the yaw setpoint to the transition-start heading.
		// The incoming MC virtual attitude setpoint rotates its yaw with
		// the velocity vector during the transition flight task, and a
		// body-fixed thrust chasing a rotating nose is a divergent
		// spiral (run 18: yaw 26 -> 91 deg at ~11 deg/s while world
		// velocity kept growing -- the 'fore-aft reversal' was pure
		// frame rotation). q_d is rebuilt from the euler fields below.
		// yaw_sp_move_rate must ALSO be zeroed: the copied MC virtual
		// setpoint carries the rotating source's yaw-rate feedforward
		// (~0.19 rad/s = the observed 11 deg/s), and mc_att_control's
		// rate FF spins the vehicle regardless of the pinned q_d
		// (run 19: identical spiral with yaw_body pinned alone).
		_v_att_sp->yaw_body = _transition_start_yaw;
		_v_att_sp->yaw_sp_move_rate = 0.f;

	} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = math::constrain(_param_vt_tilt_trans.get() +
						fabsf(_param_vt_tilt_fw.get() - _param_vt_tilt_trans.get()) * _time_since_trans_start /
						_param_vt_trans_p2_dur.get(), _param_vt_tilt_trans.get(), _param_vt_tilt_fw.get());

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

		// this line is needed such that the fw rate controller is initialized with the current throttle value.
		// if this is not then then there is race condition where the fw rate controller still publishes a zero sample throttle after transition
		_v_att_sp->thrust_body[0] = _thrust_transition;

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// tilt rotors back once motors are idle
		if (_time_since_trans_start > BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {

			float progress = (_time_since_trans_start - BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) / math::max(_param_vt_bt_tilt_dur.get(),
					 0.1f);
			progress = math::constrain(progress, 0.0f, 1.0f);
			_tilt_control = moveLinear(_param_vt_tilt_fw.get(), _param_vt_tilt_mc.get(), progress);
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		if (_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
		}

		if (_time_since_trans_start < BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {
			// blend throttle from FW value to 0
			_mc_throttle_weight = 1.0f;
			const float target_throttle = 0.0f;
			const float progress = _time_since_trans_start / BACKTRANS_THROTTLE_DOWNRAMP_DUR_S;
			blendThrottleDuringBacktransition(progress, target_throttle);

		} else if (_time_since_trans_start < timeUntilMotorsAreUp()) {
			// while we quickly rotate back the motors keep throttle at idle

			// turn on all MC motors
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			float progress = (_time_since_trans_start - timeUntilMotorsAreUp()) / BACKTRANS_THROTTLE_UPRAMP_DUR_S;
			progress = math::constrain(progress, 0.0f, 1.0f);
			_mc_throttle_weight = moveLinear(0.0f, 1.0f, progress);
		}
	}


	_v_att_sp->thrust_body[2] = -_thrust_transition;

	const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tiltrotor::fill_actuator_outputs()
{

	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Multirotor output
	_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
	_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
	_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;

	// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
	// pass the vector magnitude and collective tilt separately. MC also needs collective thrust on z.
	// Passing 3D thrust plus tilt is not feasible as they
	// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
	// by the allocator directly.
	float collective_thrust_normalized_setpoint = 0.f;

	if (_vtol_mode == vtol_mode::FW_MODE) {

		collective_thrust_normalized_setpoint = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_y.get() ;
		}

	} else {
		collective_thrust_normalized_setpoint = -_vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;
	}

	// Fixed wing output.
	// Diamond: matrix-1 elevons stay LOCKED during P1. fw_att_control
	// runs on garbage FW attitude setpoints in OFFBOARD (no FW
	// guidance) and upstream forwards its torque UNWEIGHTED here in
	// every non-MC mode -- dual control against the MC loops with the
	// elevons at a qbar where they have real authority. Observed as a
	// deterministic yaw-windup / fore-aft oscillation on the P1
	// plateau (runs 13-17). P2 and FW keep stock behavior: they only
	// happen past the groundspeed gate where FW control is legitimate.
	// (P2 included: at 16 m/s qbar ~165 Pa the elevons have real
	// authority and the garbage FW roll setpoint snapped the vehicle
	// past 45 deg within 0.8 s of the P2 unlock -- run 35.)
	if ((!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE)
	    && _vtol_mode != vtol_mode::TRANSITION_FRONT_P1
	    && _vtol_mode != vtol_mode::TRANSITION_FRONT_P2
	    && _vtol_mode != vtol_mode::TRANSITION_BACK) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];

	} else if (_vtol_mode == vtol_mode::MC_MODE
		   && _param_vt_dia_elev_qbar.get() > FLT_EPSILON
		   && distToGround() > _param_mpc_land_alt1.get()
		   && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
		// Grounded gate: on the ground in high wind the MC attitude
		// demands rail the elevons against the fixed gear stance; at
		// 138 Pa the surface downforce (~tens of N) exceeds the static
		// thrust margin and pins the takeoff (T7 15 m/s runs 1/3,
		// proven by the elevon-off A/B run 4 which lifted cleanly).
		// Altitude gate, NOT the land detector: at pinned full
		// thrust the detector de-asserts landed (low-thrust test)
		// while the vehicle is physically on its gear — the !landed
		// form left the elevons live and the pin stochastic.
		// MC_MODE ONLY: the transition phases run the tuned fw_att
		// hybrid loop with elevons locked (T3-2) — injecting a second
		// MC-driven elevon path there double-acts the surfaces at
		// full blend (~220 Pa at the P2 exit) and re-broke the calm
		// transition (T6 L3b: pitch impulse before the wind even
		// switched on).
		// Wind-hover elevon assist (T6, 20 m/s campaign). In the
		// 5-10 m/s band the fans alone cannot trim the wing's wind
		// moment (7 m/s headwind climb death), while above ~15 m/s
		// the surfaces have full authority — hover in high wind IS
		// aerodynamic flight at zero groundspeed. Blend the MC
		// roll/pitch torque demands onto the elevons scaled by
		// measured qbar so the surfaces contribute exactly as much
		// as the airflow lets them: zero in calm air (bitwise-
		// identical hover regression), full at the parameterized
		// qbar. Yaw stays on the louver/fan path (no rudder).
		// Pass the FULL MC demand: the surfaces' physical authority
		// already scales with qbar (the aerodynamics IS the blend).
		// Scaling the demand too double-attenuates — at 8.7 m/s the
		// first implementation delivered ~blend^2 ~ 1 Nm and the
		// climb still pitched over (T6 hover-8.7 first attempt). The
		// qbar param now acts only as an enable threshold: below
		// ~2 m/s airspeed the deflections are pointless servo wear.
		const float cas = _airspeed_validated->calibrated_airspeed_m_s;
		const float qbar = 0.5f * 1.225f * cas * cas;

		if (qbar > 0.01f * _param_vt_dia_elev_qbar.get()) {
			// Authority cap: the delivered surface torque grows
			// linearly with qbar, so the MC loop gain does too —
			// validated to ~90 Pa (12 m/s), roll-led divergence at
			// 138 Pa (15 m/s, T7 run 2). Above VT_DIA_ELEV_QC the
			// demand is scaled by qc/qbar, holding the delivered
			// authority at the validated level. 0 disables the cap.
			const float q_cap = _param_vt_dia_elev_qcap.get();
			const float gain = (q_cap > FLT_EPSILON && qbar > q_cap) ? (q_cap / qbar) : 1.f;
			_torque_setpoint_1->xyz[0] = gain * _vehicle_torque_setpoint_virtual_mc->xyz[0];
			_torque_setpoint_1->xyz[1] = gain * _vehicle_torque_setpoint_virtual_mc->xyz[1];
		}
	}

	// Diamond transition attitude hold, applied directly at the torque
	// output (the PX4-native port of the T1 level-hold architecture).
	// Upstream v1.15 gates mc_att_control OFF during tiltrotor
	// transitions (rates stale; runs 18-23) and re-enabling it feeds a
	// rotating upstream yaw setpoint (run 25) -- so P1/P2 attitude is
	// held here with P-D laws on the hover-validated loop-shaped gains
	// (att P 1.89/1.89/0.9, rate K 3.60/2.68/4.03), bypassing the
	// attitude/rate modules. Roll and pitch hold LEVEL (the plateau
	// accelerates on diverted Fx, not tilt); yaw holds the
	// transition-start heading. Without the pitch hold the P2 diverter
	// ramp's trim step ran pitch past 30 deg in 0.6 s (run 29).
	// SYSTEM-ID PROBE (temporary): in MC hover, log the would-be
	// roll/pitch P-D law outputs next to the live mc controller's
	// virtual torques once per second. Signs/scales must match before
	// the law may own P2 attitude (run 30 lesson).
	if (_vtol_mode == vtol_mode::MC_MODE) {
		static hrt_abstime last_probe = 0;

		if (hrt_elapsed_time(&last_probe) > 1_s) {
			last_probe = hrt_absolute_time();

			vehicle_angular_velocity_s ang_vel{};
			_angular_velocity_sub.copy(&ang_vel);
			const Eulerf attitude(Quatf(_v_att->q));

			const float roll_law = 3.60f * (math::constrain(1.89f * (0.f - attitude.phi()), -2.f, 2.f) - ang_vel.xyz[0]);
			const float pitch_law = 2.68f * (math::constrain(1.89f * (0.f - attitude.theta()), -2.f, 2.f) - ang_vel.xyz[1]);

			PX4_INFO("SYSID roll: law=%.4f mc=%.4f | pitch: law=%.4f mc=%.4f | phi=%.3f theta=%.3f",
				 (double)roll_law, (double)_vehicle_torque_setpoint_virtual_mc->xyz[0],
				 (double)pitch_law, (double)_vehicle_torque_setpoint_virtual_mc->xyz[1],
				 (double)attitude.phi(), (double)attitude.theta());
		}
	}

	// NOTE: a roll+pitch torque-level hold was tried here (run 30) and
	// slammed the allocator within 2 s of the tilt ramp -- the law's
	// sign/scale against vehicle_torque_setpoint conventions is NOT yet
	// verified (system-ID probe pending). Yaw-only is flight-proven:
	// run 29 held the plateau dead straight to a real 15.9 m/s P2.
	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1
	    || _vtol_mode == vtol_mode::TRANSITION_FRONT_P2
	    || _vtol_mode == vtol_mode::TRANSITION_BACK) {
		vehicle_angular_velocity_s ang_vel{};
		_angular_velocity_sub.copy(&ang_vel);

		const Eulerf attitude(Quatf(_v_att->q));

		// Yaw DAMPER, not heading hold. The measured plant is
		// weathervane-stable (Cn_beta +0.061): with residual crab
		// (CY_beta gives tau ~ 22 s) the weathervane torque grows
		// with qbar and a fixed-heading pin escalates into an
		// arm-wrestle that saturates both yaw effectors and collapses
		// at ~10 m/s (T5 run 33: louver + fan differentials ramped
		// monotonically, then r blew up). Rate-only damping lets the
		// nose align with the airflow (the mission leg is
		// heading-aligned, so the drift is a few degrees) while the
		// louver/fans damp the lightly-damped weathervane pendulum.
		// _transition_start_yaw is still captured for the attitude-SP
		// path (P1 upstream blending), which tolerates the drift.
		const float yaw_torque = 4.03f * (0.f - ang_vel.xyz[2]);
		_torque_setpoint_0->xyz[2] = math::constrain(yaw_torque, -1.f, 1.f);
	}

	// publish tiltrotor extra controls
	tiltrotor_extra_controls_s tiltrotor_extra_controls = {};
	tiltrotor_extra_controls.collective_tilt_normalized_setpoint = _tilt_control;
	tiltrotor_extra_controls.collective_thrust_normalized_setpoint = collective_thrust_normalized_setpoint;
	tiltrotor_extra_controls.timestamp = hrt_absolute_time();
	_tiltrotor_extra_controls_pub.publish(tiltrotor_extra_controls);
}

void Tiltrotor::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];

	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _thrust_transition;
}

void Tiltrotor::blendThrottleDuringBacktransition(float scale, float target_throttle)
{
	_thrust_transition = scale * target_throttle + (1.0f - scale) * _last_thr_in_fw_mode;
}

float Tiltrotor::timeUntilMotorsAreUp()
{
	return BACKTRANS_THROTTLE_DOWNRAMP_DUR_S + _param_vt_bt_tilt_dur.get();
}

float Tiltrotor::moveLinear(float start, float stop, float progress)
{
	return start + progress * (stop - start);
}

bool Tiltrotor::isFrontTransitionCompletedBase()
{
	// Diamond speed condition. The base implementation falls back to an
	// OPEN-LOOP TIMER when airspeed is invalid -- and without a pitot,
	// ASPD_PRIMARY=0 (groundspeed-minus-wind) stays NaN in MC flight
	// because the EKF wind estimate never initializes before FW flight.
	// Observed: P2 advanced at 1.2 m/s on the timer and the vehicle
	// stalled at FW entry. Use the EKF groundspeed as the speed source
	// when airspeed is unavailable; never advance on time alone (the
	// VT_TRANS_TIMEOUT quad-chute remains the safety net).
	const bool minimum_trans_time_elapsed =
		_time_since_trans_start > getMinimumFrontTransitionTime();

	float speed = NAN;

	if (_param_fw_use_airspd.get()
	    && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
		speed = _airspeed_validated->calibrated_airspeed_m_s;

	} else if (_local_pos->v_xy_valid) {
		speed = sqrtf(_local_pos->vx * _local_pos->vx + _local_pos->vy * _local_pos->vy);
	}

	const bool speed_condition = PX4_ISFINITE(speed) && speed >= getTransitionAirspeed();

	return minimum_trans_time_elapsed && speed_condition
	       && _tilt_control >= _param_vt_tilt_trans.get()
	       && isPitchQuiet();
}

bool Tiltrotor::isPitchQuiet()
{
	const float quiet_window_s = _param_vt_dia_pq_t.get();

	if (quiet_window_s <= 0.f) {
		return true; // gate disabled (default): stock tiltrotor behavior
	}

	const Eulerf attitude(Quatf(_v_att->q));

	vehicle_angular_velocity_s angular_velocity{};
	_angular_velocity_sub.copy(&angular_velocity);

	const bool quiet = fabsf(attitude.theta()) < math::radians(_param_vt_dia_pq_ang.get())
			   && fabsf(angular_velocity.xyz[1]) < math::radians(_param_vt_dia_pq_rate.get());

	if (!quiet) {
		_pitch_quiet_since = 0;
		return false;
	}

	if (_pitch_quiet_since == 0) {
		_pitch_quiet_since = hrt_absolute_time();
	}

	return hrt_elapsed_time(&_pitch_quiet_since) > quiet_window_s * 1_s;
}

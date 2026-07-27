/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tiltrotor_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 */

/**
 * Normalized tilt in Hover
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_MC, 0.0f);

/**
 * Normalized tilt in transition to FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_TRANS, 0.4f);

/**
 * Normalized tilt in FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_FW, 1.0f);

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it takes to tilt form VT_TILT_TRANS to VT_TILT_FW.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);

/**
 * Duration motor tilt up in backtransition
 *
 * Time in seconds it takes to tilt form VT_TILT_FW to VT_TILT_MC.
 *
 * @unit s
 * @min 0.1
 * @max 10
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BT_TILT_DUR, 1.f);

/**
 * Diamond pitch-quiet gate: pitch angle threshold
 *
 * Front transition P1 holds the VT_TILT_TRANS plateau until |pitch|
 * stays below this angle (and |pitch rate| below VT_DIA_PQ_RATE) for
 * VT_DIA_PQ_T seconds. Only active when VT_DIA_PQ_T > 0.
 *
 * @unit deg
 * @min 1.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_PQ_ANG, 5.f);

/**
 * Diamond pitch-quiet gate: pitch rate threshold
 *
 * See VT_DIA_PQ_ANG. Only active when VT_DIA_PQ_T > 0.
 *
 * @unit deg/s
 * @min 1.0
 * @max 90.0
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_PQ_RATE, 15.f);

/**
 * Diamond pitch-quiet gate: sustain window
 *
 * Seconds that pitch angle AND rate must stay below their thresholds
 * before the front transition advances past the VT_TILT_TRANS plateau.
 * 0 disables the gate (stock tiltrotor behavior).
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_PQ_T, 0.f);

/**
 * Diamond wind-hover elevon assist: full-authority dynamic pressure
 *
 * In MC mode and transitions, blend the MC roll/pitch torque demands
 * onto the elevons scaled by measured qbar / this value (clamped 0-1).
 * 0 disables the assist entirely (exact legacy hover behavior).
 * The 5-10 m/s wind band is where fans alone cannot trim the wing's
 * wind moment; ~200 Pa (18 m/s) is the natural full-authority point.
 *
 * @min 0.0
 * @max 1000.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_ELEV_QB, 0.0f);

/**
 * Diamond wind-hover elevon assist authority cap (dynamic pressure)
 *
 * Above this qbar the blended MC torque demand is scaled by
 * VT_DIA_ELEV_QC / qbar, holding the delivered surface authority
 * (and thus the added MC loop gain) constant at the level validated
 * in flight. 0 disables the cap (legacy linear growth).
 *
 * @min 0.0
 * @max 1000.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_ELEV_QC, 0.0f);

/**
 * Diamond wind-takeoff rotation pitch
 *
 * While landed in MC mode with meaningful airflow (qbar > 20 Pa) the
 * attitude setpoint is held at this nose-up pitch and the diverters
 * are cracked open: the vehicle rotates on its gear and the wing
 * lifts it off instead of the wind pressing it down. 0 disables
 * (calm boot state; the runner stages it for wind runs).
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_ROT_P, 0.0f);

/**
 * Diamond front-transition P1 tilt floor
 *
 * Floors the diverter tilt from the first TRANSITION_FRONT_P1 cycle
 * so the forward fans push nose-ward immediately instead of ramping
 * from zero. The mission handover can seed a backward drift at P1
 * entry (a ~1 s southward trajectory setpoint, then NaN); with no
 * forward force for the first ramp seconds the drift grows and the
 * reversed-flow weathervane (swept wing, fins ahead of the CG in
 * backward flow) departs in yaw at qbar ~30 Pa -- the calm climb
 * yaw-divergence class (~1/8). 0 disables (stock ramp behavior).
 *
 * @unit norm
 * @min 0.0
 * @max 0.3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_P1T_FLR, 0.0f);

/**
 * Diamond BT->MC pitch handover slew rate
 *
 * The back-transition exit steps the pitch setpoint from the flare
 * hold (+12..+17 deg) to the MC position-control demand (~-2 deg) in
 * one cycle; at the near-zero qbar of a calm BT exit the attitude
 * loop chases the 14-19 deg step with q swinging past +-1.5 rad/s
 * (the BT-handover transient class). Anchor the published pitch SP
 * at the captured handover attitude and slew it toward the MC demand
 * at this rate; release on convergence. Same step-SP-forbidden
 * principle as the descent-unload slew (T10). 0 disables.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_BT2MC_R, 0.0f);

/**
 * Diamond wind-descent wing-unload pitch bias
 *
 * While descending (LAND or climb-rate-only mode) in meaningful
 * airflow above MPC_LAND_ALT1, hold the attitude setpoint at this
 * nose-DOWN pitch: the wing unloads and the fans regain descent
 * authority (in strong wind the vertical loop otherwise throttles
 * to zero chasing a descent the wing refuses, then departs).
 * 0 disables (calm boot state; the runner stages it for wind runs).
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DIA_DST_P, 0.0f);

/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * Closed-loop unit tests for TECSControl.
 *
 * A minimal aircraft model is closed-loop coupled to the TECS controller with
 * perfect state knowledge and no noise. The model is deliberately chosen to be
 * consistent with the energy model used by TECS internally, so that we can find
 * correctness errors in the controller under zero model mismatch. Any air
 * density / weight corrections that would come from the FW performance model
 * are ignored.
 *
 * Physical model
 * --------------
 *
 *   States:  V (TAS), h (altitude)
 *   Inputs:  throttle, gamma (flight path angle) = pitch angle (zero AoA, zero wind)
 *
 *   d(STE)/dt = piecewise linear in throttle, matching TECS's above/below-trim slopes
 *   h'        = V * gamma
 *   V'        = (STE_rate - g * h') / V     [remainder goes to kinetic energy]
 *
 */

#include <gtest/gtest.h>
#include "TECS.hpp"
#include <lib/geo/geo.h>
#include <px4_platform_common/param.h>

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{

static constexpr float DT = 0.02f;  // Simulation time step [s]

// --- Vehicle constants --------------------------------------------------------

static constexpr float ALT_INIT = 300.f;  ///< initial altitude [m]

/// Compile-time default of a firmware parameter.
constexpr float paramDefault(px4::params p)
{
	return px4::parameters[static_cast<size_t>(p)].val.f;
}

/// Firmware parameter defaults, mapped the way FwLateralLongitudinalControl feeds them into TECS.
constexpr TECSControl::Param defaultParams()
{
	return {
		.max_sink_rate             = paramDefault(px4::params::FW_T_SINK_MAX),
		.min_sink_rate             = paramDefault(px4::params::FW_T_SINK_MIN),
		.max_climb_rate            = paramDefault(px4::params::FW_T_CLMB_MAX),
		.vert_accel_limit          = paramDefault(px4::params::FW_T_VERT_ACC),
		.equivalent_airspeed_trim  = paramDefault(px4::params::FW_AIRSPD_TRIM),
		.tas_min                   = paramDefault(px4::params::FW_AIRSPD_MIN),
		.tas_max                   = paramDefault(px4::params::FW_AIRSPD_MAX),
		.pitch_max                 = math::radians(paramDefault(px4::params::FW_P_LIM_MAX)),
		.pitch_min                 = math::radians(paramDefault(px4::params::FW_P_LIM_MIN)),
		.throttle_trim             = paramDefault(px4::params::FW_THR_TRIM),
		.throttle_max              = paramDefault(px4::params::FW_THR_MAX),
		.throttle_min              = paramDefault(px4::params::FW_THR_MIN),
		.altitude_error_gain       = 1.f / paramDefault(px4::params::FW_T_ALT_TC),
		.altitude_setpoint_gain_ff = paramDefault(px4::params::FW_T_HRATE_FF),
		.tas_error_percentage      = 0.15f,  // = hardcoded constant in TECS.hpp
		.airspeed_error_gain       = 1.f / paramDefault(px4::params::FW_T_TAS_TC),
		.ste_rate_time_const       = paramDefault(px4::params::FW_T_STE_R_TC),
		.seb_rate_ff               = paramDefault(px4::params::FW_T_SEB_R_FF),
		.pitch_speed_weight        = paramDefault(px4::params::FW_T_SPDWEIGHT),
		.integrator_gain_pitch     = paramDefault(px4::params::FW_T_I_GAIN_PIT),
		.pitch_damping_gain        = paramDefault(px4::params::FW_T_PTCH_DAMP),
		.integrator_gain_throttle  = paramDefault(px4::params::FW_T_THR_INTEG),
		.throttle_damping_gain     = paramDefault(px4::params::FW_T_THR_DAMPING),
		.throttle_slewrate         = paramDefault(px4::params::FW_THR_SLEW_MAX),
		.load_factor_correction    = paramDefault(px4::params::FW_T_RLL2THR),
		.load_factor               = 1.f,
		.fast_descend              = 0.f,
	};
}

// --- Testing constants --------------------------------------------------------

// Maximum airspeed error while changing altitude with
//  - No model mismatch
//  - Instant tracking of pitch and throttle, no inner loops
//  - Constant airspeed setpoint
// Does not need to equal DT, but if the only source of error is euler discretisation, then
// the error should scale linearly with DT (first order method), and then 1.0 can be
// considered to have units of m/s / s.
static constexpr float MAX_V_TRACKING_ERR_PER_DT = 1.0; // m/s / s
static constexpr float MAX_V_TRACKING_ERR = MAX_V_TRACKING_ERR_PER_DT * DT;

/// Bound on the tail of the designed first-order airspeed recovery response after a
/// disturbance, and on the pitch-loop-only airspeed convergence in fast descend.
static constexpr float MAX_V_RECOVERY_ERR = 0.1f; ///< [m/s]

struct AircraftState {
	float V{0.f};
	float h{ALT_INIT};
};

float ste_rate_modelled(const TECSControl::Param &p, float throttle)
{
	// Mirrors the formula in _calcThrottleControlOutput
	const float ste_rate_max     = p.max_climb_rate * CONSTANTS_ONE_G;
	const float ste_rate_min_abs = p.min_sink_rate  * CONSTANTS_ONE_G;
	return (throttle >= p.throttle_trim)
	       ? (throttle - p.throttle_trim) * ste_rate_max     / (p.throttle_max  - p.throttle_trim)
	       : (throttle - p.throttle_trim) * ste_rate_min_abs / (p.throttle_trim - p.throttle_min);
}

AircraftState aircraft_step(const TECSControl::Param &p, float dt, float throttle, float pitch_cmd, AircraftState s,
			    float pitch_offset = 0.f, float thrust_scale = 1.f)
{
	// thrust_scale scales the energy rate the propulsion adds over minimum throttle, the drag energy rate at
	// minimum throttle is always incurred: 1 is the modelled plant, below 1 a heavy aircraft or degraded
	// propulsion (trim throttle rises, the climb rate at max throttle drops), 0 a loss of propulsion.
	const float ste_rate_min_throttle = ste_rate_modelled(p, p.throttle_min);
	const float ste_rate = ste_rate_min_throttle + thrust_scale * (ste_rate_modelled(p, throttle) - ste_rate_min_throttle);

	// pitch_offset models a pitch-to-flight-path offset the controller does not know
	// about (trim AoA not captured by FW_PSP_OFF, attitude rigging error): the plant
	// flies level only at pitch_cmd == pitch_offset.
	const float h_dot    = s.V * (pitch_cmd - pitch_offset); // Only correct in a linearised sense (close to level)
	const float spe_rate = CONSTANTS_ONE_G * h_dot;
	const float ske_rate = ste_rate - spe_rate;
	const float V_dot    = (s.V > 0.1f) ? ske_rate / s.V : 0.f; // If V < 0.1 we have long stalled...
	return {s.V + dt * V_dot, s.h + dt * h_dot};
}

}

// --- Test fixture -------------------------------------------------------------

class TECSClosedLoopTest : public ::testing::Test
{
protected:
	TECSClosedLoopTest() { _alt_ref_model.initialize({ALT_INIT, 0.f}); }

	float _t{0.f}; /// Simulation time [s]

	TECSControl::Param _params{defaultParams()}; ///< first: the plant and the setpoints derive from it
	AircraftState      _state{_params.equivalent_airspeed_trim, ALT_INIT};
	TECSControl        _tecs{};
	TECSControl::Flag  _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};
	TECSAltitudeReferenceModel _alt_ref_model{};

	float _V_sp         {_params.equivalent_airspeed_trim};
	float _alt_sp       {ALT_INIT};
	float _alt_ref      {ALT_INIT};   ///< smoothed altitude reference fed to TECS
	float _alt_ref_rate {0.f};
	float _V_prev       {_params.equivalent_airspeed_trim};     ///< for TAS-rate finite-difference
	float _pitch_prev   {0.f};        ///< pitch command from previous step, for altitude_rate input

	// Options to re-introduce some basic model mismatch
	float _plant_pitch_offset {0.f};  ///< pitch-to-flight-path offset of the plant, unknown to TECS
	float _plant_thrust_scale {1.f};  ///< fraction of the modelled energy rate over minimum throttle the plant delivers

	bool  _tecs_initialised {false};
	FILE *_output_csv_file {nullptr};

	void TearDown() override
	{
		if (_output_csv_file) {
			std::fclose(_output_csv_file);
			_output_csv_file = nullptr;
		}
	}

	struct SimulationStats {
		float max_airspeed_error{0.f};
		float max_altitude_error{0.f};
		float max_throttle_integ_norm{0.f};
		float max_pitch_integ_norm{0.f};
		float max_energy_balance_rate_error{0.f};
		float max_total_energy_rate_error{0.f};
	};

	SimulationStats _sim_stats{};

	void resetSimStats() { _sim_stats = {}; }

	void updateSimStats(const AircraftState &state, const TECSControl::DebugOutput &tecs_debug)
	{
		auto track_max_norm = [](float & field, float val) { field = std::max(field, std::fabs(val)); };

		track_max_norm(_sim_stats.max_airspeed_error,            state.V - _V_sp);
		track_max_norm(_sim_stats.max_altitude_error,            state.h - _alt_sp);
		track_max_norm(_sim_stats.max_energy_balance_rate_error, tecs_debug.energy_balance_rate_sp - tecs_debug.energy_balance_rate_estimate);
		track_max_norm(_sim_stats.max_total_energy_rate_error,   tecs_debug.total_energy_rate_sp   - tecs_debug.total_energy_rate_estimate);

		track_max_norm(_sim_stats.max_throttle_integ_norm,       tecs_debug.throttle_integrator);
		track_max_norm(_sim_stats.max_pitch_integ_norm,          tecs_debug.pitch_integrator);

	}

	// Altitude reference generated by the altitude reference model TECS uses, with the rate limits of the vehicle
	// and the vertical acceleration limit of the controller.
	void updateAltRef()
	{
		const TECSAltitudeReferenceModel::Param param{
			.target_climbrate = _params.max_climb_rate,
			.target_sinkrate  = _params.max_sink_rate,
			.vert_accel_limit = _params.vert_accel_limit,
			.max_climb_rate   = _params.max_climb_rate,
			.max_sink_rate    = _params.max_sink_rate,
		};
		_alt_ref_model.update(DT, {_alt_sp, NAN}, _state.h, _state.V * (_pitch_prev - _plant_pitch_offset), param);
		_alt_ref      = _alt_ref_model.getAltitudeReference().alt;
		_alt_ref_rate = _alt_ref_model.getAltitudeReference().alt_rate;
	}

	TECSControl::Setpoint makeSetpoint() const
	{
		return {
			.altitude_reference            = {_alt_ref, _alt_ref_rate},
			.altitude_rate_setpoint_direct = NAN,   // use altitude control loop
			.tas_setpoint                  = _V_sp,
		};
	}

	TECSControl::Input makeInput() const
	{
		return {
			.altitude      = _state.h,
			.altitude_rate = _state.V * (_pitch_prev - _plant_pitch_offset),
			.tas           = _state.V,
			.tas_rate      = (_state.V - _V_prev) / DT,
		};
	}

	// Initialise TECS from the current aircraft state with zeroed integrators. run() does this on its first
	// call, so _params, the plant and the setpoints can be changed freely before. Later changes to _params
	// take effect on the next update; call this to also reset the controller state.
	void initTecs()
	{
		_tecs.initialize(makeSetpoint(), makeInput(), _params, _flag);
		_tecs_initialised = true;
	}

	void setAirspeedSetpoint(float v_sp)
	{
		_V_sp = v_sp;
	}

	void setAltitudeSetpoint(float alt_sp)
	{
		_alt_sp = alt_sp;
	}

	// Advance simulation for `duration` seconds, updating _sim_stats.
	// If TECS_TEST_LOG_DIR is set in the environment, writes a CSV to
	// $TECS_TEST_LOG_DIR/<test_name>.csv with one row per timestep.
	void run(float duration)
	{
		float t_start = _t;

		if (!_tecs_initialised) {
			initTecs();
		}

		if (!_output_csv_file) {
			// File not present yet (first run in test) - open it, and write header

			const char *log_dir = std::getenv("TECS_TEST_LOG_DIR");

			if (log_dir) {
				const auto *test_info = ::testing::UnitTest::GetInstance()->current_test_info();
				std::string path = std::string(log_dir) + "/" + test_info->name() + ".csv";
				_output_csv_file = std::fopen(path.c_str(), "w");

				if (_output_csv_file) {
					std::fprintf(_output_csv_file,
						     "t,V,h,V_sp,alt_sp,pitch_cmd,throttle,"
						     "pitch_integ,throttle_integ,"
						     "ste_rate_sp,ste_rate_est,seb_rate_sp,seb_rate_est\n");
				}
			}
		}

		while (_t < t_start + duration) {

			updateAltRef();
			_tecs.update(DT, makeSetpoint(), makeInput(), _params, _flag);

			_V_prev     = _state.V;
			_pitch_prev = _tecs.getPitchSetpoint();
			_state      = aircraft_step(_params, DT, _tecs.getThrottleSetpoint(), _pitch_prev, _state, _plant_pitch_offset,
						    _plant_thrust_scale);
			updateSimStats(_state, _tecs.getDebugOutput());

			if (_output_csv_file) {
				const auto &dbg = _tecs.getDebugOutput();
				std::fprintf(_output_csv_file, "%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
					     (double) _t,
					     (double) _state.V, (double) _state.h,
					     (double) _V_sp, (double) _alt_sp,
					     (double) _pitch_prev, (double) _tecs.getThrottleSetpoint(),
					     (double) dbg.pitch_integrator, (double) dbg.throttle_integrator,
					     (double) dbg.total_energy_rate_sp, (double) dbg.total_energy_rate_estimate,
					     (double) dbg.energy_balance_rate_sp, (double) dbg.energy_balance_rate_estimate);
			}

			_t += DT;
		}
	}
};

// --- Steady-state cruise ------------------------------------------------------
// Starting at perfect trim the feedforward is exact, integrators remain zero,
// and airspeed must not drift at all.  This exercises that the model and TECS
// are self-consistent at the operating point.

TEST_F(TECSClosedLoopTest, SteadyStateCruiseDefaultTuning)
{
	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_V_TRACKING_ERR);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_V_TRACKING_ERR);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_V_TRACKING_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_V_TRACKING_ERR);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpDefaultTuning)
{
	run(60.0f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownDefaultTuning)
{
	run(60.0f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Altitude step with high speed weight ------------------------------------
//
// Even with a speed weight != 1, the feedforward should still be consistent
// and not introduce any airspeed errors.

TEST_F(TECSClosedLoopTest, AltitudeStepUpHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowAccel)
{
	// 10x smaller vertical accel limit than default
	_params.vert_accel_limit = 0.7f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 100.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowAccel)
{
	// 10x smaller vertical accel limit than default
	_params.vert_accel_limit = 0.7f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 100.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Plant pitch-to-flight-path offset (model mismatch) -----------------------
//
// The plant flies level at pitch_cmd = 10 deg instead of 0 (e.g. incorrect FW_PSP_OFF).
// The pitch integrator should converge to this offset.

TEST_F(TECSClosedLoopTest, SteadyStateCruisePitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.f);

	// Reset stats to caputre errors after convergence
	resetSimStats();
	run(60.f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_RECOVERY_ERR);

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_LT(fabsf(_tecs.getDebugOutput().pitch_integrator - _plant_pitch_offset), math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpDefaultTuningPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_RECOVERY_ERR);

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_LT(fabsf(_tecs.getDebugOutput().pitch_integrator - _plant_pitch_offset), math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpHighSpeedWeightPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);
	_params.pitch_speed_weight = 1.8f;

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_RECOVERY_ERR);

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_LT(fabsf(_tecs.getDebugOutput().pitch_integrator - _plant_pitch_offset), math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownHighSpeedWeightPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);
	_params.pitch_speed_weight = 1.8f;

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_RECOVERY_ERR);

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_LT(fabsf(_tecs.getDebugOutput().pitch_integrator - _plant_pitch_offset), math::radians(0.1f));
}

// --- Pitch limit below pitch demand ----------------------------------------

TEST_F(TECSClosedLoopTest, AltitudeStepUpPitchLimited)
{
	// Max pitch so low that it becomes the limiting factor for the achievable climb rate
	_params.pitch_max = math::radians(5.f);
	const float flyable_climb_rate = _params.equivalent_airspeed_trim * _params.pitch_max;

	run(10.f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(15.f);

	// Mid climb: pitch at limit with correspondingly lower climb rate
	EXPECT_NEAR(_pitch_prev, _params.pitch_max, math::radians(0.5f));
	EXPECT_NEAR(_state.V * (_pitch_prev - _plant_pitch_offset), flyable_climb_rate, 0.1f);

	run(185.f);

	// Still do not allow airspeed errors
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownPitchLimited)
{
	// Min pitch so low that it becomes the limiting factor for the achievable sink rate
	_params.pitch_min = math::radians(-5.f);
	const float flyable_sink_rate = _params.equivalent_airspeed_trim * _params.pitch_min;

	run(10.f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(15.f);

	// Mid climb: pitch at limit with correspondingly lower sink rate
	EXPECT_NEAR(_pitch_prev, _params.pitch_min, math::radians(0.5f));
	EXPECT_NEAR(_state.V * (_pitch_prev - _plant_pitch_offset), flyable_sink_rate, 0.1f);

	run(185.f);

	// Still do not allow airspeed errors
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Propulsion failure --------------------------------------------------------
//
//  - Plant delivers minimum throttle regardless of command
//  - TECS must give up altitude and pitch down to not lose too much airspeed
//
// TODO: update these after https://github.com/PX4/PX4-Autopilot/pull/26859 merges

TEST_F(TECSClosedLoopTest, PropulsionFailureCruise)
{
	run(20.f);
	_plant_thrust_scale = 0.f;
	resetSimStats();
	run(120.f);

	static constexpr float V_MIN = 5.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN);
}

TEST_F(TECSClosedLoopTest, PropulsionFailureClimb)
{
	run(20.f);
	setAltitudeSetpoint(ALT_INIT + 200.f);
	run(15.f);
	_plant_thrust_scale = 0.f;
	resetSimStats();
	run(120.f);

	static constexpr float V_MIN = 5.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN);
}

TEST_F(TECSClosedLoopTest, PropulsionFailureCruiseHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(20.f);
	_plant_thrust_scale = 0.f;
	resetSimStats();
	run(120.f);

	// Same floor as with the default weight (the feedforward is the same), but the weighted pitch integrator
	// recovers the airspeed above tas_min by the end of the run.
	static constexpr float V_MIN = 6.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN);
	EXPECT_GT(_state.V, _params.tas_min);
}

TEST_F(TECSClosedLoopTest, PropulsionFailureClimbHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(20.f);
	setAltitudeSetpoint(ALT_INIT + 200.f);
	run(15.f);
	_plant_thrust_scale = 0.f;
	resetSimStats();
	run(120.f);

	static constexpr float V_MIN = 6.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN);
	EXPECT_GT(_state.V, _params.tas_min);
}

// --- Airspeed setpoint change -------------------------------------------------

// Step the airspeed setpoint with constant altitude setpoint.
// The airspeed should be followed relatively quickly, with limited altitude errors.

TEST_F(TECSClosedLoopTest, AirspeedStepUpDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim + 1.f);

	// Capture the rough shape of the airspeed step response
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.3f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.05f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.02f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.01f);

	// This is possible without sacrificing a lot of altitude
	EXPECT_LT(_sim_stats.max_altitude_error, 0.5f);
}

TEST_F(TECSClosedLoopTest, AirspeedStepDownDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim - 1.f);

	// Capture the rough shape of the airspeed step response
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.3f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.05f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.02f);
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.01f);

	// This is possible without sacrificing a lot of altitude
	EXPECT_LT(_sim_stats.max_altitude_error, 0.5f);
}

TEST_F(TECSClosedLoopTest, FastDescend)
{
	// The decision whether or not fast descend runs happens in the outer
	// TECS class, not TECSControl which we test here - so we can only test
	// whether fast descend runs correctly when enabling it, NOT whether the
	// outer logic enables it correctly.

	run(10.0f);

	_params.fast_descend = 1.0f;
	setAirspeedSetpoint(_params.tas_max);
	initTecs(); // reset the controller state on the mode switch, as the firmware does

	// Irrelevant, _params.fast_descend determines whether or not we do fast descend
	setAltitudeSetpoint(ALT_INIT - 200.0f);

	// Run until we are fully in fast descend
	run(60.f);

	// Throttle is forced to minimum during fast descend
	EXPECT_NEAR(_tecs.getThrottleSetpoint(), _params.throttle_min, 1e-3f);
	// Pitch loop drives airspeed toward tas_max
	EXPECT_NEAR(_state.V, _params.tas_max, MAX_V_RECOVERY_ERR);
}

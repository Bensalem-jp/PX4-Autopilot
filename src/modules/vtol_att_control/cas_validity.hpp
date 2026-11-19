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
 * @file cas_validity.hpp
 *
 * Opt-in CAS validity for the Diamond front-transition speed source
 * (isaac #411 Option A, PdM HC-D107 / D-125 item 5). Header only, no uORB:
 * the caller copies airspeed_validated, the raw airspeed instances and the
 * local position into Inputs. It replaces the speed selection in
 * Tiltrotor::isFrontTransitionCompletedBase(); the result feeds
 * PitchQuietGate::Inputs::speed, so it serves the PQ gate and the stock
 * path (VT_DIA_PQ_T <= 0) alike.
 *
 * Disabled (the default): selectSpeed() is the legacy selection, unchanged.
 *
 * Enabled with FW_USE_AIRSPD=1: the CAS is used only if it is usable
 * (evaluate()); otherwise the speed is NaN, which makes the gate's speed
 * condition false and resets its window. There is NO groundspeed fallback
 * in that case.
 *
 * Thresholds have NO adopted default. A value <= 0 means "unset". Enabled
 * with a needed threshold unset fails closed (Reason::ThresholdUnset): the
 * CAS is never usable, so P1 -> P2 cannot happen on speed. No threshold
 * value is hard-coded here.
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace cas_validity
{

using Timestamp = uint64_t; // microseconds, same base as hrt_abstime

// AirspeedValidated.selected_airspeed_index: 1-3 = sensor, 0 = groundspeed
// minus wind, -1 = invalid.
static constexpr int kMaxRawInstances = 3;

// Legacy selector-alive check of the original Tiltrotor code (1_s). Used on
// the disabled path only, so that path stays bit-identical.
static constexpr Timestamp kLegacyValidatedMaxAgeUs = 1000000;

enum class Reason : uint8_t {
	Usable = 0,
	ThresholdUnset,          // enabled, but a needed threshold is <= 0
	ValidatedNeverPublished, // airspeed_validated.timestamp == 0
	ValidatedStale,          // selector cycle age > max_validated_age_us
	IndexNotSensor,          // selected_airspeed_index < 1
	IndexNoRawInstance,      // selected index has no raw airspeed instance
	MeasurementInvalid,      // airspeed_sensor_measurement_valid == false
	RawNeverSampled,         // selected raw timestamp_sample == 0
	RawStale,                // selected raw sample age > max_raw_sample_age_us
	NonFinite,               // CAS or selected raw IAS not finite
	Disagreement,            // another fresh sensor differs by more than allowed
};

enum class Agreement : uint8_t {
	NotEvaluated = 0, // no other fresh, finite sensor (e.g. one pitot)
	Agree,
	Disagree,
};

enum class ConfigStatus : uint8_t {
	Disabled = 0,
	Ok,
	ThresholdUnset,   // enabled with T_V / T_R (or, > 1 pitot, disagreement) unset: P1 -> P2 impossible
	NoAirspeedSensor, // enabled, FW_USE_AIRSPD=1, no pitot: P1 -> P2 impossible
};

struct Config {
	bool enabled{false};
	// No default is adopted; <= 0 means unset.
	int32_t max_validated_age_us{0};   // T_V: now - airspeed_validated.timestamp
	int32_t max_raw_sample_age_us{0};  // T_R: now - airspeed[index-1].timestamp_sample
	float max_disagreement_m_s{0.f};   // |IAS_selected - IAS_other|; needed only with > 1 fresh sensor
};

struct RawSample {
	bool present{false};           // instance exists (subscription advertised)
	Timestamp timestamp_sample{0};
	float indicated_airspeed_m_s{NAN};
};

struct Inputs {
	Timestamp now{0};
	bool fw_use_airspd{false};

	// airspeed_validated
	Timestamp validated_timestamp{0};
	float calibrated_airspeed_m_s{NAN};
	bool measurement_valid{false};
	int8_t selected_index{-1};

	// raw airspeed instances; raw[i] is sensor index i + 1
	RawSample raw[kMaxRawInstances] {};

	// vehicle_local_position
	bool v_xy_valid{false};
	float vx{0.f};
	float vy{0.f};
};

struct Evaluation {
	Reason reason{Reason::ThresholdUnset};
	Agreement agreement{Agreement::NotEvaluated};
	bool usable() const { return reason == Reason::Usable; }
};

inline bool thresholdSet(int32_t value_us) { return value_us > 0; }

inline bool thresholdSet(float value) { return std::isfinite(value) && value > 0.f; }

/**
 * Age check, bounded on both sides by the same configured threshold: a
 * sample stamped in the future is fresh only if it is at most max_age_us
 * ahead of now (CE411-M1). Unlike PitchQuietGate::isFresh, an unbounded
 * future stamp would let one bad stamp unlock a frozen CAS indefinitely.
 * A frozen future stamp can still count as fresh for at most 2 * max_age_us
 * of wall time ([sample - T, sample + T]).
 */
inline bool withinAge(Timestamp now, Timestamp sample, int32_t max_age_us)
{
	const Timestamp max_age = static_cast<Timestamp>(max_age_us);

	if (sample >= now) {
		return sample - now <= max_age;
	}

	return now - sample <= max_age;
}

/** Legacy speed selection, verbatim logic of Tiltrotor at f53d473ecf. */
inline float legacySpeed(const Inputs &in)
{
	float speed = NAN;
	const bool airspeed_fresh = (in.now - in.validated_timestamp) <= kLegacyValidatedMaxAgeUs;

	if (in.fw_use_airspd && airspeed_fresh && std::isfinite(in.calibrated_airspeed_m_s)) {
		speed = in.calibrated_airspeed_m_s;

	} else if (in.v_xy_valid) {
		speed = sqrtf(in.vx * in.vx + in.vy * in.vy);
	}

	return speed;
}

/**
 * Cross-check the selected sensor against every other raw instance that is
 * present, sampled, fresh (T_R) and finite. A stale or non-finite other
 * instance cannot confirm or refute; it is ignored (NotEvaluated if none is
 * left). Requires a set max_disagreement_m_s once a comparison is possible.
 */
inline Reason checkDisagreement(const Inputs &in, const Config &cfg, int selected, Agreement &agreement)
{
	agreement = Agreement::NotEvaluated;
	const float selected_ias = in.raw[selected].indicated_airspeed_m_s;

	for (int i = 0; i < kMaxRawInstances; i++) {
		const RawSample &other = in.raw[i];

		if (i == selected || !other.present || other.timestamp_sample == 0
		    || !withinAge(in.now, other.timestamp_sample, cfg.max_raw_sample_age_us)
		    || !std::isfinite(other.indicated_airspeed_m_s)) {
			continue;
		}

		if (!thresholdSet(cfg.max_disagreement_m_s)) {
			return Reason::ThresholdUnset;
		}

		if (fabsf(selected_ias - other.indicated_airspeed_m_s) > cfg.max_disagreement_m_s) {
			agreement = Agreement::Disagree;
			return Reason::Disagreement;
		}

		agreement = Agreement::Agree;
	}

	return Reason::Usable;
}

/** Is the CAS usable as the transition speed? First failing check is the reason. */
inline Evaluation evaluate(const Inputs &in, const Config &cfg)
{
	Evaluation result{};

	if (!thresholdSet(cfg.max_validated_age_us) || !thresholdSet(cfg.max_raw_sample_age_us)) {
		result.reason = Reason::ThresholdUnset;
		return result;
	}

	if (in.validated_timestamp == 0) {
		result.reason = Reason::ValidatedNeverPublished;
		return result;
	}

	if (!withinAge(in.now, in.validated_timestamp, cfg.max_validated_age_us)) {
		result.reason = Reason::ValidatedStale;
		return result;
	}

	if (in.selected_index < 1) {
		result.reason = Reason::IndexNotSensor;
		return result;
	}

	const int selected = in.selected_index - 1;

	if (selected >= kMaxRawInstances || !in.raw[selected].present) {
		result.reason = Reason::IndexNoRawInstance;
		return result;
	}

	if (!in.measurement_valid) {
		result.reason = Reason::MeasurementInvalid;
		return result;
	}

	const RawSample &raw = in.raw[selected];

	if (raw.timestamp_sample == 0) {
		result.reason = Reason::RawNeverSampled;
		return result;
	}

	if (!withinAge(in.now, raw.timestamp_sample, cfg.max_raw_sample_age_us)) {
		result.reason = Reason::RawStale;
		return result;
	}

	if (!std::isfinite(in.calibrated_airspeed_m_s) || !std::isfinite(raw.indicated_airspeed_m_s)) {
		result.reason = Reason::NonFinite;
		return result;
	}

	result.reason = checkDisagreement(in, cfg, selected, result.agreement);
	return result;
}

/**
 * Speed for PitchQuietGate::Inputs::speed. Disabled or FW_USE_AIRSPD=0: the
 * legacy selection. Enabled with FW_USE_AIRSPD=1: the CAS if usable, else NaN
 * (no groundspeed fallback).
 */
inline float selectSpeed(const Inputs &in, const Config &cfg, Evaluation &evaluation)
{
	evaluation = Evaluation{};

	if (!cfg.enabled || !in.fw_use_airspd) {
		evaluation.reason = Reason::Usable;
		return legacySpeed(in);
	}

	evaluation = evaluate(in, cfg);
	return evaluation.usable() ? in.calibrated_airspeed_m_s : NAN;
}

/**
 * Configuration guard (CE addition 1, invariant I7) for a boot / pre-arm
 * check. Enabled with FW_USE_AIRSPD=1 and no physical airspeed sensor, or
 * with T_V / T_R unset, P1 -> P2 can never happen.
 */
inline ConfigStatus checkConfiguration(const Config &cfg, bool fw_use_airspd, int32_t aspd_primary,
				       int32_t sys_has_num_aspd)
{
	if (!cfg.enabled) {
		return ConfigStatus::Disabled;
	}

	if (!thresholdSet(cfg.max_validated_age_us) || !thresholdSet(cfg.max_raw_sample_age_us)) {
		return ConfigStatus::ThresholdUnset;
	}

	if (fw_use_airspd && (aspd_primary < 1 || sys_has_num_aspd < 1)) {
		return ConfigStatus::NoAirspeedSensor;
	}

	// With more than one pitot, evaluate() needs the disagreement threshold
	// as soon as a second sensor is fresh.
	if (fw_use_airspd && sys_has_num_aspd > 1 && !thresholdSet(cfg.max_disagreement_m_s)) {
		return ConfigStatus::ThresholdUnset;
	}

	return ConfigStatus::Ok;
}

/**
 * Skew/window advisory (CE411-L7). A frozen sample stamped in the future
 * stays fresh for up to 2 * T of wall time (withinAge). If 2 * T_V or
 * 2 * T_R reaches the VT_DIA_PQ_T window, such a sample alone could complete
 * the window; with the stock path (window <= 0) one fresh cycle is enough.
 * Advisory only: it does not block, it tells the operator the chosen
 * thresholds do not exclude that case. False when disabled or unset (the
 * unset case is reported by checkConfiguration()).
 */
inline bool skewWindowAdvisory(const Config &cfg, float pq_window_s)
{
	if (!cfg.enabled || !thresholdSet(cfg.max_validated_age_us) || !thresholdSet(cfg.max_raw_sample_age_us)) {
		return false;
	}

	const double window_us = (std::isfinite(pq_window_s) && pq_window_s > 0.f) ? static_cast<double>(pq_window_s) * 1e6 :
				 0.0;
	const double t_v_us = static_cast<double>(cfg.max_validated_age_us);
	const double t_r_us = static_cast<double>(cfg.max_raw_sample_age_us);
	return 2.0 * t_v_us >= window_us || 2.0 * t_r_us >= window_us;
}

/**
 * Rate-limited reason reporting: an unusable reason is reported once, at
 * most once per min_interval_us. A reason that appears inside the interval
 * is deferred, not dropped: it is reported on the first call after the
 * interval if it still holds. Usable clears the reported state. The
 * interval is a logging choice, passed in by the caller.
 */
class ReasonReporter
{
public:
	bool shouldReport(Timestamp now, Reason reason, Timestamp min_interval_us)
	{
		if (reason == Reason::Usable) {
			_reported = Reason::Usable;
			return false;
		}

		if (reason == _reported) {
			return false;
		}

		if (_last_report != 0 && now - _last_report < min_interval_us) {
			return false;
		}

		_reported = reason;
		_last_report = now;
		return true;
	}

private:
	Reason _reported{Reason::Usable};
	Timestamp _last_report{0};
};

} // namespace cas_validity

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
	RawTimestampInvalid,     // selected raw: publish time 0, sample after publish, or publish time not fresh
	OtherRawNeverSampled,    // another present instance has timestamp_sample 0 (cannot be cross-checked)
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
	// Hysteresis (CE MED-2): an unusable CAS within this time of the last
	// usable one keeps the last usable value (UnusableHold). <= 0 = unset:
	// no hold, every unusable cycle restarts the window.
	int32_t max_unusable_hold_us{0};
};

struct RawSample {
	bool present{false};           // instance exists (subscription advertised)
	Timestamp timestamp{0};        // publish time
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

		if (i == selected || !other.present) {
			continue;
		}

		// CE LOW-5, fail closed: a present instance that never stamps its
		// samples (e.g. the UAVCAN bridge) can never be cross-checked, yet the
		// pre-arm check counts it as a second pitot. Block instead of
		// silently dropping the comparison.
		if (other.timestamp_sample == 0) {
			return Reason::OtherRawNeverSampled;
		}

		if (!withinAge(in.now, other.timestamp_sample, cfg.max_raw_sample_age_us)
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

	// The sample cannot be taken after it was published, and the publish
	// time obeys the same freshness bound as the sample.
	if (raw.timestamp == 0 || raw.timestamp_sample > raw.timestamp
	    || !withinAge(in.now, raw.timestamp, cfg.max_raw_sample_age_us)) {
		result.reason = Reason::RawTimestampInvalid;
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
 * Skew/window check (CE411-L7, CE MED-1). A frozen sample stamped in the
 * future stays fresh for up to 2 * T of wall time (withinAge), and the hold
 * (UnusableHold) can bridge a further max_unusable_hold_us. If
 * 2 * max(T_V, T_R) + hold reaches the VT_DIA_PQ_T window, such a sample
 * alone could complete the window; with the stock path (window <= 0) one
 * fresh cycle is enough. False when disabled or unset (the unset case is
 * reported by checkConfiguration()).
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
	const double hold_us = thresholdSet(cfg.max_unusable_hold_us) ? static_cast<double>(cfg.max_unusable_hold_us) : 0.0;
	const double t_max_us = t_v_us > t_r_us ? t_v_us : t_r_us;
	return 2.0 * t_max_us + hold_us >= window_us;
}

/**
 * Hysteresis for the P1 speed (CE MED-2), opt-in through
 * Config::max_unusable_hold_us. An unusable CAS that follows a usable one by
 * at most the hold time yields the last usable CAS, so one bad cycle does not
 * restart the whole window; past the hold time the speed is NaN again.
 * Bridges only transient per-sample faults (bridgeable()); any other reason
 * ends the hold. Never starts without a usable sample. Unset hold: NaN, the behaviour without
 * hysteresis. reset() at every P1 entry and outside P1.
 *
 * A held value only keeps the window open: the caller must not let P1
 * complete on a held cycle (holding() true), so P1 -> P2 always happens on a
 * usable CAS.
 */
class UnusableHold
{
public:
	void reset()
	{
		_last_usable = 0;
		_last_speed = NAN;
		_holding = false;
	}

	float apply(Timestamp now, Reason reason, float speed, int32_t hold_us)
	{
		_holding = false;

		if (reason == Reason::Usable) {
			_last_usable = now;
			_last_speed = speed;
			return speed;
		}

		if (!bridgeable(reason)) {
			// Not a transient per-sample fault: no hold now, and none across it later.
			_last_usable = 0;
			return NAN;
		}

		if (!thresholdSet(hold_us) || _last_usable == 0 || now < _last_usable) {
			return NAN;
		}

		if (now - _last_usable > static_cast<Timestamp>(hold_us)) {
			return NAN;
		}

		_holding = true;
		return _last_speed;
	}

	/**
	 * Only transient per-sample faults are bridged (lead decision 2026-11-20,
	 * following CE-FORK411b): a late or invalid sample. Never a disagreement,
	 * a non-sensor index (groundspeed-minus-wind fallback) or a missing
	 * instance, an unset threshold, a never-published/never-sampled stream or
	 * an unverifiable second pitot.
	 */
	static bool bridgeable(Reason reason)
	{
		switch (reason) {
		case Reason::ValidatedStale:
		case Reason::MeasurementInvalid:
		case Reason::RawStale:
		case Reason::NonFinite:
		case Reason::RawTimestampInvalid:
			return true;

		default:
			return false;
		}
	}

	/** True if the last apply() returned a held value (the CAS itself is unusable). */
	bool holding() const { return _holding; }

private:
	Timestamp _last_usable{0};
	float _last_speed{NAN};
	bool _holding{false};
};

/** Pre-arm failures (CE MED-1 / MED-2), a bit set; 0 = may arm. */
enum ArmingFailure : uint16_t {
	kArmThresholdUnset = 1 << 0,       // T_V or T_R unset
	kArmDisagreementUnset = 1 << 1,    // more than one pitot, no disagreement threshold
	kArmWindowTooShort = 1 << 2,       // 2 * max(T) + hold >= VT_DIA_PQ_T, or stock path
	kArmNoSensor = 1 << 3,             // no raw airspeed instance, or ASPD_PRIMARY < 1
	kArmSensorUnhealthy = 1 << 4,      // evaluate() not usable now
	kArmSelectorPeriodUnknown = 1 << 5, // no airspeed_validated interval measured yet
	kArmSelectorPeriodTooLong = 1 << 6, // T_V <= the measured selector publish interval
};

struct ArmingInputs {
	Config cfg{};
	bool fw_use_airspd{false};
	float pq_window_s{0.f};
	int32_t aspd_primary{0};
	int32_t sys_has_num_aspd{0};
	int raw_instances{0};                  // advertised raw airspeed instances
	Timestamp max_selector_interval_us{0}; // 0 = not measured yet
	Evaluation sensor{};                   // evaluate() at the time of the check
};

/**
 * Pre-arm check. Only with the opt-in enabled and FW_USE_AIRSPD=1 (otherwise
 * the CAS does not decide P1 and nothing is refused).
 */
inline uint16_t checkArming(const ArmingInputs &in)
{
	if (!in.cfg.enabled || !in.fw_use_airspd) {
		return 0;
	}

	uint16_t failures = 0;
	const bool thresholds_set = thresholdSet(in.cfg.max_validated_age_us) && thresholdSet(in.cfg.max_raw_sample_age_us);

	if (!thresholds_set) {
		failures |= kArmThresholdUnset;
	}

	if ((in.raw_instances > 1 || in.sys_has_num_aspd > 1) && !thresholdSet(in.cfg.max_disagreement_m_s)) {
		failures |= kArmDisagreementUnset;
	}

	if (thresholds_set && skewWindowAdvisory(in.cfg, in.pq_window_s)) {
		failures |= kArmWindowTooShort;
	}

	if (in.raw_instances < 1 || in.aspd_primary < 1) {
		failures |= kArmNoSensor;

	} else if (thresholds_set && !in.sensor.usable()) {
		failures |= kArmSensorUnhealthy;
	}

	if (in.max_selector_interval_us == 0) {
		failures |= kArmSelectorPeriodUnknown;

	} else if (thresholds_set
		   && static_cast<Timestamp>(in.cfg.max_validated_age_us) <= in.max_selector_interval_us) {
		failures |= kArmSelectorPeriodTooLong;
	}

	return failures;
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

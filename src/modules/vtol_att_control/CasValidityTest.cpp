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
 * @file CasValidityTest.cpp
 *
 * Unit tests of cas_validity.hpp together with the real PitchQuietGate:
 * every blocking case asserts the P1 -> P2 decision, not only the reason.
 * Thresholds are 137 ms / 310 ms on purpose: no value here is adopted.
 */

#include "cas_validity.hpp"
#include "pitch_quiet_gate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

using namespace cas_validity;

namespace
{
constexpr int32_t kTv = 137000;
constexpr int32_t kTr = 310000;
constexpr float kDisagree = 1.5f;
constexpr float kCas = 15.f;
constexpr float kGroundspeed = 20.f;
constexpr Timestamp kT0 = 50000000;
constexpr Timestamp kStep = 20000;
constexpr float kWindowS = 1.f;

Config enabledConfig()
{
	Config cfg{};
	cfg.enabled = true;
	cfg.max_validated_age_us = kTv;
	cfg.max_raw_sample_age_us = kTr;
	return cfg;
}

// Enabled, FW_USE_AIRSPD=1, one fresh pitot at 15 m/s, groundspeed 20 m/s.
Inputs nominal(Timestamp now)
{
	Inputs in{};
	in.now = now;
	in.fw_use_airspd = true;
	in.validated_timestamp = now - 50000;
	in.calibrated_airspeed_m_s = kCas;
	in.measurement_valid = true;
	in.selected_index = 1;
	in.raw[0].present = true;
	in.raw[0].timestamp_sample = now - 60000;
	in.raw[0].indicated_airspeed_m_s = kCas;
	in.v_xy_valid = true;
	in.vx = kGroundspeed;
	return in;
}

struct DriveResult {
	bool passed{false};
	Timestamp first_pass{0};
	bool all_nan{true};
	bool any_usable{false};
	Reason last_reason{Reason::Usable};
};

// Drive the real gate every kStep for duration_s; everything but the speed
// source is quiet and in range.
DriveResult drive(double duration_s, const std::function<Inputs(Timestamp)> &make, const Config &cfg,
		  float pq_window_s = kWindowS)
{
	PitchQuietGate gate;
	PitchQuietGate::Params params{};
	params.min_trans_time_s = 5.f;
	params.transition_airspeed = 14.f;
	params.tilt_trans = 0.3f;
	params.pq_angle_rad = 0.1f;
	params.pq_rate_rad_s = 0.1f;
	params.pq_window_s = pq_window_s;

	DriveResult run{};
	const Timestamp end = kT0 + static_cast<Timestamp>(duration_s * 1e6);

	for (Timestamp t = kT0; t <= end; t += kStep) {
		const Inputs in = make(t);
		Evaluation evaluation{};
		const float speed = selectSpeed(in, cfg, evaluation);

		PitchQuietGate::Inputs g{};
		g.now = t;
		g.time_since_trans_start_s = 10.f;
		g.speed = speed;
		g.tilt = 0.4f;
		g.pitch_rad = 0.f;
		g.pitch_timestamp = t;
		g.pitch_rate_rad_s = 0.f;
		g.pitch_rate_timestamp = t;

		run.all_nan = run.all_nan && std::isnan(speed);
		run.any_usable = run.any_usable || evaluation.usable();
		run.last_reason = evaluation.reason;

		if (gate.update(g, params) && !run.passed) {
			run.passed = true;
			run.first_pass = t;
		}
	}

	return run;
}

void expectBlocked(const std::function<Inputs(Timestamp)> &make, const Config &cfg, Reason reason,
		   double duration_s = 2.0)
{
	const DriveResult run = drive(duration_s, make, cfg);
	EXPECT_FALSE(run.passed);
	EXPECT_TRUE(run.all_nan);
	EXPECT_EQ(run.last_reason, reason);
}

void expectPasses(const std::function<Inputs(Timestamp)> &make, const Config &cfg)
{
	const DriveResult run = drive(1.5, make, cfg);
	ASSERT_TRUE(run.passed);
	EXPECT_GT(run.first_pass - kT0, static_cast<Timestamp>(kWindowS * 1e6f));
}

uint32_t bits(float value)
{
	uint32_t out = 0;
	std::memcpy(&out, &value, sizeof(out));
	return out;
}

// Verbatim logic of the legacy selection in Tiltrotor (fork f53d473ecf).
float legacyOracle(const Inputs &in)
{
	float speed = NAN;
	const bool airspeed_fresh = (in.now - in.validated_timestamp) <= 1000000ULL;

	if (in.fw_use_airspd && airspeed_fresh && std::isfinite(in.calibrated_airspeed_m_s)) {
		speed = in.calibrated_airspeed_m_s;

	} else if (in.v_xy_valid) {
		speed = sqrtf(in.vx * in.vx + in.vy * in.vy);
	}

	return speed;
}
} // namespace

TEST(CasValidityTest, UsableCasPassesAfterTheWindow)
{
	expectPasses(nominal, enabledConfig());
}

TEST(CasValidityTest, ExactThresholdsAreInclusive)
{
	expectPasses([](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = t - kTv; in.raw[0].timestamp_sample = t - kTr; return in; },
	enabledConfig());
}

TEST(CasValidityTest, StaleValidatedBlocks)
{
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = t - kTv - 1; return in; },
	enabledConfig(), Reason::ValidatedStale);
}

TEST(CasValidityTest, StaleRawSampleBlocks)
{
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.raw[0].timestamp_sample = t - kTr - 1; return in; },
	enabledConfig(), Reason::RawStale);
}

TEST(CasValidityTest, ZeroTimestampsBlock)
{
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = 0; return in; },
	enabledConfig(), Reason::ValidatedNeverPublished);
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.raw[0].timestamp_sample = 0; return in; },
	enabledConfig(), Reason::RawNeverSampled);
}

TEST(CasValidityTest, SmallFutureSkewPassesLargeSkewBlocks)
{
	expectPasses([](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = t + 5000; in.raw[0].timestamp_sample = t + 5000; return in; },
	enabledConfig());
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = t + kTv + 1; return in; },
	enabledConfig(), Reason::ValidatedStale);
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.raw[0].timestamp_sample = t + kTr + 1; return in; },
	enabledConfig(), Reason::RawStale);
}

// CE411-M1: a frozen stream stamped ahead of hrt never completes the window.
TEST(CasValidityTest, FrozenFutureStampedRawNeverPasses)
{
	for (const Timestamp stamp : {kT0 + 1000000, kT0 + 10000000, std::numeric_limits<Timestamp>::max()}) {
		const auto make = [stamp](Timestamp t) {
			Inputs in = nominal(t);
			in.validated_timestamp = t;
			in.calibrated_airspeed_m_s = 14.5f;
			in.raw[0].timestamp_sample = stamp;
			in.raw[0].indicated_airspeed_m_s = 14.5f;
			return in;
		};
		EXPECT_FALSE(drive(13.0, make, enabledConfig()).passed) << stamp;
	}
}

TEST(CasValidityTest, FrozenFutureStampedValidatedNeverPasses)
{
	for (const Timestamp stamp : {kT0 + 1000000, kT0 + 10000000, std::numeric_limits<Timestamp>::max()}) {
		const auto make = [stamp](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = stamp; return in; };
		EXPECT_FALSE(drive(13.0, make, enabledConfig()).passed) << stamp;
	}
}

TEST(CasValidityTest, NonFiniteValuesBlock)
{
	for (const float bad : {NAN, INFINITY, -INFINITY}) {
		expectBlocked([bad](Timestamp t) { Inputs in = nominal(t); in.calibrated_airspeed_m_s = bad; return in; },
		enabledConfig(), Reason::NonFinite);
		expectBlocked([bad](Timestamp t) { Inputs in = nominal(t); in.raw[0].indicated_airspeed_m_s = bad; return in; },
		enabledConfig(), Reason::NonFinite);
	}
}

TEST(CasValidityTest, FlagsAndIndexBlock)
{
	expectBlocked([](Timestamp t) { Inputs in = nominal(t); in.measurement_valid = false; return in; },
	enabledConfig(), Reason::MeasurementInvalid);

	for (const int8_t index : {int8_t{0}, int8_t{-1}}) {
		expectBlocked([index](Timestamp t) { Inputs in = nominal(t); in.selected_index = index; return in; },
		enabledConfig(), Reason::IndexNotSensor);
	}

	for (const int8_t index : {int8_t{2}, int8_t{3}, int8_t{4}}) {
		expectBlocked([index](Timestamp t) { Inputs in = nominal(t); in.selected_index = index; return in; },
		enabledConfig(), Reason::IndexNoRawInstance);
	}
}

TEST(CasValidityTest, SensorDisagreement)
{
	Config cfg = enabledConfig();
	cfg.max_disagreement_m_s = kDisagree;
	const auto second = [](float ias) {
		return [ias](Timestamp t) {
			Inputs in = nominal(t);
			in.raw[1].present = true;
			in.raw[1].timestamp_sample = t - 40000;
			in.raw[1].indicated_airspeed_m_s = ias;
			return in;
		};
	};
	expectBlocked(second(kCas + kDisagree + 0.25f), cfg, Reason::Disagreement);
	expectPasses(second(kCas + kDisagree), cfg);

	// No threshold, or a non-finite one (CE411-L3), with a fresh second sensor: fail closed.
	for (const float bad : {0.f, INFINITY, -INFINITY, NAN}) {
		Config unset = enabledConfig();
		unset.max_disagreement_m_s = bad;
		expectBlocked(second(kCas + 50.f), unset, Reason::ThresholdUnset);
	}
}

TEST(CasValidityTest, NoGroundspeedFallbackWhenEnabled)
{
	const DriveResult run = drive(2.0, [](Timestamp t) { Inputs in = nominal(t); in.validated_timestamp = t - 1500000; return in; },
	enabledConfig());
	EXPECT_FALSE(run.passed);
	EXPECT_TRUE(run.all_nan);
}

TEST(CasValidityTest, UnsetThresholdFailsClosed)
{
	for (const int32_t unset : {0, -1, std::numeric_limits<int32_t>::min()}) {
		Config cfg = enabledConfig();
		cfg.max_validated_age_us = unset;
		expectBlocked(nominal, cfg, Reason::ThresholdUnset);

		cfg = enabledConfig();
		cfg.max_raw_sample_age_us = unset;
		expectBlocked(nominal, cfg, Reason::ThresholdUnset);
	}
}

TEST(CasValidityTest, DefaultConfigIsDisabledAndUnset)
{
	const Config cfg{};
	EXPECT_FALSE(cfg.enabled);
	EXPECT_EQ(cfg.max_validated_age_us, 0);
	EXPECT_EQ(cfg.max_raw_sample_age_us, 0);
	EXPECT_FLOAT_EQ(cfg.max_disagreement_m_s, 0.f);
}

TEST(CasValidityTest, StockPathIsBlockedToo)
{
	const auto stale = [](Timestamp t) { Inputs in = nominal(t); in.raw[0].timestamp_sample = t - kTr - 1; return in; };
	EXPECT_TRUE(drive(0.1, nominal, enabledConfig(), 0.f).passed);
	EXPECT_FALSE(drive(2.0, stale, enabledConfig(), 0.f).passed);
}

TEST(CasValidityTest, DisabledOrFwUseAirspd0IsBitIdenticalToLegacy)
{
	const float cas_grid[] = {15.f, 13.9f, -0.27f, 0.f, -0.f, NAN, INFINITY, -INFINITY};
	const int64_t age_grid[] = {0, 1, 999999, 1000000, 1000001, 5000000, static_cast<int64_t>(kT0), -1, -2000000};
	const float vel_grid[][2] = {{0.f, 0.f}, {12.f, 9.f}, {-3.f, 0.5f}, {NAN, 1.f}};

	Config disabled_garbage{};
	disabled_garbage.max_validated_age_us = 1;
	disabled_garbage.max_raw_sample_age_us = -5;
	disabled_garbage.max_disagreement_m_s = NAN;
	const Config configs[] = {Config{}, disabled_garbage, enabledConfig()};

	int compared = 0;

	for (const Config &cfg : configs) {
		for (const float cas : cas_grid) {
			for (const int64_t age : age_grid) {
				for (const auto &vel : vel_grid) {
					for (const bool fw : {false, true}) {
						for (const bool v_xy_valid : {false, true}) {
							Inputs in = nominal(kT0);
							in.calibrated_airspeed_m_s = cas;
							in.validated_timestamp = static_cast<Timestamp>(static_cast<int64_t>(kT0) - age);
							in.vx = vel[0];
							in.vy = vel[1];
							in.fw_use_airspd = fw;
							in.v_xy_valid = v_xy_valid;

							if (cfg.enabled && fw) {
								continue; // the new path; covered above
							}

							Evaluation evaluation{};
							EXPECT_EQ(bits(selectSpeed(in, cfg, evaluation)), bits(legacyOracle(in)));
							compared++;
						}
					}
				}
			}
		}
	}

	EXPECT_GT(compared, 1000);
}

TEST(CasValidityTest, ConfigurationGuard)
{
	Config cfg = enabledConfig();
	EXPECT_EQ(checkConfiguration(Config{}, true, 0, 0), ConfigStatus::Disabled);
	EXPECT_EQ(checkConfiguration(cfg, true, 1, 1), ConfigStatus::Ok);
	EXPECT_EQ(checkConfiguration(cfg, true, 0, 1), ConfigStatus::NoAirspeedSensor);
	EXPECT_EQ(checkConfiguration(cfg, true, 1, 0), ConfigStatus::NoAirspeedSensor);
	EXPECT_EQ(checkConfiguration(cfg, false, 0, 0), ConfigStatus::Ok);
	EXPECT_EQ(checkConfiguration(cfg, true, 1, 2), ConfigStatus::ThresholdUnset);
	cfg.max_disagreement_m_s = kDisagree;
	EXPECT_EQ(checkConfiguration(cfg, true, 1, 2), ConfigStatus::Ok);
	cfg.max_raw_sample_age_us = 0;
	EXPECT_EQ(checkConfiguration(cfg, true, 1, 1), ConfigStatus::ThresholdUnset);
}

// CE411-L7: advisory when 2 * T reaches the pitch-quiet window.
TEST(CasValidityTest, SkewWindowAdvisory)
{
	Config cfg = enabledConfig(); // 2 * 310 ms < 1 s
	EXPECT_FALSE(skewWindowAdvisory(cfg, 1.f));
	EXPECT_FALSE(skewWindowAdvisory(cfg, 0.621f));
	Config edge = enabledConfig();
	edge.max_raw_sample_age_us = 250000;          // 2 * T_R == 0.5 s exactly
	EXPECT_TRUE(skewWindowAdvisory(edge, 0.5f));  // equality is an advisory (>=)
	EXPECT_FALSE(skewWindowAdvisory(edge, 0.5001f));
	EXPECT_TRUE(skewWindowAdvisory(cfg, 0.f));    // stock path: one fresh cycle suffices
	EXPECT_TRUE(skewWindowAdvisory(cfg, NAN));
	cfg.max_validated_age_us = 500000;            // 2 * T_V == 1 s
	EXPECT_TRUE(skewWindowAdvisory(cfg, 1.f));
	EXPECT_FALSE(skewWindowAdvisory(Config{}, 0.f));
	Config unset = enabledConfig();
	unset.max_raw_sample_age_us = 0;
	EXPECT_FALSE(skewWindowAdvisory(unset, 0.f)); // reported by checkConfiguration()
}

TEST(CasValidityTest, ReasonReporterIsEdgeTriggeredAndDefersInsteadOfDropping)
{
	ReasonReporter reporter;
	const Timestamp s = 1000000;
	EXPECT_FALSE(reporter.shouldReport(1 * s, Reason::Usable, s));
	EXPECT_TRUE(reporter.shouldReport(2 * s, Reason::RawStale, s));
	EXPECT_FALSE(reporter.shouldReport(2 * s + 1, Reason::RawStale, s));
	EXPECT_FALSE(reporter.shouldReport(2 * s + 2, Reason::Usable, s));
	EXPECT_FALSE(reporter.shouldReport(2 * s + 3, Reason::RawStale, s));
	EXPECT_FALSE(reporter.shouldReport(2 * s + 4, Reason::NonFinite, s));
	EXPECT_TRUE(reporter.shouldReport(3 * s + 4, Reason::NonFinite, s));
	EXPECT_FALSE(reporter.shouldReport(3 * s + 5, Reason::NonFinite, s));
}

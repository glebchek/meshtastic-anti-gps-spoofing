#pragma once

#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_GPS && !defined(MESHTASTIC_EXCLUDE_GPS_SPOOF_DETECTION)

#include "GSVParser.h"
#include <cstdint>

enum class SpoofResult : uint8_t {
    CLEAN = 0,
    WARN_UNIFORMITY,  // C/N0 std dev too low
    WARN_HIGH_CN0,    // Too many sats with high C/N0
    WARN_SUDDEN_JUMP, // Abrupt C/N0 change across epochs
    WARN_RATIO,       // Abnormal high-to-total satellite ratio
    WARN_ELEVATION,   // No correlation between elevation and C/N0
    SPOOFING_DETECTED // 2+ warnings triggered simultaneously
};

class GPSSpoofDetector {
  public:
    GPSSpoofDetector();

    /**
     * Evaluate satellite C/N0 data for spoofing indicators.
     * Returns the most severe result, or SPOOFING_DETECTED if 2+ warnings triggered.
     * Requires at least 4 satellites to run detection.
     */
    SpoofResult evaluate(const SatelliteCN0 *sats, uint8_t count);

    bool isSpoofingDetected() const { return lastResult == SpoofResult::SPOOFING_DETECTED; }
    float lastStdDev() const { return cachedStdDev; }
    float lastMeanCN0() const { return cachedMean; }
    uint8_t lastHighCN0Count() const { return cachedHighCount; }

  private:
    // Thresholds
    static constexpr float UNIFORMITY_STDDEV_THRESHOLD = 3.5f;   // dBHz
    static constexpr float HIGH_CN0_THRESHOLD = 45.0f;           // dBHz
    static constexpr float HIGH_CN0_FRACTION = 0.80f;            // 80%
    static constexpr float SUDDEN_JUMP_THRESHOLD = 10.0f;        // dBHz avg delta
    static constexpr float RATIO_CN0_THRESHOLD = 40.0f;          // dBHz
    static constexpr float RATIO_FRACTION = 0.80f;               // 80%
    static constexpr uint8_t RATIO_MIN_SATS = 8;                 // Only check ratio if enough sats
    static constexpr uint8_t MIN_SATS_FOR_DETECTION = 4;
    static constexpr uint8_t MIN_CN0_FOR_ANALYSIS = 20;       // Ignore weak/almanac-only sats
    static constexpr float ELEV_CORR_THRESHOLD = 0.5f;        // Min expected Pearson correlation
    static constexpr uint8_t ELEV_MIN_SATS = 6;               // Need enough sats with elevation
    static constexpr uint8_t ELEV_MIN_SPREAD = 30;             // Min elevation spread (degrees)

    // Previous epoch data for sudden-jump detection
    SatelliteCN0 prevSats[MAX_TRACKED_SATS] = {};
    uint8_t prevSatCount = 0;
    bool hasPreviousEpoch = false;

    // Cached stats from last evaluation
    float cachedStdDev = 0.0f;
    float cachedMean = 0.0f;
    uint8_t cachedHighCount = 0;
    SpoofResult lastResult = SpoofResult::CLEAN;

    bool checkUniformity(const SatelliteCN0 *sats, uint8_t count);
    bool checkHighCN0(const SatelliteCN0 *sats, uint8_t count);
    bool checkSuddenJump(const SatelliteCN0 *sats, uint8_t count);
    bool checkRatio(const SatelliteCN0 *sats, uint8_t count);
    bool checkElevationCorrelation(const SatelliteCN0 *sats, uint8_t count);

    void storePreviousEpoch(const SatelliteCN0 *sats, uint8_t count);
};

#endif // !MESHTASTIC_EXCLUDE_GPS && !MESHTASTIC_EXCLUDE_GPS_SPOOF_DETECTION

#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_GPS && !defined(MESHTASTIC_EXCLUDE_GPS_SPOOF_DETECTION)

#include "GPSSpoofDetector.h"
#include <cmath>
#include <cstring>

GPSSpoofDetector::GPSSpoofDetector()
{
    memset(prevSats, 0, sizeof(prevSats));
}

SpoofResult GPSSpoofDetector::evaluate(const SatelliteCN0 *sats, uint8_t count)
{
    // Filter out weak/almanac-only satellites that don't contribute to nav solution
    // but can mask uniformity of spoofed signals
    SatelliteCN0 filtered[MAX_TRACKED_SATS];
    uint8_t filteredCount = 0;
    for (uint8_t i = 0; i < count && filteredCount < MAX_TRACKED_SATS; i++) {
        if (sats[i].cn0 >= MIN_CN0_FOR_ANALYSIS) {
            filtered[filteredCount++] = sats[i];
        }
    }

    if (filteredCount < MIN_SATS_FOR_DETECTION) {
        storePreviousEpoch(filtered, filteredCount);
        lastResult = SpoofResult::CLEAN;
        return lastResult;
    }

    uint8_t warnings = 0;
    SpoofResult singleResult = SpoofResult::CLEAN;

    if (checkUniformity(filtered, filteredCount)) {
        warnings++;
        singleResult = SpoofResult::WARN_UNIFORMITY;
    }
    if (checkHighCN0(filtered, filteredCount)) {
        warnings++;
        singleResult = SpoofResult::WARN_HIGH_CN0;
    }
    if (checkSuddenJump(filtered, filteredCount)) {
        warnings++;
        singleResult = SpoofResult::WARN_SUDDEN_JUMP;
    }
    if (checkRatio(filtered, filteredCount)) {
        warnings++;
        singleResult = SpoofResult::WARN_RATIO;
    }
    if (checkElevationCorrelation(filtered, filteredCount)) {
        warnings++;
        singleResult = SpoofResult::WARN_ELEVATION;
    }

    storePreviousEpoch(filtered, filteredCount);

    if (warnings >= 2) {
        lastResult = SpoofResult::SPOOFING_DETECTED;
    } else if (warnings == 1) {
        lastResult = singleResult;
    } else {
        lastResult = SpoofResult::CLEAN;
    }

    return lastResult;
}

bool GPSSpoofDetector::checkUniformity(const SatelliteCN0 *sats, uint8_t count)
{
    // Compute mean
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += sats[i].cn0;
    }
    cachedMean = sum / count;

    // Compute std dev
    float variance = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        float diff = sats[i].cn0 - cachedMean;
        variance += diff * diff;
    }
    cachedStdDev = sqrtf(variance / count);

    return cachedStdDev < UNIFORMITY_STDDEV_THRESHOLD;
}

bool GPSSpoofDetector::checkHighCN0(const SatelliteCN0 *sats, uint8_t count)
{
    uint8_t highCount = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (sats[i].cn0 > HIGH_CN0_THRESHOLD) {
            highCount++;
        }
    }
    cachedHighCount = highCount;

    return (float)highCount / count > HIGH_CN0_FRACTION;
}

bool GPSSpoofDetector::checkSuddenJump(const SatelliteCN0 *sats, uint8_t count)
{
    if (!hasPreviousEpoch)
        return false;

    // Match satellites by PRN and compute average absolute C/N0 delta
    float totalDelta = 0.0f;
    uint8_t matched = 0;

    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = 0; j < prevSatCount; j++) {
            if (sats[i].prn == prevSats[j].prn) {
                float delta = (float)sats[i].cn0 - (float)prevSats[j].cn0;
                totalDelta += fabsf(delta);
                matched++;
                break;
            }
        }
    }

    if (matched < MIN_SATS_FOR_DETECTION)
        return false;

    float avgDelta = totalDelta / matched;
    return avgDelta > SUDDEN_JUMP_THRESHOLD;
}

bool GPSSpoofDetector::checkRatio(const SatelliteCN0 *sats, uint8_t count)
{
    if (count < RATIO_MIN_SATS)
        return false;

    uint8_t aboveThreshold = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (sats[i].cn0 > RATIO_CN0_THRESHOLD) {
            aboveThreshold++;
        }
    }

    return (float)aboveThreshold / count > RATIO_FRACTION;
}

bool GPSSpoofDetector::checkElevationCorrelation(const SatelliteCN0 *sats, uint8_t count)
{
    // Collect satellites that have valid elevation data
    float elev[MAX_TRACKED_SATS];
    float cn0[MAX_TRACKED_SATS];
    uint8_t n = 0;

    for (uint8_t i = 0; i < count && n < MAX_TRACKED_SATS; i++) {
        if (sats[i].elevation > 0 && sats[i].elevation <= 90) {
            elev[n] = (float)sats[i].elevation;
            cn0[n] = (float)sats[i].cn0;
            n++;
        }
    }

    if (n < ELEV_MIN_SATS)
        return false;

    // Check elevation spread - need meaningful range to compute correlation
    float minElev = elev[0], maxElev = elev[0];
    for (uint8_t i = 1; i < n; i++) {
        if (elev[i] < minElev)
            minElev = elev[i];
        if (elev[i] > maxElev)
            maxElev = elev[i];
    }
    if ((maxElev - minElev) < ELEV_MIN_SPREAD)
        return false;

    // Pearson correlation coefficient between elevation and C/N0
    float sumE = 0, sumC = 0;
    for (uint8_t i = 0; i < n; i++) {
        sumE += elev[i];
        sumC += cn0[i];
    }
    float meanE = sumE / n;
    float meanC = sumC / n;

    float cov = 0, varE = 0, varC = 0;
    for (uint8_t i = 0; i < n; i++) {
        float dE = elev[i] - meanE;
        float dC = cn0[i] - meanC;
        cov += dE * dC;
        varE += dE * dE;
        varC += dC * dC;
    }

    if (varE < 1.0f || varC < 1.0f)
        return false; // Degenerate case

    float r = cov / sqrtf(varE * varC);

    // Real sky: positive correlation (higher elevation = higher C/N0)
    // Spoofed: near-zero or negative correlation
    return r < ELEV_CORR_THRESHOLD;
}

void GPSSpoofDetector::storePreviousEpoch(const SatelliteCN0 *sats, uint8_t count)
{
    uint8_t toCopy = count < MAX_TRACKED_SATS ? count : MAX_TRACKED_SATS;
    memcpy(prevSats, sats, toCopy * sizeof(SatelliteCN0));
    prevSatCount = toCopy;
    hasPreviousEpoch = true;
}

#endif // !MESHTASTIC_EXCLUDE_GPS && !MESHTASTIC_EXCLUDE_GPS_SPOOF_DETECTION

# GPS Spoofing Detection via C/N0 Analysis

## Overview

This module adds C/N0-based GPS spoofing detection for devices using the L76K GNSS chip (`GNSS_MODEL_MTK`). It parses GSV sentences to extract per-satellite signal data and applies statistical heuristics to identify spoofed GPS signals. When spoofing is detected, the device rejects GPS position fixes and behaves as if it has no GPS lock.

## Architecture

```
Serial RX bytes
      |
      v
+------------------+     +---------------------+
|   TinyGPS++      |     |    GSVParser         |
|  (GGA, RMC, etc) |     | (GSV sentences only) |
+------------------+     +---------------------+
                                   |
                          PRN + C/N0 + Elevation
                                   |
                                   v
                          +---------------------+
                          | GPSSpoofDetector     |
                          | (5 heuristic checks) |
                          +---------------------+
                                   |
                            SpoofResult
                                   |
                                   v
                          +---------------------+
                          |  GPS::whileActive()  |
                          | spoofingDetected flag |
                          +---------------------+
                                   |
                                   v
                          +---------------------+
                          | GPS::lookForLocation()|
                          |  reject if spoofed   |
                          +---------------------+
```

## Files

| File | Type | Description |
|------|------|-------------|
| `src/gps/GSVParser.h` | New (header-only) | NMEA GSV sentence state-machine parser |
| `src/gps/GPSSpoofDetector.h` | New | Spoofing detection engine header |
| `src/gps/GPSSpoofDetector.cpp` | New | Spoofing detection engine implementation |
| `src/gps/GPS.h` | Modified | Added parser, detector, and debounce members |
| `src/gps/GPS.cpp` | Modified | Integration: setup, byte feed, position rejection |

## GSVParser

A lightweight, header-only state-machine parser for NMEA GSV sentences. It is fed character-by-character from the same serial stream as TinyGPS++, in parallel.

**Parsed data per satellite:**
- PRN (satellite ID)
- C/N0 (carrier-to-noise ratio, dBHz)
- Elevation (degrees above horizon)

**Key properties:**
- Handles all talker IDs: GP (GPS), GL (GLONASS), GB (BeiDou), GA (Galileo)
- Tracks multi-sentence groups (e.g., 3 GPGSV sentences for 11 satellites)
- Validates NMEA checksums
- No heap allocation; fixed array for up to 24 satellites
- Returns `true` from `encode()` when a complete group is received

## GPSSpoofDetector

The detection engine runs 5 independent heuristic checks on the satellite data. A `SPOOFING_DETECTED` result requires **2 or more** checks to trigger simultaneously.

### Pre-filtering

Before any checks run, satellites with C/N0 < 20 dBHz are excluded. These are typically almanac-only entries or barely-tracked satellites that don't contribute to the navigation solution but can mask the uniformity of spoofed signals.

### Detection Checks

| # | Check | Condition | Threshold | Rationale |
|---|-------|-----------|-----------|-----------|
| 1 | **Uniformity** | Std dev of C/N0 (requires >= 8 sats) | < 3.5 dBHz | Real satellites have varied C/N0 (5-15 dBHz std dev) due to different elevations and atmospheric paths. A single-antenna spoofer broadcasts all signals at similar power. Skipped when fewer than 8 satellites are available, since indoor reception with few satellites naturally produces low C/N0 variance due to uniform wall attenuation. |
| 2 | **High C/N0** | Fraction with C/N0 > 45 | > 80% | Low-elevation satellites naturally have low C/N0. A spoofer often over-powers all signals. |
| 3 | **Sudden Jump** | Avg absolute C/N0 delta vs previous epoch (matched by PRN) | > 10.0 dBHz | Real C/N0 changes 1-3 dBHz per epoch. Spoofing onset causes an abrupt jump across all satellites. |
| 4 | **Ratio** | Fraction with C/N0 > 40 (when > 8 sats) | > 80% | Sanity check correlating with the High C/N0 check at a different threshold. |
| 5 | **Elevation Correlation** | Pearson correlation between elevation and C/N0 | r < 0.5 | Physics: higher elevation = shorter atmospheric path = higher C/N0. A spoofer transmits all signals from one point, so "claimed" elevation has no correlation with actual signal strength. Requires >= 6 satellites with elevation data and >= 30 degree elevation spread. |

### Composite Decision

- 0 warnings: `CLEAN`
- 1 warning: `WARN_*` (logged but position not rejected)
- 2+ warnings: `SPOOFING_DETECTED` (position rejected)

Minimum 4 satellites (after filtering) are required to run detection. Below this threshold, the result is always `CLEAN`.

Individual checks have their own minimum satellite requirements:
- **Uniformity**: >= 8 satellites (indoor signals with few sats have naturally low C/N0 variance)
- **Ratio**: >= 8 satellites
- **Elevation Correlation**: >= 6 satellites with valid elevation data, >= 30° elevation spread

### Memory Usage

Approximately 160 bytes total:
- Current epoch satellite data (72 bytes)
- Previous epoch satellite data for sudden-jump detection (72 bytes)
- Cached statistics and state (~16 bytes)

## GPS Integration

### Setup (L76K only)

The `$PCAS03` configuration command is modified to enable GSV output every 3rd fix:

```
Before: $PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02  (GGA + RMC only)
After:  $PCAS03,1,0,0,3,1,0,0,0,0,0,,,0,0*01  (GGA + GSV@3 + RMC)
```

Bandwidth impact at 9600 baud (~960 bytes/sec):
- Existing GGA + RMC: ~150 bytes/sec
- Added GSV (average): ~80 bytes/sec
- Total: ~230 bytes/sec (24% utilization)

### Byte Processing

In `GPS::whileActive()`, each received byte is fed to both `reader.encode(c)` (TinyGPS++) and `gsvParser.encode(c)`. When a complete GSV group is received, the detector evaluates it.

### Position Rejection

In `GPS::lookForLocation()`, after confirming GPS lock, the `spoofingDetected` flag is checked. If true, the function returns `false`, causing the device to behave as if it has no lock. The existing timeout logic in `runOnce()` handles this gracefully.

### Debounce

To prevent rapid toggling between `SPOOFING_DETECTED` and `CLEAN` (which occurs because GSV groups from different constellations arrive at different times), the `spoofingDetected` flag uses hold logic:

- Set immediately on `SPOOFING_DETECTED`
- Cleared only after **3 consecutive** `CLEAN` epochs (~9 seconds at GSV every 3 seconds)
- A single re-detection resets the counter

## Compile Guard

All spoof detection code is guarded by:

```cpp
#if !defined(MESHTASTIC_EXCLUDE_GPS_SPOOF_DETECTION)
```

Define this macro to completely exclude spoof detection from the build. When excluded, the original `$PCAS03` command (GGA + RMC only) is used, and no additional memory is consumed.

## Log Messages

| Level | Message | Meaning |
|-------|---------|---------|
| WARN | `GPS SPOOFING DETECTED: C/N0 anomaly (mean=X, stddev=X, high=X)` | 2+ heuristic checks triggered; position will be rejected |
| INFO | `GPS spoofing condition cleared` | 3 consecutive clean epochs; position accepted again |
| WARN | `GPS position rejected: spoofing detected` | A GPS fix was available but discarded due to active spoofing flag |

## Limitations

- Only active for L76K (`GNSS_MODEL_MTK`). Other GNSS chips do not enable GSV output.
- The elevation correlation check requires satellites with elevation data in the GSV sentence. Some constellations (notably GLONASS on L76K) may not always include elevation, reducing the number of satellites available for this check.
- Thresholds are hardcoded. Field testing in different environments (urban canyons, dense forest, indoor) may require adjustment.
- The system detects C/N0-based anomalies. It does not detect replay attacks where the spoofer accurately simulates real sky signal characteristics.

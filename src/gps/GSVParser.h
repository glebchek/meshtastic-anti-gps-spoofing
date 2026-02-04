#pragma once

#include <cstdint>
#include <cstring>

/**
 * Lightweight state-machine parser for NMEA GSV sentences.
 * Fed character-by-character in parallel with TinyGPS++.
 *
 * GSV format:
 * $GPGSV,3,1,11,01,45,045,38,02,32,120,42,03,78,290,35,04,12,315,28*7C
 *                   ^^          ^^                  ^^          ^^
 *                   PRN         C/N0                PRN         C/N0
 *
 * Handles talker IDs: GP, GL, GB, GA (GPS, GLONASS, BeiDou, Galileo)
 * No heap allocation; fixed arrays.
 */

static constexpr uint8_t MAX_TRACKED_SATS = 24;

struct SatelliteCN0 {
    uint8_t prn;       // Satellite PRN/ID
    uint8_t cn0;       // C/N0 in dBHz (0-99)
    uint8_t elevation; // Elevation in degrees (0-90), 0 if unknown
};

class GSVParser {
  public:
    /**
     * Feed one byte from the NMEA stream.
     * Returns true when a complete GSV group has been received
     * (all sentences in the group, e.g. 3 of 3).
     */
    bool encode(char c)
    {
        switch (state) {
        case IDLE:
            if (c == '$') {
                state = TALKER;
                talkerPos = 0;
                checksum = 0;
                fieldNum = 0;
                fieldPos = 0;
                fieldBuf[0] = '\0';
            }
            break;

        case TALKER:
            checksum ^= (uint8_t)c;
            if (talkerPos < sizeof(talkerBuf) - 1) {
                talkerBuf[talkerPos++] = c;
                talkerBuf[talkerPos] = '\0';
            }
            // After 5 chars we have e.g. "GPGSV"
            if (talkerPos == 5) {
                // Check if this is a GSV sentence (last 3 chars = "GSV")
                if (talkerBuf[2] == 'G' && talkerBuf[3] == 'S' && talkerBuf[4] == 'V') {
                    state = HEADER_COMMA;
                } else {
                    state = IDLE;
                }
            }
            break;

        case HEADER_COMMA:
            // Consume the comma after talker+type
            if (c == ',') {
                checksum ^= (uint8_t)c;
                fieldNum = 1; // Next field is field 1 (total sentences)
                fieldPos = 0;
                fieldBuf[0] = '\0';
                state = FIELDS;
            } else {
                state = IDLE; // Unexpected
            }
            break;

        case FIELDS:
            if (c == ',' || c == '*') {
                if (c == ',')
                    checksum ^= (uint8_t)c;
                processField();
                fieldNum++;
                fieldPos = 0;
                fieldBuf[0] = '\0';
                if (c == '*') {
                    state = CHECKSUM1;
                }
            } else if (c == '\r' || c == '\n') {
                // Premature end
                state = IDLE;
            } else {
                checksum ^= (uint8_t)c;
                if (fieldPos < sizeof(fieldBuf) - 1) {
                    fieldBuf[fieldPos++] = c;
                    fieldBuf[fieldPos] = '\0';
                }
            }
            break;

        case CHECKSUM1:
            checksumRx = hexVal(c) << 4;
            state = CHECKSUM2;
            break;

        case CHECKSUM2: {
            checksumRx |= hexVal(c);
            state = IDLE;
            if (checksumRx == checksum) {
                return onSentenceComplete();
            }
            // Checksum mismatch - discard
            break;
        }
        }

        return false;
    }

    uint8_t getSatCount() const { return satCount; }
    const SatelliteCN0 *getSatellites() const { return satellites; }

    void reset()
    {
        satCount = 0;
        totalSentences = 0;
        currentSentence = 0;
        state = IDLE;
    }

  private:
    enum State : uint8_t { IDLE, TALKER, HEADER_COMMA, FIELDS, CHECKSUM1, CHECKSUM2 };

    State state = IDLE;
    uint8_t checksum = 0;
    uint8_t checksumRx = 0;

    char talkerBuf[6] = {};
    uint8_t talkerPos = 0;

    char fieldBuf[8] = {};
    uint8_t fieldPos = 0;
    uint8_t fieldNum = 0; // 1-based within the GSV sentence

    // GSV group tracking
    uint8_t totalSentences = 0;
    uint8_t currentSentence = 0;

    // Satellite data accumulation
    SatelliteCN0 satellites[MAX_TRACKED_SATS] = {};
    uint8_t satCount = 0;

    // Temp storage for current satellite block within a sentence
    uint8_t tempPRN = 0;
    uint8_t tempElev = 0;

    static uint8_t hexVal(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        return 0;
    }

    uint8_t parseU8() const
    {
        uint8_t val = 0;
        for (uint8_t i = 0; i < fieldPos; i++) {
            val = val * 10 + (fieldBuf[i] - '0');
        }
        return val;
    }

    /**
     * GSV field layout (1-based):
     *   1: total sentences in group
     *   2: sentence number
     *   3: total sats in view
     *   4,5,6,7:   sat1 PRN, elevation, azimuth, C/N0
     *   8,9,10,11:  sat2 ...
     *   12,13,14,15: sat3 ...
     *   16,17,18,19: sat4 ...
     */
    void processField()
    {
        if (fieldPos == 0)
            return; // Empty field

        switch (fieldNum) {
        case 1:
            totalSentences = parseU8();
            break;
        case 2: {
            uint8_t sentNum = parseU8();
            if (sentNum == 1) {
                // First sentence of a new group - reset accumulation
                satCount = 0;
            }
            currentSentence = sentNum;
            break;
        }
        case 3:
            // Total sats in view (informational, we count ourselves)
            break;
        default:
            // Satellite data fields: groups of 4 starting at field 4
            if (fieldNum >= 4) {
                uint8_t offset = (fieldNum - 4) % 4;
                if (offset == 0) {
                    // PRN
                    tempPRN = parseU8();
                    tempElev = 0;
                } else if (offset == 1) {
                    // Elevation
                    tempElev = parseU8();
                } else if (offset == 3) {
                    // C/N0 (4th in each sat block)
                    if (satCount < MAX_TRACKED_SATS) {
                        satellites[satCount].prn = tempPRN;
                        satellites[satCount].cn0 = parseU8();
                        satellites[satCount].elevation = tempElev;
                        satCount++;
                    }
                }
                // offset 2 (azimuth) is ignored
            }
            break;
        }
    }

    /**
     * Called when a valid GSV sentence is complete.
     * Returns true when the entire group is complete.
     */
    bool onSentenceComplete()
    {
        return (totalSentences > 0 && currentSentence == totalSentences);
    }
};

#pragma once

// Shared by the swarm sensor modules: compact JSON for PRIVATE_APP packets and the common sampling schedule.

#include "NodeDB.h"
#include "RTC.h"
#include <algorithm>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Sensor power schedule (SensorPower.cpp): every SENSOR_CYCLE_SECS the sensor supply is switched on for SENSOR_ON_SECS,
// and the sensors are read SENSOR_WARMUP_SECS after power-on.
#ifndef SENSOR_CYCLE_SECS
#define SENSOR_CYCLE_SECS (30 * 60)
#endif
#ifndef SENSOR_ON_SECS
#define SENSOR_ON_SECS (3 * 60)
#endif
#ifndef SENSOR_WARMUP_SECS
#define SENSOR_WARMUP_SECS (2 * 60)
#endif
static_assert(SENSOR_ON_SECS < SENSOR_CYCLE_SECS, "The sensor power window must be shorter than the cycle");
// Worst case the sonde + weather station reads (all retries timing out) take about 40 s after warm-up.
static_assert(SENSOR_ON_SECS - SENSOR_WARMUP_SECS >= 45, "Leave at least 45 s of power after warm-up for the readings");
#ifndef SONDE_SIMULATE
#define SONDE_SIMULATE 0
#endif

namespace swarm
{

// Instruments report -9999 (or garbage) for missing probes.
inline bool usable(float v)
{
    return isfinite(v) && v > -9000.0f && fabsf(v) < 1.0e6f;
}

struct JsonWriter {
    char *buf;
    size_t cap;
    size_t len = 0;
    bool ok = true;

    JsonWriter(char *b, size_t c) : buf(b), cap(c) { raw("{"); }

    void raw(const char *s)
    {
        size_t n = strlen(s);
        if (!ok || len + n + 1 > cap) {
            ok = false;
            return;
        }
        memcpy(buf + len, s, n);
        len += n;
        buf[len] = '\0';
    }

    void key(const char *k)
    {
        raw(len > 1 ? ",\"" : "\"");
        raw(k);
        raw("\":");
    }

    void integer(const char *k, uint32_t v)
    {
        char tmp[12];
        snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)v);
        key(k);
        raw(tmp);
    }

    void str(const char *k, const char *v)
    {
        key(k);
        raw("\"");
        raw(v);
        raw("\"");
    }

    // Integer-only fixed-point formatting so the output doesn't depend on newlib float printf support.
    void fixedValue(int32_t scaled, uint8_t decimals)
    {
        char tmp[16];
        char *p = tmp + sizeof(tmp);
        *--p = '\0';
        uint32_t mag = scaled < 0 ? (uint32_t)(-(int64_t)scaled) : (uint32_t)scaled;
        for (uint8_t i = 0; i < decimals; i++) {
            *--p = '0' + mag % 10;
            mag /= 10;
        }
        if (decimals)
            *--p = '.';
        do {
            *--p = '0' + mag % 10;
            mag /= 10;
        } while (mag);
        if (scaled < 0)
            *--p = '-';
        raw(p);
    }

    void fixed(const char *k, int32_t scaled, uint8_t decimals)
    {
        key(k);
        fixedValue(scaled, decimals);
    }

    static int32_t scaled(float v, uint8_t decimals)
    {
        static const float scale[] = {1.0f, 10.0f, 100.0f, 1000.0f};
        return (int32_t)lroundf(v * scale[decimals]);
    }

    void number(const char *k, float v, uint8_t decimals)
    {
        if (!usable(v))
            return;
        fixed(k, scaled(v, decimals), decimals);
    }

    // Array of readings; unusable entries become null so positions stay aligned.
    void numberArray(const char *k, const float *values, size_t count, uint8_t decimals)
    {
        key(k);
        raw("[");
        for (size_t i = 0; i < count; i++) {
            if (i)
                raw(",");
            if (usable(values[i]))
                fixedValue(scaled(values[i], decimals), decimals);
            else
                raw("null");
        }
        raw("]");
    }

    size_t finish()
    {
        raw("}");
        return ok ? len : 0;
    }
};

inline void addPosition(JsonWriter &j)
{
    if (localPosition.latitude_i != 0 || localPosition.longitude_i != 0) {
        j.fixed("lat", localPosition.latitude_i / 10, 6);
        j.fixed("lon", localPosition.longitude_i / 10, 6);
    }
}

// Milliseconds until this node's next sampling slot. Slots are aligned to wall-clock boundaries (e.g. :00 and :30) so
// the swarm samples together, offset per node by a fixed hash of its ID so nodes don't all transmit at once.
inline int32_t msUntilNextSlot()
{
    const uint64_t intervalMs = (uint64_t)SENSOR_CYCLE_SECS * 1000;
    uint32_t now = getValidTime(RTCQualityDevice);
    if (now == 0)
        return (int32_t)intervalMs;

    const uint64_t spreadMs = std::min<uint64_t>(intervalMs / 2, 60 * 1000);
    const uint64_t offsetMs = spreadMs ? ((uint64_t)nodeDB->getNodeNum() * 2654435761u) % spreadMs : 0;
    const uint64_t nowMs = (uint64_t)now * 1000;
    uint64_t target = ((nowMs + intervalMs - offsetMs) / intervalMs) * intervalMs + offsetMs;
    // `now` only has 1 s resolution, so make sure we never re-fire the slot we just ran.
    if (target < nowMs + 2000)
        target += intervalMs;
    return (int32_t)(target - nowMs);
}

} // namespace swarm

#include "SensorPower.h"

#if defined(HAS_SONDE) || defined(HAS_TEMP_STRING)

#include "RTC.h"
#include "SwarmReport.h"
#ifdef HAS_SONDE
#include "SondeModule.h"
#endif
#ifdef HAS_TEMP_STRING
#include "TempStringModule.h"
#endif

#ifndef SENSOR_POWER_PIN
#define SENSOR_POWER_PIN -1
#endif
#ifndef SENSOR_POWER_ON_LEVEL
#define SENSOR_POWER_ON_LEVEL HIGH
#endif

SensorPower *sensorPower;

// Long enough for the radio and mesh to come up, short enough that a rebooted node reports right away.
static constexpr int32_t FIRST_WINDOW_DELAY_MS = 30 * 1000;
static constexpr uint32_t CYCLE_MS = (uint32_t)SENSOR_CYCLE_SECS * 1000;
static constexpr uint32_t ON_MS = (uint32_t)SENSOR_ON_SECS * 1000;
static constexpr uint32_t WARMUP_MS = (uint32_t)SENSOR_WARMUP_SECS * 1000;

SensorPower::SensorPower() : concurrency::OSThread("SensorPower") {}

void SensorPower::setPower(bool on)
{
#if SENSOR_POWER_PIN >= 0
    digitalWrite(SENSOR_POWER_PIN, on ? SENSOR_POWER_ON_LEVEL : !SENSOR_POWER_ON_LEVEL);
#endif
#ifdef HAS_SONDE
    if (sondeModule)
        sondeModule->sensorsPowered(on);
#endif
}

int32_t SensorPower::msUntilNextWindow() const
{
    if (getValidTime(RTCQualityDevice))
        return swarm::msUntilNextSlot();
    // No clock yet, so no shared slots: keep a fixed cycle measured from this window's start.
    const uint32_t elapsed = millis() - windowStartMs;
    return elapsed < CYCLE_MS ? (int32_t)(CYCLE_MS - elapsed) : 0;
}

int32_t SensorPower::runOnce()
{
    switch (step) {
    case Step::Boot:
#if SENSOR_POWER_PIN >= 0
        pinMode(SENSOR_POWER_PIN, OUTPUT);
#endif
        setPower(false);
        LOG_INFO("Sensors: supply on pin %d, on for %us every %us, read %us after power-on", SENSOR_POWER_PIN, SENSOR_ON_SECS,
                 SENSOR_CYCLE_SECS, SENSOR_WARMUP_SECS);
        step = Step::PowerOn;
        return FIRST_WINDOW_DELAY_MS;

    case Step::PowerOn:
        windowStartMs = millis();
        LOG_INFO("Sensors: power on");
        setPower(true);
        step = Step::Sample;
        return WARMUP_MS;

    case Step::Sample:
        LOG_INFO("Sensors: warmed up, reading");
#ifdef HAS_SONDE
        if (sondeModule)
            sondeModule->sampleNow();
#endif
#ifdef HAS_TEMP_STRING
        if (tempStringModule)
            tempStringModule->sampleNow();
#endif
        step = Step::PowerOff;
        return ON_MS - WARMUP_MS;

    case Step::PowerOff: {
        setPower(false);
        const int32_t next = msUntilNextWindow();
        LOG_INFO("Sensors: power off, next window in %ds", next / 1000);
        step = Step::PowerOn;
        return next;
    }
    }
    return FIRST_WINDOW_DELAY_MS;
}

#endif

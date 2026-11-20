#pragma once

#include "configuration.h"

#if defined(HAS_SONDE) || defined(HAS_TEMP_STRING)

#include "concurrency/OSThread.h"

/**
 * The one schedule every swarm sensor follows. At the start of each SENSOR_CYCLE_SECS slot it switches the sensor
 * supply on (SENSOR_POWER_PIN, if fitted) and tells the sensor modules, asks them for their readings
 * SENSOR_WARMUP_SECS later, and switches the supply off SENSOR_ON_SECS after power-on. Slots are the swarm's shared
 * wall-clock slots (swarm::msUntilNextSlot); the first window opens shortly after boot.
 */
class SensorPower : private concurrency::OSThread
{
  public:
    SensorPower();

  protected:
    int32_t runOnce() override;

  private:
    enum class Step : uint8_t { Boot, PowerOn, Sample, PowerOff };

    Step step = Step::Boot;
    uint32_t windowStartMs = 0;

    void setPower(bool on);
    int32_t msUntilNextWindow() const;
};

extern SensorPower *sensorPower;

#endif

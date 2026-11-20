#pragma once

#include "configuration.h"

#ifdef HAS_TEMP_STRING

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "temp_string_ids.h"
#include <DallasTemperature.h>
#include <OneWire.h>

/**
 * DS18B20 temperature string on a 1-Wire bus (TEMP_STRING_PIN). When SensorPower says the sensors are warm it finds the
 * probes (they were unpowered between windows), starts one conversion on every probe, reads them one per step so the
 * mesh keeps running, then sends one JSON packet ("type":"ts", "ts":[...] in string order) on PRIVATE_APP. Probes are
 * addressed by ROM code, so a dead or missing probe only blanks its own position.
 */
class TempStringModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    TempStringModule();

    /// Called by SensorPower once the sensors have warmed up: take and send this cycle's readings.
    void sampleNow();

  protected:
    int32_t runOnce() override;

  private:
    enum class Step : uint8_t { Boot, Idle, Convert, Read, Report };

    OneWire bus;
    DallasTemperature sensors;
    Step step = Step::Boot;
    float temps[TEMP_STRING_COUNT] = {};
    uint8_t index = 0;
    uint8_t valid = 0;
    uint16_t seq = 0;
    uint8_t lastTotal = 0xFF;

    uint8_t discover();
    float readProbe(const uint8_t *rom);
    void simulate();
    void sendReport();
};

extern TempStringModule *tempStringModule;

#endif

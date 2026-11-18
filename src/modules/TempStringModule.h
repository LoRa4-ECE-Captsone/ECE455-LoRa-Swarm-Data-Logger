#pragma once

#include "configuration.h"

#ifdef HAS_TEMP_STRING

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "temp_string_ids.h"
#include <DallasTemperature.h>
#include <OneWire.h>

/**
 * DS18B20 temperature string on a 1-Wire bus (TEMP_STRING_PIN). Each sampling slot it starts one conversion on every
 * probe, waits for it, reads the probes one per step so the mesh keeps running, then sends one JSON packet
 * ("type":"ts", "ts":[...] in string order) on PRIVATE_APP. Probes are addressed by ROM code, so a dead or missing probe
 * only blanks its own position.
 */
class TempStringModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    TempStringModule();

  protected:
    int32_t runOnce() override;

  private:
    enum class Step : uint8_t { Boot, Convert, Read, Report };

    OneWire bus;
    DallasTemperature sensors;
    Step step = Step::Boot;
    float temps[TEMP_STRING_COUNT] = {};
    uint8_t index = 0;
    uint8_t valid = 0;
    uint16_t seq = 0;
    uint32_t lastBusWarnMs = 0;
    bool busWarned = false;

    void discover();
    float readProbe(const uint8_t *rom);
    void simulate();
    void sendReport();
};

extern TempStringModule *tempStringModule;

#endif

#include "TempStringModule.h"

#ifdef HAS_TEMP_STRING

#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "SwarmReport.h"
#include "Throttle.h"
#include <math.h>
#include <string.h>

TempStringModule *tempStringModule;

// 14 values at worst "-55.00," plus the header, position and sequence fields must fit one 233-byte payload.
static_assert(TEMP_STRING_COUNT <= 16, "More probes than fit in one packet; split the string across two packets");

static constexpr int32_t FIRST_CYCLE_DELAY_MS = 20 * 1000;
static constexpr int32_t CONVERSION_MS = 800; // 12-bit conversion takes 750 ms
static constexpr int32_t STEP_MS = 20;
static constexpr uint32_t BUS_WARN_INTERVAL_MS = 10 * 60 * 1000;
static constexpr uint8_t READ_ATTEMPTS = 2;

static constexpr uint8_t CMD_CONVERT_T = 0x44;
static constexpr uint8_t CMD_READ_SCRATCHPAD = 0xBE;

TempStringModule::TempStringModule()
    : SinglePortModule("tempstring", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("TempString"), bus(TEMP_STRING_PIN)
{
}

static void formatRom(const uint8_t *rom, char *out)
{
    for (uint8_t i = 0; i < 8; i++)
        snprintf(out + 3 * i, 4, i < 7 ? "%02X:" : "%02X", rom[i]);
}

void TempStringModule::discover()
{
    if (!bus.reset()) {
        LOG_WARN("TempString: no 1-Wire devices answer on pin %u (check wiring and the 4.7k pull-up to 3.3 V)", TEMP_STRING_PIN);
        return;
    }

    bool matched[TEMP_STRING_COUNT] = {};
    uint8_t rom[8];
    uint8_t total = 0;
    char text[24];
    bus.reset_search();
    while (bus.search(rom) && total < 64) {
        if (OneWire::crc8(rom, 7) != rom[7])
            continue;
        total++;
        bool known = false;
        for (uint8_t i = 0; i < TEMP_STRING_COUNT; i++) {
            if (memcmp(rom, TEMP_STRING_ROMS[i], 8) == 0) {
                matched[i] = known = true;
                break;
            }
        }
        if (!known) {
            formatRom(rom, text);
            LOG_WARN("TempString: unlisted 1-Wire device %s on the bus (not in temp_string_ids.h)", text);
        }
    }

    uint8_t found = 0;
    for (uint8_t i = 0; i < TEMP_STRING_COUNT; i++) {
        if (matched[i]) {
            found++;
        } else {
            formatRom(TEMP_STRING_ROMS[i], text);
            LOG_WARN("TempString: probe %u (%s) not found", i + 1, text);
        }
    }
    LOG_INFO("TempString: %u/%u listed probes found on pin %u (%u devices on bus)", found, TEMP_STRING_COUNT, TEMP_STRING_PIN,
             total);
}

float TempStringModule::readProbe(const uint8_t *rom)
{
    for (uint8_t attempt = 0; attempt < READ_ATTEMPTS; attempt++) {
        if (!bus.reset())
            continue;
        bus.select(rom);
        bus.write(CMD_READ_SCRATCHPAD);
        uint8_t d[9];
        bus.read_bytes(d, sizeof(d));

        // A missing probe reads all ones and a shorted line all zeros; the config byte rules both out.
        if (OneWire::crc8(d, 8) != d[8] || (d[4] & 0x9F) != 0x1F)
            continue;

        int16_t raw = (int16_t)((d[1] << 8) | d[0]);
        const uint8_t resolutionBits = 9 + ((d[4] >> 5) & 0x03);
        raw &= ~((1 << (12 - resolutionBits)) - 1); // low bits are undefined below 12-bit resolution
        const float celsius = raw / 16.0f;

        // 85.0 C is the power-on value: this probe never ran its conversion.
        if (celsius < -55.0f || celsius >= 85.0f)
            continue;
        return celsius;
    }
    return NAN;
}

void TempStringModule::simulate()
{
    for (uint8_t i = 0; i < TEMP_STRING_COUNT; i++)
        temps[i] = 21.0f - 0.35f * i + (float)random(-50, 51) / 1000.0f;
    valid = TEMP_STRING_COUNT;
}

void TempStringModule::sendReport()
{
    char json[meshtastic_Constants_DATA_PAYLOAD_LEN + 1];
    swarm::JsonWriter j(json, sizeof(json));
    j.integer("node_id", nodeDB->getNodeNum());
    j.str("type", "ts");
    j.integer("seq", seq);
    uint32_t now = getValidTime(RTCQualityDevice);
    if (now)
        j.integer("t", now);
    if (SONDE_SIMULATE)
        j.integer("sim", 1);
    j.numberArray("ts", temps, TEMP_STRING_COUNT, 2);
    swarm::addPosition(j);
    size_t len = j.finish();
    if (len == 0) {
        LOG_ERROR("TempString: report exceeds %d bytes, dropped", meshtastic_Constants_DATA_PAYLOAD_LEN);
        return;
    }

    LOG_INFO("TempString: %u/%u probes read: %s", valid, TEMP_STRING_COUNT, json);
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = NODENUM_BROADCAST;
    p->want_ack = false;
    p->decoded.payload.size = len;
    memcpy(p->decoded.payload.bytes, json, len);
    // ccToPhone so a receiver plugged into this node over USB also gets its own readings.
    service->sendToMesh(p, RX_SRC_LOCAL, true);
    seq++;
}

int32_t TempStringModule::runOnce()
{
    switch (step) {
    case Step::Boot:
        LOG_INFO("TempString: %u DS18B20 probes on 1-Wire pin %u, every %us%s", TEMP_STRING_COUNT, TEMP_STRING_PIN,
                 SONDE_INTERVAL_SECS, SONDE_SIMULATE ? " (SIMULATED)" : "");
        if (!SONDE_SIMULATE)
            discover();
        step = Step::Convert;
        return FIRST_CYCLE_DELAY_MS;

    case Step::Convert:
        index = 0;
        valid = 0;
        for (float &t : temps)
            t = NAN;
        if (SONDE_SIMULATE) {
            simulate();
            step = Step::Report;
            return STEP_MS;
        }
        if (!bus.reset()) {
            if (!busWarned || !Throttle::isWithinTimespanMs(lastBusWarnMs, BUS_WARN_INTERVAL_MS)) {
                LOG_WARN("TempString: no presence pulse on pin %u; skipping this cycle", TEMP_STRING_PIN);
                busWarned = true;
                lastBusWarnMs = millis();
            }
            return swarm::msUntilNextSlot();
        }
        // Start every probe at once; power=1 holds the line high so parasite-powered probes can convert too.
        bus.skip();
        bus.write(CMD_CONVERT_T, 1);
        step = Step::Read;
        return CONVERSION_MS;

    case Step::Read:
        if (index == 0)
            bus.depower();
        temps[index] = readProbe(TEMP_STRING_ROMS[index]);
        if (!isnan(temps[index]))
            valid++;
        if (++index < TEMP_STRING_COUNT)
            return STEP_MS;
        step = Step::Report;
        return STEP_MS;

    case Step::Report:
        if (valid)
            sendReport();
        else
            LOG_WARN("TempString: no probe returned a valid reading this cycle");
        step = Step::Convert;
        return swarm::msUntilNextSlot();
    }
    return STEP_MS;
}

#endif

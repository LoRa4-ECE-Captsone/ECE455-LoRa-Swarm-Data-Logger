#include "GnssI2CModule.h"

#if defined(HAS_GNSS_I2C) && !MESHTASTIC_EXCLUDE_GPS

#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Throttle.h"
#include "modules/PositionModule.h"
#include <Wire.h>
#include <algorithm>
#include <string.h>

GnssI2CModule *gnssI2CModule;

static constexpr uint8_t UBX_REG_BYTES_AVAILABLE = 0xFD;
static constexpr size_t I2C_CHUNK = 32;
static constexpr size_t MAX_BYTES_PER_RUN = 1024;
static constexpr int32_t POLL_INTERVAL_MS = 1000;
static constexpr int32_t PROBE_RETRY_MS = 30 * 1000;
static constexpr uint32_t PUBLISH_INTERVAL_MS = 30 * 1000;
static constexpr uint32_t CLOCK_SET_INTERVAL_MS = 10 * 60 * 1000;
static constexpr uint32_t FIX_MAX_AGE_MS = 5000;
static constexpr uint8_t MAX_BUS_ERRORS = 5;
static constexpr uint32_t SEARCH_LOG_INTERVAL_MS = 2 * 60 * 1000;
static constexpr uint32_t CONFIG_ACK_WAIT_MS = 3000;
static constexpr uint8_t MAX_CONFIG_TRIES = 3;

// NMEA output on the I2C (DDC) port: keep GGA (fix, satellites, HDOP, altitude) and RMC (date, speed, course) that
// TinyGPS++ needs; turn off GLL/GSA/GSV/VTG, which add ~600 bytes/s of satellite chatter the poll would fall behind on.
static const uint8_t NMEA_RATES[][2] = {{0x00, 1}, {0x04, 1}, {0x01, 0}, {0x02, 0}, {0x03, 0}, {0x05, 0}};
static constexpr uint8_t NMEA_RATE_COUNT = sizeof(NMEA_RATES) / sizeof(NMEA_RATES[0]);
// First 8 bytes of UBX-ACK-ACK / UBX-ACK-NAK for a UBX-CFG-MSG (0x06 0x01) command.
static constexpr uint64_t UBX_ACK_CFG_MSG = 0xB562050102000601ULL;
static constexpr uint64_t UBX_NAK_CFG_MSG = 0xB562050002000601ULL;

static int32_t toDegInt(const RawDegrees &d)
{
    int32_t r = d.deg * 10000000 + d.billionths / 100;
    return d.negative ? -r : r;
}

GnssI2CModule::GnssI2CModule() : concurrency::OSThread("GnssI2C")
{
#ifdef PIN_3V3_EN
    // The RAK12500 runs from the switched 3V3_S rail. initVariant() already enables it; keep it on explicitly.
    pinMode(PIN_3V3_EN, OUTPUT);
    digitalWrite(PIN_3V3_EN, HIGH);
#endif
}

bool GnssI2CModule::probe()
{
    Wire.beginTransmission(GNSS_I2C_ADDR);
    return Wire.endTransmission() == 0;
}

bool GnssI2CModule::sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[6 + 8 + 2];
    if (len > 8)
        return false;
    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = len & 0xFF;
    frame[5] = len >> 8;
    memcpy(frame + 6, payload, len);
    uint8_t ckA = 0, ckB = 0;
    for (uint16_t i = 2; i < 6 + len; i++) {
        ckA += frame[i];
        ckB += ckA;
    }
    frame[6 + len] = ckA;
    frame[7 + len] = ckB;

    // u-blox treats a multi-byte I2C write as message data (a single byte would set its register pointer).
    Wire.beginTransmission(GNSS_I2C_ADDR);
    Wire.write(frame, 8 + len);
    return Wire.endTransmission() == 0;
}

void GnssI2CModule::configure()
{
    ubxAcks = 0;
    ubxNaks = 0;
    uint8_t sent = 0;
    for (uint8_t i = 0; i < NMEA_RATE_COUNT; i++) {
        // 3-byte UBX-CFG-MSG sets the rate on the port the command arrives on, i.e. this I2C port.
        const uint8_t payload[3] = {0xF0, NMEA_RATES[i][0], NMEA_RATES[i][1]};
        if (sendUbx(0x06, 0x01, payload, sizeof(payload)))
            sent++;
        delay(5);
    }
    configTries++;
    configPending = true;
    configSentMs = millis();
    LOG_DEBUG("GNSS: sent %u/%u NMEA output settings (try %u)", sent, NMEA_RATE_COUNT, configTries);
}

void GnssI2CModule::checkConfigured()
{
    if (!configPending || Throttle::isWithinTimespanMs(configSentMs, CONFIG_ACK_WAIT_MS))
        return;
    configPending = false;
    if (ubxAcks >= NMEA_RATE_COUNT) {
        LOG_INFO("GNSS: u-blox configured over I2C (%u/%u settings acknowledged)", ubxAcks, NMEA_RATE_COUNT);
    } else if (configTries < MAX_CONFIG_TRIES) {
        LOG_WARN("GNSS: only %u/%u settings acknowledged (%u rejected), retrying", ubxAcks, NMEA_RATE_COUNT, ubxNaks);
        configure();
    } else {
        LOG_WARN("GNSS: u-blox did not acknowledge its configuration; using its default NMEA output");
    }
}

void GnssI2CModule::enableInNodeConfig()
{
    // Meshtastic only detects UART GPS modules, so on this board it marks the GPS "not present" at first boot.
    // Now that the receiver has answered on I2C, record it as enabled (the app then shows GPS on).
    if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT) {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        LOG_INFO("GNSS: GPS mode set to ENABLED in the node config");
    }
}

int32_t GnssI2CModule::bytesAvailable()
{
    Wire.beginTransmission(GNSS_I2C_ADDR);
    Wire.write(UBX_REG_BYTES_AVAILABLE);
    if (Wire.endTransmission(false) != 0)
        return -1;
    if (Wire.requestFrom((uint8_t)GNSS_I2C_ADDR, (uint8_t)2) != 2)
        return -1;
    uint16_t count = (uint16_t)Wire.read() << 8;
    count |= (uint16_t)Wire.read();
    return count == 0xFFFF ? 0 : count;
}

bool GnssI2CModule::drain()
{
    int32_t available = bytesAvailable();
    if (available < 0)
        return false;

    size_t remaining = std::min((size_t)available, MAX_BYTES_PER_RUN);
    while (remaining > 0) {
        uint8_t want = (uint8_t)std::min(remaining, I2C_CHUNK);
        uint8_t got = Wire.requestFrom((uint8_t)GNSS_I2C_ADDR, want);
        if (got == 0)
            return false;
        while (Wire.available()) {
            const uint8_t b = Wire.read();
            reader.encode(b);
            ubxWindow = (ubxWindow << 8) | b;
            if (ubxWindow == UBX_ACK_CFG_MSG)
                ubxAcks++;
            else if (ubxWindow == UBX_NAK_CFG_MSG)
                ubxNaks++;
        }
        remaining -= std::min(remaining, (size_t)got);
    }
    return true;
}

bool GnssI2CModule::hasFreshLock()
{
    uint8_t quality = reader.fixQuality();
    return quality >= 1 && quality <= 5 && reader.location.isValid() && reader.location.age() < FIX_MAX_AGE_MS &&
           reader.hdop.isValid() && reader.hdop.value() > 0;
}

void GnssI2CModule::maybeSetClock()
{
    if (clockSetOnce && Throttle::isWithinTimespanMs(lastClockSetMs, CLOCK_SET_INTERVAL_MS))
        return;
    if (!reader.time.isValid() || !reader.date.isValid() || reader.date.year() < 2024)
        return;

    struct tm t = {};
    t.tm_sec = reader.time.second();
    t.tm_min = reader.time.minute();
    t.tm_hour = reader.time.hour();
    t.tm_mday = reader.date.day();
    t.tm_mon = reader.date.month() - 1;
    t.tm_year = reader.date.year() - 1900;
    if (perhapsSetRTC(RTCQualityGPS, t) == RTCSetResultSuccess) {
        clockSetOnce = true;
        lastClockSetMs = millis();
    }
}

void GnssI2CModule::publish()
{
    meshtastic_Position p = meshtastic_Position_init_default;
    const RawDegrees lat = reader.location.rawLat();
    const RawDegrees lng = reader.location.rawLng();

    p.location_source = meshtastic_Position_LocSource_LOC_INTERNAL;
    p.has_latitude_i = true;
    p.latitude_i = toDegInt(lat);
    p.has_longitude_i = true;
    p.longitude_i = toDegInt(lng);

    if (reader.altitude.isValid()) {
        p.has_altitude = true;
        p.altitude = (int32_t)reader.altitude.meters();
        if (reader.geoidHeight.isValid()) {
            p.has_altitude_geoidal_separation = true;
            p.altitude_geoidal_separation = (int32_t)reader.geoidHeight.meters();
            p.has_altitude_hae = true;
            p.altitude_hae = p.altitude + p.altitude_geoidal_separation;
        }
    }

    // TinyGPS reports HDOP in hundredths; PDOP is approximated the same way GPS.cpp does without GSA parsing.
    p.HDOP = reader.hdop.value();
    p.PDOP = 1.41 * reader.hdop.value();
    p.fix_quality = reader.fixQuality();
    if (reader.satellites.isValid())
        p.sats_in_view = reader.satellites.value();
    if (reader.course.isValid() && reader.course.value() < 36000) {
        p.has_ground_track = true;
        p.ground_track = reader.course.value() * 1e3;
    }
    if (reader.speed.isValid()) {
        p.has_ground_speed = true;
        p.ground_speed = reader.speed.kmph();
    }

    if (reader.date.isValid() && reader.time.isValid()) {
        struct tm t = {};
        t.tm_sec = reader.time.second();
        t.tm_min = reader.time.minute();
        t.tm_hour = reader.time.hour();
        t.tm_mday = reader.date.day();
        t.tm_mon = reader.date.month() - 1;
        t.tm_year = reader.date.year() - 1900;
        p.timestamp = gm_mktime(&t);
    }
    p.time = getValidTime(RTCQualityFromNet);

    LOG_DEBUG("GNSS: publish lat=%d lon=%d sats=%u hdop=%u", p.latitude_i, p.longitude_i, p.sats_in_view, p.HDOP);
    nodeDB->updatePosition(nodeDB->getNodeNum(), p, RX_SRC_LOCAL);
    if (positionModule)
        positionModule->handleNewPosition();
}

int32_t GnssI2CModule::runOnce()
{
    if (!present) {
        present = probe();
        if (!present) {
            LOG_DEBUG("GNSS: no u-blox at I2C 0x%02x, retry in %ds", GNSS_I2C_ADDR, PROBE_RETRY_MS / 1000);
            return PROBE_RETRY_MS;
        }
        LOG_INFO("GNSS: u-blox found at I2C 0x%02x", GNSS_I2C_ADDR);
        busErrors = 0;
        configTries = 0;
        enableInNodeConfig();
        configure();
    }

    if (!drain()) {
        if (++busErrors >= MAX_BUS_ERRORS) {
            LOG_WARN("GNSS: %u consecutive I2C failures, re-probing", busErrors);
            present = false;
            fixValid = false;
            return PROBE_RETRY_MS;
        }
        return POLL_INTERVAL_MS;
    }
    busErrors = 0;

    checkConfigured();
    maybeSetClock();

    bool lock = hasFreshLock();
    if (lock != fixValid) {
        LOG_INFO("GNSS: fix %s (sats=%u)", lock ? "acquired" : "lost",
                 reader.satellites.isValid() ? reader.satellites.value() : 0);
        fixValid = lock;
    }
    if (!lock && !Throttle::isWithinTimespanMs(lastSearchLogMs, SEARCH_LOG_INTERVAL_MS)) {
        LOG_INFO("GNSS: receiver running, no fix yet (time %s, needs open sky)", reader.time.isValid() ? "known" : "not known");
        lastSearchLogMs = millis();
    }

    // Respect "GPS disabled" from the app: keep draining so the receiver's buffer doesn't back up, but don't publish.
    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED) {
        if (!disabledLogged)
            LOG_INFO("GNSS: GPS is disabled in the node config; not publishing positions");
        disabledLogged = true;
        return POLL_INTERVAL_MS;
    }
    disabledLogged = false;

    if (lock && (!publishedOnce || !Throttle::isWithinTimespanMs(lastPublishMs, PUBLISH_INTERVAL_MS))) {
        publish();
        publishedOnce = true;
        lastPublishMs = millis();
    }
    return POLL_INTERVAL_MS;
}

#endif

#include "SondeModule.h"

#ifdef HAS_SONDE

#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "SwarmReport.h"
#include "Throttle.h"
#include <math.h>
#include <string.h>

using swarm::JsonWriter;
using swarm::usable;

SondeModule *sondeModule;

#ifndef SONDE_BRUSH
#define SONDE_BRUSH 1
#endif
// 0 = no weather station on the bus.
#ifndef WX_MODBUS_ADDR
#define WX_MODBUS_ADDR 0
#endif

static constexpr int32_t BRUSH_DELAY_MS = 2000;    // Y4000 answers ~1 s after power-up
static constexpr int32_t STABILIZE_MS = 50 * 1000; // EnviroDIY ModularSensors Y4000 stabilisation time
static_assert((int32_t)SENSOR_WARMUP_SECS * 1000 >= BRUSH_DELAY_MS + STABILIZE_MS,
              "SENSOR_WARMUP_SECS is shorter than the Y4000 needs to stabilise after its brush");
static constexpr uint32_t REPLY_TIMEOUT_MS = 1000;
static constexpr int32_t REPLY_POLL_MS = 50;
static constexpr int32_t RETRY_GAP_MS = 500;
static constexpr int32_t STEP_MS = 10;
static constexpr uint8_t MAX_ATTEMPTS = 3;
static constexpr uint32_t ERROR_REPORT_INTERVAL_MS = 5 * 60 * 1000;
// DFRobot asks for > 200 ms between SEN0657 polls; at 200 ms it mostly answered with a fixed "not ready" pattern.
static constexpr int32_t WX_GAP_MS = 1000;
static constexpr uint8_t WX_MAX_ATTEMPTS = 2;

static constexpr uint8_t FN_READ_HOLDING = 0x03;
static constexpr uint8_t FN_WRITE_MULTIPLE = 0x10;
// EnviroDIY YosemitechModbus: the sonde's 8 floats start at 0x2601 (the manual's 0x2600 is wrong).
static constexpr uint16_t REG_VALUES = 0x2601;
static constexpr uint16_t REG_STATUS = 0x0800;
static constexpr uint16_t REG_BRUSH = 0x2F00;
static constexpr size_t VALUES_REPLY_LEN = 5 + 8 * 4; // addr, fn, byte count, 8 floats, crc16
static constexpr size_t STATUS_REPLY_LEN = 5 + 2;

// DFRobot SEN0657: one block read 0x01F4..0x0201 covers every measurement (big-endian 16-bit registers).
// One register per request: on this unit at 9600 baud, multi-register reads usually come back as a constant
// "not ready" pattern (0x32D0, 0xF000, ...) while single-register reads are reliable.
static constexpr uint16_t WX_REGS[] = {
    0x01F4, // wind speed, m/s x10
    0x01F7, // wind direction, degrees (0 = north, clockwise)
    0x01F8, // humidity, %RH x10
    0x01F9, // temperature, degC x10 (signed)
    0x01FD, // pressure, 0.1 kPa = hPa
    0x01FE, // light, high word
    0x01FF, // light, low word
    0x0201, // rain, mm x10, accumulated since power-up or zeroing
};
static constexpr uint8_t WX_READ_COUNT = sizeof(WX_REGS) / sizeof(WX_REGS[0]);
// SEN0657 settings registers (DFRobot wiki): Modbus address 1-254 (factory 1) and baud code (factory 1 = 4800).
static constexpr uint16_t REG_WX_ADDRESS = 0x07D0;
static constexpr uint32_t WX_BAUD_CODES[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 1200};
static constexpr uint16_t NO_REPLY = 0xFFFF;

// Words this SEN0657 sends instead of data when a reading isn't ready (not in DFRobot's docs). Every register is
// checked because 0x32D0 as a light low word would otherwise pass as 13008 lux.
static bool wxNotReady(uint16_t raw)
{
    return raw == 0x32D0 || raw == 0xF000;
}

enum ValueIndex : uint8_t { DO_MGL, TURBIDITY_NTU, COND_MS_CM, PH, TEMP_C, ORP_MV, CHL_UG_L, BGA };

static uint16_t modbusCrc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

SondeModule::SondeModule() : SinglePortModule("sonde", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("Sonde") {}

void SondeModule::wake(uint32_t ms)
{
    enabled = true;
    setIntervalFromNow(ms);
}

void SondeModule::openBus(bool on)
{
    // The UART is only open while the sensors are powered so the UARTE doesn't hold HFCLK on between cycles.
    if (on)
        RS485_SERIAL.begin(RS485_BAUD);
    else
        RS485_SERIAL.end();
    powered = on;
}

void SondeModule::sensorsPowered(bool on)
{
    if (on == powered)
        return;
    if (!on && step != Step::Idle && step != Step::Boot && step != Step::Brush) {
        LOG_WARN("Sonde: sensor power went off mid-read; sending what was read");
        sendReport();
        step = Step::Idle;
        disable();
    }
    openBus(on);
    if (on && SONDE_BRUSH && !SONDE_SIMULATE) {
        step = Step::Brush;
        wake(BRUSH_DELAY_MS);
    }
}

void SondeModule::sampleNow()
{
    valuesOk = false;
    statusOk = false;
    weatherOk = false;
    weatherAnswered = false;
    weatherPart = 0;
    weatherValid = 0;
    luxHighOk = false;
    weather = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
    attempts = 0;
    if (SONDE_SIMULATE) {
        simulateValues();
        step = Step::Report;
    } else {
        step = Step::RequestValues;
    }
    wake(0);
}

void SondeModule::logWxConfig()
{
    const uint16_t addr = wxConfig[0], code = wxConfig[1];
    const uint32_t baud = code < sizeof(WX_BAUD_CODES) / sizeof(WX_BAUD_CODES[0]) ? WX_BAUD_CODES[code] : 0;
    if (addr == NO_REPLY || code == NO_REPLY)
        LOG_WARN("SEN0657: answers at addr %u / %u baud but not on its settings registers 0x07D0/0x07D1", WX_MODBUS_ADDR,
                 RS485_BAUD);
    else if (addr == WX_MODBUS_ADDR && baud == RS485_BAUD)
        LOG_INFO("SEN0657: settings registers confirm Modbus addr %u at %u baud", addr, baud);
    else
        LOG_WARN("SEN0657: answers at addr %u / %u baud, but its settings registers read addr 0x%04x, baud code 0x%04x",
                 WX_MODBUS_ADDR, RS485_BAUD, addr, code);
}

void SondeModule::sendFrame(uint8_t *frame, size_t lenWithoutCrc)
{
    uint16_t crc = modbusCrc(frame, lenWithoutCrc);
    frame[lenWithoutCrc] = crc & 0xFF;
    frame[lenWithoutCrc + 1] = crc >> 8;

    while (RS485_SERIAL.available())
        RS485_SERIAL.read();
    txLen = lenWithoutCrc + 2;
    memcpy(txFrame, frame, txLen);
    echoPending = true;
    RS485_SERIAL.write(frame, txLen);
    RS485_SERIAL.flush();

    rxLen = 0;
    requestSentMs = millis();
}

void SondeModule::requestRegisters(uint8_t addr, uint16_t start, uint16_t count)
{
    uint8_t frame[8] = {addr, FN_READ_HOLDING, (uint8_t)(start >> 8), (uint8_t)start, (uint8_t)(count >> 8), (uint8_t)count};
    sendFrame(frame, 6);
}

void SondeModule::requestBrush()
{
    uint8_t frame[9] = {SONDE_MODBUS_ADDR, FN_WRITE_MULTIPLE, (uint8_t)(REG_BRUSH >> 8), (uint8_t)REG_BRUSH, 0x00, 0x00, 0x00};
    sendFrame(frame, 7);
}

SondeModule::Reply SondeModule::pollReply(uint8_t addr, size_t expectedLen)
{
    while (RS485_SERIAL.available() && rxLen < sizeof(rx))
        rx[rxLen++] = RS485_SERIAL.read();

    // If the transceiver echoes our own request, drop exactly that echo. Byte 2 always differs between a read
    // request (start address high byte) and its reply (byte count), so a real reply is never mistaken for one.
    if (echoPending) {
        size_t n = std::min(rxLen, txLen);
        if (memcmp(rx, txFrame, n) != 0) {
            echoPending = false;
        } else if (rxLen >= txLen) {
            memmove(rx, rx + txLen, rxLen - txLen);
            rxLen -= txLen;
            echoPending = false;
        } else if (Throttle::isWithinTimespanMs(requestSentMs, REPLY_TIMEOUT_MS)) {
            return Reply::Pending;
        }
    }

    // Resync on the slave address in case the transceiver produced noise at power-up.
    size_t skip = 0;
    while (skip < rxLen && rx[skip] != addr)
        skip++;
    if (skip) {
        memmove(rx, rx + skip, rxLen - skip);
        rxLen -= skip;
    }

    if (rxLen >= 5 && (rx[1] & 0x80)) {
        LOG_WARN("RS485: Modbus exception 0x%02x from addr %u", rx[2], addr);
        return Reply::Failed;
    }
    if (rxLen >= expectedLen) {
        uint16_t crc = modbusCrc(rx, expectedLen - 2);
        bool crcOk = rx[expectedLen - 2] == (crc & 0xFF) && rx[expectedLen - 1] == (crc >> 8);
        bool shapeOk = rx[1] == FN_READ_HOLDING && rx[2] == expectedLen - 5;
        if (crcOk && shapeOk)
            return Reply::Ok;
        LOG_WARN("RS485: bad reply from addr %u (crc ok=%d, shape ok=%d, %u bytes)", addr, crcOk, shapeOk, rxLen);
        return Reply::Failed;
    }
    if (Throttle::isWithinTimespanMs(requestSentMs, REPLY_TIMEOUT_MS))
        return Reply::Pending;

    LOG_WARN("RS485: no reply from Modbus addr %u (%u bytes)", addr, rxLen);
    return Reply::Failed;
}

void SondeModule::decodeValues()
{
    // Floats arrive little-endian, the same byte order as the nRF52.
    for (uint8_t i = 0; i < VALUE_COUNT; i++)
        memcpy(&values[i], &rx[3 + 4 * i], sizeof(float));
    valuesOk = true;
}

void SondeModule::decodeWeather()
{
    const uint16_t raw = (uint16_t)((rx[3] << 8) | rx[4]);
    LOG_DEBUG("RS485: weather reg 0x%04x = 0x%04x (%u)", WX_REGS[weatherPart], raw, raw);
    if (wxNotReady(raw)) {
        if (weatherPart == 5)
            luxHighOk = false;
        return;
    }
    // Physically impossible values are dropped (left NaN) so a stray "not ready" word never reaches the mesh.
    auto keep = [this](float &field, float value, float low, float high) {
        if (value >= low && value <= high) {
            field = value;
            weatherValid++;
        }
    };
    switch (weatherPart) {
    case 0:
        keep(weather.windSpeed, raw / 10.0f, 0, 75);
        break;
    case 1:
        keep(weather.windDir, raw, 0, 360);
        break;
    case 2:
        keep(weather.humidity, raw / 10.0f, 0, 100);
        break;
    case 3:
        keep(weather.airTemp, (int16_t)raw / 10.0f, -50, 70);
        break;
    case 4:
        keep(weather.pressureHpa, raw, 300, 1100);
        break;
    case 5:
        luxHigh = raw;
        luxHighOk = true;
        return;
    case 6:
        if (luxHighOk)
            keep(weather.lux, (float)(((uint32_t)luxHigh << 16) | raw), 0, 250000);
        break;
    case 7:
        keep(weather.rainMm, raw / 10.0f, 0, 1000);
        break;
    }
    weatherOk = weatherOk || !isnan(weather.windSpeed) || !isnan(weather.windDir) || !isnan(weather.humidity) ||
                !isnan(weather.airTemp) || !isnan(weather.pressureHpa) || !isnan(weather.lux) || !isnan(weather.rainMm);
}

void SondeModule::simulateValues()
{
    auto jitter = [](float base, float span) { return base + span * (float)random(-1000, 1001) / 1000.0f; };
    values[DO_MGL] = jitter(8.0f, 0.5f);
    values[TURBIDITY_NTU] = jitter(2.2f, 0.3f);
    values[COND_MS_CM] = jitter(0.41f, 0.01f);
    values[PH] = jitter(7.2f, 0.15f);
    values[TEMP_C] = jitter(19.0f, 1.0f);
    values[ORP_MV] = jitter(210.0f, 20.0f);
    values[CHL_UG_L] = jitter(1.5f, 0.5f);
    values[BGA] = jitter(300.0f, 100.0f);
    valuesOk = true;
    if (WX_MODBUS_ADDR) {
        weather = {jitter(3.0f, 2.0f),        jitter(180.0f, 90.0f), jitter(15.0f, 2.0f),
                   jitter(60.0f, 10.0f),      jitter(1013.0f, 5.0f), 0.0f,
                   jitter(20000.0f, 10000.0f)};
        weatherOk = true;
    }
}

size_t SondeModule::buildJson(char *buf, size_t cap, bool compact)
{
    JsonWriter j(buf, cap);
    j.integer("node_id", nodeDB->getNodeNum());
    j.integer("seq", seq);
    uint32_t now = getValidTime(RTCQualityDevice);
    if (now)
        j.integer("t", now);
    if (SONDE_SIMULATE)
        j.integer("sim", 1);

    if (valuesOk) {
        j.number("water_temp", values[TEMP_C], 2);
        j.number("ph", values[PH], 2);
        // The dashboard works in µS/cm; the Y4000 reports mS/cm.
        if (usable(values[COND_MS_CM]))
            j.number("conductivity", values[COND_MS_CM] * 1000.0f, 1);
        j.number("dissolved_oxygen", values[DO_MGL], 2);
        j.number("turbidity", values[TURBIDITY_NTU], 2);
        if (!compact) {
            j.number("orp", values[ORP_MV], 1);
            j.number("chl", values[CHL_UG_L], 2);
            j.number("bga", values[BGA], 0);
            if (statusOk)
                j.integer("status", status);
        }
    } else {
        j.str("err", "no_reply");
    }

    swarm::addPosition(j);
    return j.finish();
}

size_t SondeModule::buildWeatherJson(char *buf, size_t cap)
{
    JsonWriter j(buf, cap);
    j.integer("node_id", nodeDB->getNodeNum());
    j.str("type", "wx");
    j.integer("seq", weatherSeq);
    uint32_t now = getValidTime(RTCQualityDevice);
    if (now)
        j.integer("t", now);
    if (SONDE_SIMULATE)
        j.integer("sim", 1);
    j.number("wind_speed", weather.windSpeed, 1);
    j.number("wind_dir", weather.windDir, 0);
    j.number("air_temp", weather.airTemp, 1);
    j.number("humidity", weather.humidity, 1);
    j.number("pressure", weather.pressureHpa, 0);
    j.number("lux", weather.lux, 0);
    j.number("rain", weather.rainMm, 1);
    swarm::addPosition(j);
    return j.finish();
}

void SondeModule::sendJson(const char *json, size_t len)
{
    LOG_INFO("RS485: %s", json);
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = NODENUM_BROADCAST;
    p->want_ack = false;
    p->decoded.payload.size = len;
    memcpy(p->decoded.payload.bytes, json, len);
    // ccToPhone so a receiver plugged into this node over USB also gets its own readings.
    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

void SondeModule::sendReport()
{
    char json[meshtastic_Constants_DATA_PAYLOAD_LEN + 1];

    // A node with no sonde attached (e.g. a relay) shouldn't flood the mesh with error reports.
    if (valuesOk || !errorReported || !Throttle::isWithinTimespanMs(lastErrorReportMs, ERROR_REPORT_INTERVAL_MS)) {
        size_t len = buildJson(json, sizeof(json), false);
        if (len == 0)
            len = buildJson(json, sizeof(json), true);
        if (len) {
            sendJson(json, len);
            seq++;
        } else {
            LOG_ERROR("Sonde: report exceeds %d bytes, dropped", meshtastic_Constants_DATA_PAYLOAD_LEN);
        }
        errorReported = !valuesOk;
        lastErrorReportMs = millis();
    } else {
        LOG_DEBUG("Sonde: still no reply, error report suppressed");
    }

    if (weatherOk) {
        size_t len = buildWeatherJson(json, sizeof(json));
        if (len) {
            sendJson(json, len);
            weatherSeq++;
        }
    }
}

int32_t SondeModule::runOnce()
{
    switch (step) {
    case Step::Boot:
        LOG_INFO("RS485: %u baud 8N1; Y4000 sonde at Modbus addr %u, SEN0657 weather station at addr %u (0 = none)%s",
                 RS485_BAUD, SONDE_MODBUS_ADDR, WX_MODBUS_ADDR, SONDE_SIMULATE ? " (SIMULATED)" : "");
        step = Step::Idle;
        return disable();

    case Step::Idle:
        return disable();

    case Step::Brush:
        requestBrush();
        step = Step::Idle;
        return disable();

    case Step::RequestValues:
        requestRegisters(SONDE_MODBUS_ADDR, REG_VALUES, VALUE_COUNT * 2);
        step = Step::AwaitValues;
        return REPLY_POLL_MS;

    case Step::AwaitValues:
        switch (pollReply(SONDE_MODBUS_ADDR, VALUES_REPLY_LEN)) {
        case Reply::Pending:
            return REPLY_POLL_MS;
        case Reply::Ok:
            decodeValues();
            step = Step::RequestStatus;
            return STEP_MS;
        case Reply::Failed:
            if (++attempts < MAX_ATTEMPTS) {
                step = Step::RequestValues;
                return RETRY_GAP_MS;
            }
            attempts = 0;
            step = WX_MODBUS_ADDR ? Step::RequestWeather : Step::Report;
            return WX_MODBUS_ADDR ? WX_GAP_MS : STEP_MS;
        }
        break;

    case Step::RequestStatus:
        requestRegisters(SONDE_MODBUS_ADDR, REG_STATUS, 1);
        step = Step::AwaitStatus;
        return REPLY_POLL_MS;

    case Step::AwaitStatus: {
        Reply r = pollReply(SONDE_MODBUS_ADDR, STATUS_REPLY_LEN);
        if (r == Reply::Pending)
            return REPLY_POLL_MS;
        if (r == Reply::Ok) {
            status = rx[3];
            statusOk = true;
        }
        attempts = 0;
        step = WX_MODBUS_ADDR ? Step::RequestWeather : Step::Report;
        return WX_MODBUS_ADDR ? WX_GAP_MS : STEP_MS;
    }

    case Step::RequestWeather:
        requestRegisters(WX_MODBUS_ADDR, WX_REGS[weatherPart], 1);
        step = Step::AwaitWeather;
        return REPLY_POLL_MS;

    case Step::AwaitWeather: {
        Reply r = pollReply(WX_MODBUS_ADDR, 5 + 2);
        if (r == Reply::Pending)
            return REPLY_POLL_MS;
        if (r == Reply::Failed && ++attempts < WX_MAX_ATTEMPTS) {
            step = Step::RequestWeather;
            return WX_GAP_MS;
        }
        if (r == Reply::Ok) {
            weatherAnswered = true;
            decodeWeather();
        }
        attempts = 0;
        // A part that never answered leaves its fields NaN, which the JSON writer omits.
        if (++weatherPart < WX_READ_COUNT) {
            step = Step::RequestWeather;
            return WX_GAP_MS;
        }
        LOG_INFO("Weather: %u/7 values valid from Modbus addr %u", weatherValid, WX_MODBUS_ADDR);
        if (!weatherAnswered) {
            LOG_WARN("SEN0657: no reply at Modbus addr %u, %u baud. Factory default is addr 1 at 4800 baud: set it to addr %u, "
                     "baud code 2 (9600) via registers 0x07D0/0x07D1",
                     WX_MODBUS_ADDR, RS485_BAUD, WX_MODBUS_ADDR);
        } else if (!wxConfigChecked) {
            wxConfigPart = 0;
            step = Step::RequestWxConfig;
            return WX_GAP_MS;
        }
        step = Step::Report;
        return STEP_MS;
    }

    case Step::RequestWxConfig:
        requestRegisters(WX_MODBUS_ADDR, REG_WX_ADDRESS + wxConfigPart, 1);
        step = Step::AwaitWxConfig;
        return REPLY_POLL_MS;

    case Step::AwaitWxConfig: {
        Reply r = pollReply(WX_MODBUS_ADDR, 5 + 2);
        if (r == Reply::Pending)
            return REPLY_POLL_MS;
        wxConfig[wxConfigPart] = r == Reply::Ok ? (uint16_t)((rx[3] << 8) | rx[4]) : NO_REPLY;
        if (++wxConfigPart < 2) {
            step = Step::RequestWxConfig;
            return WX_GAP_MS;
        }
        logWxConfig();
        wxConfigChecked = true;
        step = Step::Report;
        return STEP_MS;
    }

    case Step::Report:
        sendReport();
        step = Step::Idle;
        return disable();
    }
    return STEP_MS;
}

#endif

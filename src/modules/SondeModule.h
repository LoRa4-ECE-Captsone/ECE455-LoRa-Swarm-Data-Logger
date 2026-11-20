#pragma once

#include "configuration.h"

#ifdef HAS_SONDE

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"

/**
 * Owns the RS485 bus (RAK5802 on RS485_SERIAL) and polls every Modbus RTU device on it in turn, so they never
 * talk over each other: the Yosemitech Y4000 sonde (SONDE_MODBUS_ADDR) and, if fitted, the DFRobot SEN0657
 * 7-in-1 weather station (WX_MODBUS_ADDR).
 * SensorPower decides when: the bus opens and the sonde is brushed when the sensor supply comes on, and the readings
 * start once the sensors have warmed up. Each read is a non-blocking state machine so the mesh keeps running:
 * read sonde values + status -> read weather (and, once per boot, the station's address/baud registers) -> send JSON
 * on PRIVATE_APP (one sonde packet, one "type":"wx" packet). The JSON keys match ui/receiver.py.
 */
class SondeModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    SondeModule();

    /// Called by SensorPower when the sensor supply switches.
    void sensorsPowered(bool on);
    /// Called by SensorPower once the sensors have warmed up: take and send this cycle's readings.
    void sampleNow();

  protected:
    int32_t runOnce() override;

  private:
    enum class Step : uint8_t {
        Boot,
        Idle,
        Brush,
        RequestValues,
        AwaitValues,
        RequestStatus,
        AwaitStatus,
        RequestWeather,
        AwaitWeather,
        RequestWxConfig,
        AwaitWxConfig,
        Report
    };
    enum class Reply : uint8_t { Pending, Ok, Failed };

    // Y4000 register order: DO, turbidity, conductivity, pH, temperature, ORP, chlorophyll, BGA.
    static constexpr uint8_t VALUE_COUNT = 8;

    struct Weather {
        float windSpeed;
        float windDir;
        float airTemp;
        float humidity;
        float pressureHpa;
        float rainMm;
        float lux;
    };

    Step step = Step::Boot;
    float values[VALUE_COUNT] = {};
    Weather weather = {};
    bool powered = false;
    bool errorReported = false;
    uint32_t lastErrorReportMs = 0;
    bool valuesOk = false;
    bool statusOk = false;
    bool weatherOk = false;
    uint8_t status = 0;
    uint8_t attempts = 0;
    uint8_t weatherPart = 0;
    uint16_t luxHigh = 0;
    bool luxHighOk = false;
    uint16_t seq = 0;
    uint16_t weatherSeq = 0;
    uint32_t requestSentMs = 0;
    uint8_t rx[48] = {};
    size_t rxLen = 0;
    uint8_t txFrame[16] = {};
    size_t txLen = 0;
    bool echoPending = false;
    uint8_t weatherValid = 0;
    bool weatherAnswered = false;
    bool wxConfigChecked = false;
    uint8_t wxConfigPart = 0;
    uint16_t wxConfig[2] = {};

    void wake(uint32_t ms);
    void openBus(bool on);
    void sendFrame(uint8_t *frame, size_t lenWithoutCrc);
    void requestRegisters(uint8_t addr, uint16_t start, uint16_t count);
    void requestBrush();
    Reply pollReply(uint8_t addr, size_t expectedLen);
    void decodeValues();
    void decodeWeather();
    void simulateValues();
    size_t buildJson(char *buf, size_t cap, bool compact);
    size_t buildWeatherJson(char *buf, size_t cap);
    void sendJson(const char *json, size_t len);
    void sendReport();
    void logWxConfig();
};

extern SondeModule *sondeModule;

#endif

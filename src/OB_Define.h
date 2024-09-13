#ifndef OB_DEFINE_H_
#define OB_DEFINE_H_

#include <Arduino.h>
#include <ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ezTime.h>

#define DEBUG_ON 1

/* Debug messages */
#if DEBUG_ON == 1
#define debug(x) Serial.print(x)
#define debugBase(x, y) Serial.print(x, y)
#define debugln(x) Serial.println(x)
#define debuglnBase(x, y) Serial.println(x, y)
// #define debugf(x, ...) Serial.printf(x, ...)
#else
#define debug(x)
#define debugBase(x, y)
#define debugln(x)
#define debuglnBase(x, y)
#endif

/*Constants FOR ONE-BOARD CONTROLLER v1*/
// PINS
// Inputs v2.0
#define D_IN_1 3
#define D_IN_2 9
#define D_IN_3 10
#define D_IN_4 11
#define D_IN_5 12
#define D_IN_6 13
#define D_IN_7 14
#define D_IN_8 21
#define TEMP 2

// vvv Version 1 vvv
// #define D_IN_1 1
// #define D_IN_2 2
// #define D_IN_3 38
// #define D_IN_4 37
// #define D_IN_5 36
// #define D_IN_6 35
// #define D_IN_7 47
// #define D_IN_8 21
// #define TEMP 10

// Outputs (same as version 1.0 except LED)
#define D_OUT_1 4
#define D_OUT_2 5
#define D_OUT_3 6
#define D_OUT_4 7
#define D_OUT_5 15
#define D_OUT_6 16
#define D_OUT_7 17
#define D_OUT_8 18

#define OUTPUT_MAX 19 // highest output pin number plus 1  
// #define STATUS_LED 48 -- version 1.0
#define STATUS_LED 1

// Serial For Display
#define HMI_TX_PIN 38
#define HMI_RX_PIN 36
// vvv Version 1 vvv
// #define TX_PIN 14
// #define RX_PIN 13

// Serial For Modbus
#define MB_TX_PIN 37
#define MB_RX_PIN 35
// vvv Version 1 vvv
// #define MB_TX_PIN 12
// #define MB_RX_PIN 11

// I2C
#define S_DATA 48
#define S_CLK 47 
// vvv Version 1 vvv
// #define S_DATA 8
// #define S_CLK 9

#define DEF_TANK_LEVEL_SENSOR_MAX 5000
extern Timezone local;
extern bool OutputStatus[];

// Functions
void PinSetup(void);
// inline bool ReadInput(uint inputPin) { return !digitalRead(inputPin); } -- Version 0.1 
bool ReadInput(uint inputPin);
void WriteOutput(uint outputPin, bool val);
bool AnalogSetup(ADS1115 &AnalogDC);
inline float map_f(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
inline float map_4to20(float x, float out_min, float out_max) { return (x - 4.0F) * (out_max - out_min) / (16.0F) + out_min; }

class VolumeMeter
{
private:
    bool firstEntry=true;
    uint8_t _inputPin;
    bool _last_status;
    ulong _timeStamp = 0;
    const uint _MAX_PULSE_TIME = 30000;
    const float _L_PER_PULSE = 5;
    const float _M3_PER_PULSE = _L_PER_PULSE / 1000;
    float CalculateFlow(uint timeDiff);

public:
    float FlowRate = 0; // flow rate in lph
    uint Volume_L = 0;

    VolumeMeter(uint8_t inputPin, uint lpp = 5) : _L_PER_PULSE(lpp) { _inputPin = inputPin; }
    bool CheckVolumeMeter(bool calculateFlow = true);
};

class PulseMeter
{
private:
    uint8_t _inputPin;
    ulong _timeStamp = 0;
    // TODO below may need to be calibrated per size 4.8 is the calibration for small meter (1/2 inch)
    float _FLOW_FACTOR;

public:
    volatile uint Pulses = 0;
    float FlowRate = 0; // flow rate in lph TODO maybe change to uint
    double cummulativeFlow = 0;
    uint volumePulses;
    ulong Volume = 0;
    PulseMeter(uint8_t inputPin, const float Factor) : _FLOW_FACTOR(Factor) { _inputPin = inputPin; }
    void CalculateFlow();
};

class DigitalInput
{
private:
    uint _inputPin;
    ulong _debounceStamp;
    bool _last_status;
    ulong _delayStamp;
    bool _last_delay_status;

public:
    DigitalInput(uint pin)
    {
        _inputPin = pin;
        _last_status = ReadInput(pin);
        _last_delay_status = _last_status;
    }
    bool ReadInputDebounce(uint debounce_ms);
    bool ReadInputDelay(uint delay_ms, bool input);
};

class InputTimer
{
    ulong _delayStamp;
    bool _last_delay_status = false;

public:
    bool DelayTimer(uint delay_s, bool input);
    void ResetTimer(void) { _delayStamp = millis(); }
    void setStatus(bool status) { _last_delay_status = status; }
};

class TempSensor
{
private:
    static uint _indexCount;
    uint _index = 0;
    uint _inputPin;
    // const uint _readingPeriod = 550;    // minimum time between readings [ms]
    uint _readingPeriod;
    ulong _timeStamp;
    DeviceAddress _deviceAddress;

public:
    bool connected = false;
    float currTemp;
    TempSensor();
    bool ReadTemp(void);
};

class TankLevelSensor
{
private:
    uint tankEmpty_mm = 0;   // the height of tank
    uint tankFull_mm = 2000; // the height of the water in mm at 100%
    uint sensorMax;

public:
    enum tankState_t
    {
        T_EMPTY,
        T_LOW,
        T_MID,
        T_FULL,
        T_HIGH
    } tankState = T_MID;
    float Level = 0.0F; // level in mm
    float Level_p = 0;  // percentage x 10
    // const uint multPecentage = 10;
    const float TankMax_p = 110.0F;  // 110%
    const float TankHigh_p = 105.0F; // 105%
    const float TankFull_p = 100.0;  // 100%
    const float TankLow_p = 25.0;    // 25%
    const float TankEmpty_p = 1.0;   // 1%
    // bool alarm = 0;                                                     // set when low,high or empty and reset when full or mid
    TankLevelSensor(uint _sensorMax = DEF_TANK_LEVEL_SENSOR_MAX) { sensorMax = _sensorMax; } // Default at 5m
    bool ComputeTankLevel(float signal4to20mA);                                              // compute the tank level and return wheteher it is under 4ma or over 20ma
    void setTankEmpty(uint EMPTY) { tankEmpty_mm = EMPTY; }                                  // input height of empty tank in mm
    // returns if different from current value
    uint getTankFull(void) { return tankFull_mm; }
    bool setTankFull(uint FULL) // input height of full tank in mm
    {
        if (FULL != tankFull_mm)
        {
            tankFull_mm = FULL;
            return true;
        }
        return false;
    }
};

#endif // OB_DEFINE_H_
/*  NEXTION LIBRARY FOR GCG ONE BOARD - Specific to UK RO
 *
 *   Library uses a data structure to receive communication from the Nextion Controller.
 */
#ifndef NEXTION_GCG_H_
#define NEXTION_GCG_H_
#include <Arduino.h>
#include "OB_Define.h"

// // Initial settings  //TODO fill in values
// Defaults
#define DEF_FLUSH_TIME 60 // 1 minute

#define DEF_PF_MIN 2200 // SP = 2000 -- 15% = 300
#define DEF_PF_MAX 2600
#define DEF_PF_FAULT_MIN 2100 // SP = 2000 -- 15% = 300
#define DEF_PF_FAULT_MAX 2700
#define DEF_RF_MIN 2200 // SP = 4500 -- 15% = 675
#define DEF_RF_MAX 3600
#define DEF_BF_MIN 580 // SP = 500 -- 10% = 50
#define DEF_BF_MAX 800
#define DEF_BF_FAULT_MIN 500

#define DEF_EC_MAX 45
#define DEF_EC_FAULT_MAX 45

#define DEF_FP_FAULT 10
#define DEF_HP_FAULT 70
#define DEF_DP_WARNING 10
#define DEF_DP_FAULT 15

#define DEF_REC_MIN 750 // Recovery %
#define DEF_REC_MAX 850

// Status Pictures  // uint8_t[5] = {offPic, onPic, warningPic, faultPic, disconnectPic}
const uint8_t SensorBkgndPics[5] = {1, 2, 2, 2, 2}; // background (picc)
const uint8_t ValvePics[5] = {11, 12, 12, 12, 12};  // Off, On
const uint8_t FRP_Pics[5] = {13, 14, 14, 14, 14};   // Off, On
const uint8_t PumpPics[5] = {15, 16, 17, 18, 18};   // Off, On, Fault, Warning/disconnect(orange)
// const uint8_t DosingPics[5] = {19, 20, 21, 22, 22};
const uint8_t FloatPics[5] = {20, 21, 20, 20, 20};

const uint8_t WifiPics[2] = {9, 10};
const uint8_t Button_Off = 5;
const uint8_t Button_On = 6;

#define MAX_NUM_WARNINGS 12

// Definitions
#define NO_DECIMAL 1
#define ONE_DECIMAL 10
#define TWO_DECIMAL 100
// Precheck on the HMI page print. true to skip the check and print all false to do precheck and only send changes
#define RESEND_ALL true
#define ONLY_CHANGES false

#define joinLsbMsb(lsb, msb) ((msb << 8) | lsb)

typedef std::function<bool(char type, char ID, bool on_off)> cbNextion_t; // Callback skeleton for requests

enum Page_t
{
    P_NONE = 0,
    P_SPLASH = 0xa0,
    P_MAIN = 0xa1,
    P_MANUAL = 0xa2,
    // P_RO = 0xa2,
    P_GAUGES = 0xa3,
    P_SETTINGS = 0xa4,
    P_MANUAL_OLD = 0xa5,
    P_POPUP = 0xa6,
    P_SLEEP = 0xa7,
    P_GLOBAL = 0xa8 // should never go here
};
enum IconType_t
{
    I_PUMP = 'm',
    I_DOSING = 'd',
    I_BW_HEAD = 'a',
    I_VALVE = 'v',
    I_PLANT = 'r',
    I_FLOAT = 'l',
    I_ICON = 'i'
};
enum SensorType_t
{
    I_PRESSURE = 'p',
    I_FLOW = 'f',
    I_SENSOR = 's',
    I_OTHER = 'x'
};
enum Status_t
{
    Off = 0,
    On = 1,
    Warning = 2,
    Fault = 3,
    Disconnected = 4,
    None = 5
};
enum Warn_t
{
    W_None,
    W_High,
    W_Low
};

// Time Strings
#define TIME_STRING "H:i T"
#define DATE_STRING "D, d-M-Y"
#define LOG_STRING "d-M H:i:s"

// RTC values
// #define RTC_YEAR "rtc0"
// #define RTC_MONTH "rtc1"
// #define RTC_DAY "rtc2"
// #define RTC_HOUR "rtc3"
// #define RTC_MIN "rtc4"
// #define RTC_SEC "rtc5"
// #define RTC_DOW "rtc6" // day of the week

// Serial
#define Comms Serial2
#define BAUD_RATE 9600
#define MAX_DATA_SIZE 44 // max number of bytes in a packet
#define START '<'
#define STOP '>'
#define QUOTE '\"'

// headers

struct Value
{
    char type;
    char num;
    uint val = 0;
    uint lastVal = 0;
    uint maxVal = UINT_MAX;
    uint minVal = 0;
    uint mult;
    bool alarm = 0;
    bool lastSentAlarm = 0;
    uint16_t colour[2] = {65535, 64480}; // WHITE, ORANGE
    Value(char IconType, char SeqNo, uint _mult = NO_DECIMAL, uint _min = 0, uint _max = UINT_MAX, uint16_t _aClr = 64480)
    {
        type = IconType;
        num = SeqNo;
        mult = _mult;
        maxVal = _max;
        minVal = _min;
        colour[1] = _aClr;
    }
    bool setMinMax(uint _Val, bool max_nMin)
    {
        if (max_nMin)
            if (maxVal != _Val)
                maxVal = _Val;
            else
                return false;
        else if (minVal != _Val)
            minVal = _Val;
        else
            return false;
        return true;
    }
    // void setValue(uint _val) { val = _val * mult; } handled by nextion function convertToHMIvalue
    bool checkMin(void) { return (val < minVal); }
    bool checkMax(void) { return (val >= maxVal); }
};
struct PumpValues
{
    Value hours;
    Value volts;
    Value current;
    Value frequency;
    Value power;
    const uint FIFTY_Hz = 500;

    PumpValues(uint hrsSN, uint voltSN, uint currentSN, uint freqSN, uint pwrSN)
        : hours('x', hrsSN, NO_DECIMAL),
          volts('x', voltSN, NO_DECIMAL),
          current('x', currentSN, ONE_DECIMAL),
          frequency('x', freqSN, ONE_DECIMAL, 0, 500),
          power('x', pwrSN, ONE_DECIMAL) {}
};
struct Icon
{
    Status_t status = Off;
    Status_t lastStatus = None;
    // for Pumps
    uint lastSentPumpFault = 0;
    // Page_t page;
    IconType_t type;
    char number = '0';
    uint value = 0;
    const uint8_t *statusPics;
    Icon(IconType_t IconType, uint8_t SeqNo, const uint8_t *Pics); //, uint8_t offPic, uint8_t onPic, uint8_t warningPic = 0, uint8_t faultPic = 0, uint8_t disconnectPic = 0);
    void ChangeStatus(Status_t newStatus) { status = newStatus; }
};
struct Sensor
{
    SensorType_t type;
    enum MaxMin_t
    {
        AlarmMin,
        AlarmMax,
        FaultMin,
        FaultMax
    };
    char number = '0';
    uint8_t pic[3];
    uint mult;
    int value;
    int lastSentValue = 0;
    int minAlarmVal = INT_MIN;
    int maxAlarmVal = INT_MAX; // value at which sensor will show alarm
    int minFaultVal = INT_MIN;
    int maxFaultVal = INT_MAX; // value at which sensor will show fault
    bool alarm = 0;
    bool lastStatus = 0;
    bool fault = 0;
    bool lastFaultStatus = 0;
    Sensor(SensorType_t SensType, char SeqNo, const uint8_t *Pics, uint Multiplier = 1, uint _minVal = INT_MIN, uint _maxVal = INT_MAX, uint _Fmin = INT_MIN, uint _Fmax = INT_MAX);
    bool setMinMax(int _Val, MaxMin_t AorF_maxmin)
    {
        switch (AorF_maxmin)
        {
        case AlarmMin:
            if (minAlarmVal != _Val)
                minAlarmVal = _Val;
            else
                return false;
            break;
        case AlarmMax:
            if (maxAlarmVal != _Val)
                maxAlarmVal = _Val;
            else
                return false;
            break;
        case FaultMin:
            if (minFaultVal != _Val)
                minFaultVal = _Val;
            else
                return false;
            break;
        case FaultMax:
            if (maxFaultVal != _Val)
                maxFaultVal = _Val;
            else
                return false;
            break;
        default:
            debugln("Not type MaxMin_t");
        }

        return true;
    }
    bool checkAlarmMin(void) { return (value < minAlarmVal); }
    bool checkAlarmMax(void) { return (value >= maxAlarmVal); }
    bool checkFaultMin(void) { return (value < minFaultVal); }
    bool checkFaultMax(void) { return (value >= maxFaultVal); }
    // void resetMax(void) { maxVal = UINT_MAX; }
    // void resetMin(void) { minVal = 0; }
};
struct TankBar
{
    uint16_t c_ALARM = 63488; // RED
    uint16_t c_NORMAL = 2045;
    char number = '0';
    uint value;
    uint lastSentValue = 0;
    uint minVal;
    uint maxVal;
    bool alarm = 0;
    bool lastStatus = 0;
    uint mult;
    TankBar(uint8_t SeqNo, uint Multiplier = ONE_DECIMAL)
    {
        number = SeqNo;
        mult = Multiplier;
    }
    bool setMinVal(uint _alarmVal)
    {
        if (_alarmVal != minVal)
        {
            minVal = _alarmVal;
            return true;
        }
        return false;
    }
    bool setMaxVal(uint _alarmVal)
    {
        if (_alarmVal != maxVal)
        {
            maxVal = _alarmVal;
            return true;
        }
        return false;
    }
    bool checkMin(void) { return (value < minVal); }
    bool checkMax(void) { return (value >= maxVal); }
};
struct Button
{
    bool on_off = false;
    // uint8_t number;
    uint8_t pic[2];
    // uint8_t OffPic = 13;
    // uint8_t OnPic = 14;
    Button(uint8_t _offpic, uint8_t _onpic)
    {
        // number = SeqNo;
        pic[0] = _offpic;
        pic[1] = _onpic;
    }
};

class Warnings
{
    static const uint txt_maxl = 40;
    const char noneStr[1] = "";
    const char highStr[6] = "High ";
    const char lowStr[6] = "Low ";
    InputTimer Timer;
    uint TimerDelay_ms;
    const char *PreStr = noneStr;
    char TimeStamp[16] = LOG_STRING;
    char WarningString[25];

public:
    char OutputStr[txt_maxl + 1];
    bool WarningLogged = false;
    Warnings(const char *string, uint delay_s) : TimerDelay_ms(delay_s * 1000) { snprintf(WarningString, 25, "%s", string); }
    bool isLogged(void) { return WarningLogged; }
    bool CheckIfWarningTriggered(bool input)
    {
        if (!WarningLogged)
            return Timer.DelayTimer(TimerDelay_ms, input);
        return false;
    }
    void setType(Warn_t WarnType)
    {
        if (WarnType == W_High)
            PreStr = highStr;
        else if (WarnType == W_Low)
            PreStr = lowStr;
        else
            PreStr = noneStr;
    }
    void LogWarning(Warn_t WarnType = W_None)
    {
        WarningLogged = true;
        setType(WarnType);
        if (ezt::timeStatus() == timeSet)
        {
            snprintf(TimeStamp, sizeof(TimeStamp), local.dateTime(LOG_STRING).c_str());
            snprintf(OutputStr, txt_maxl, "%s%s \\r[%s]", PreStr, WarningString, TimeStamp);
        }
        else
            snprintf(OutputStr, txt_maxl, "%s%s", PreStr, WarningString);
    }
    char *getOutputStr() { return &OutputStr[0]; }
    void ClearLog(void)
    {
        WarningLogged = false;
        Timer.ResetTimer();
        Timer.setStatus(false);
    }
};

class Nextion
{
private:
    const uint START_UP_TIMEOUT = 1000; // [ms]
    const uint SPLASH_PAGE_TIME = 5000; // [ms]

    char ser_cnt = 0;
    char header = H_NONE;
    char length = 0;
    cbNextion_t cbNextion = nullptr;
    bool startUp = true;

    uint WarningCntr = 0;
    Warnings *ActiveWarnings[MAX_NUM_WARNINGS] = {nullptr};
    uint WarningIndex = 0;\
    int lastSentDay = -1;
    int lastSentMin = -1;

    enum header_t
    {
        H_NONE = 0,
        H_PAGE_NUM = 'p',
        H_USER_BUTTON = 'u',
        H_MNL_BUTTON = 'b',
        H_MANUAL_STATE = 'k',
        H_ALL_SETTINGS = 's',
        H_FLOWS_SET = 0xb1,
        H_SENSORS_SET = 0xb2,
        H_TIME_SET = 0xb3,
        H_PRESS_SET = 0xb4,
        H_SYSTEM_SET = 0xb5,
        H_WARNING = 'w',
        H_RESET = 'r',
        H_RTC = 'c' // deprecated with Blynk
    };

    // bool sleep = 0;
    void CommsHandler(void);
    bool ChangeIfPageExists(char page);
    // void ManualButtonHandler(uint type, uint n, uint on_off);
    // void RTC_Handler(uint rtc6);
    void ChangeCropPic(const char *item, uint8_t picture);
    void ChangePic(const char *item, uint8_t picture);
    void ChangeValue(const char *item, uint val);
    void ChangeForegroundColour(const char *item, uint colour);
    void ChangeBackgroundColour(const char *item, uint colour);
    void SendGlobalStr(const char *item, const char *str);
    void SendGlobalVal(const char *item, uint val);
    void ChangeVis(const char *item, bool visible);
    void _endTrans(void);

public:
    bool timeEnabled = true;
    bool wifiStatus = false;
    uint RTC_day = 0; // takes day from nextion
    Page_t currPage = P_NONE;
    char data[MAX_DATA_SIZE + 1] = {0};
    Nextion(uint baud, uint Nextion_TX, uint Nextion_RX); // done
    void ChangePage(uint nextPage);
    void setCallback(cbNextion_t _cb) { cbNextion = _cb; }
    void Startup(void);
    void SendStartupValues(void);
    void SendSetting(const char *setting, uint settingVal);
    void SendTimeStr(void);
    void SendDateStr(void);
    void SendStateStr(const char *state);
    void SendTimeValues(void);
    void SendButtonPicVal(Button button);
    void SendIconStatus(Icon &icon, bool Precheck = ONLY_CHANGES);
    void SendSensorValueStatus(Sensor &sensor, bool SkipPrecheck = ONLY_CHANGES);
    void SendSensorValue(Sensor &sensor, bool SkipPrecheck = ONLY_CHANGES);
    void SendSensorStatus(Sensor &sensor, bool SkipPrecheck = ONLY_CHANGES);
    void SendTankLevelStatus(TankBar &tank, bool SkipPrecheck = ONLY_CHANGES);
    void SendPumpValues(PumpValues &pump, bool SkipPrecheck = ONLY_CHANGES);
    void SendValue(Value &value, bool SkipPrecheck);
    void SendStatus(Value &value, bool SkipPrecheck);
    void PrintPage(bool firstEntry = ONLY_CHANGES);
    void ConvertToHMI_SensorValue(float val, Sensor &sensor, bool checkMaxMin = true);
    void ConvertToHMI_Value(float val, Value &value, bool checkMaxMin = true);
    void ConvertToHMI_TankLevel(float val, TankBar &tank, bool alarm);
    void UpdateCountdown(uint seconds);
    void RemoveCountdown(void);
    void AddWarning(Warnings &addedWarn, Warn_t type = W_None);
    void AddFault(Warnings &addedWarn, Warn_t type = W_None);
    void ClearWarnings(void);
    void NextWarning(bool INCREMENT_nDECREMENT);
    void SendCurrWarning(void);
    void SendPumpFault(uint fault, Icon &PumpIcon);
    void ActivateCIPWarning(void);
    void UpdateWifiStatus(bool on_off);
    void UpdateTimeVisibility(bool visible);
    void Listen(void);
};

#endif
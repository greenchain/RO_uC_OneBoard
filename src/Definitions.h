#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <preferences.h>
#include "OB_Define.h"
#include "Nextion.h"
#include "MyModbus.h"

//********************** CALIBRATION **********************
#define DEF_VOLTAGE_FACTOR 2670
#define DEF_CURRENT_FACTOR 1280
#define NumADC_samples 40

/*Constants FOR RO CONTROLLER*/
#define SENSOR_MIN_0 0.0F

#define SENSOR_MAX_EC 2000.0F

#define P_10_BAR 10.0F
#define P_16_BAR 16.0F
#define P_25_BAR 25.0F

#define P_SENSOR_MAX_FeedP P_10_BAR
#define P_SENSOR_MAX_HPin P_10_BAR
#define P_SENSOR_MAX_PreMP P_10_BAR
#define P_SENSOR_MAX_PostMP P_10_BAR
#define P_SENSOR_MAX_Boost P_10_BAR

#define VSD_V_FACTOR 100

#define DEF_SMALL_FLOW_FACTOR 5.76F // 60 / 4.8
#define DEF_LARGE_FLOW_FACTOR 0.55F // 60 / 0.5

// PINS
// Relay Outputs
#define INLET_VALVE D_OUT_1
#define FLUSH_VALVE D_OUT_2
#define FEED_START_STOP D_OUT_3
// #define REMIN_VALVE D_OUT_4
#define HPP_START_STOP D_OUT_4
#define SPARE1 D_OUT_5
#define SPARE2 D_OUT_6
#define SPARE3 D_OUT_7
#define WARNING_LIGHT D_OUT_8

// Inputs
#define FEED_AVAILABLE D_IN_1
#define PRODUCT_FLOAT D_IN_2
#define BW_RELAY D_IN_3
#define FLOW_PERM D_IN_4
#define FLOW_BRINE D_IN_5
#define FLOW_RECYCLE D_IN_6
#define VOLUME_IN D_IN_7
#define FAULT_IN D_IN_8

enum state_t
{
    // states
    ST_STOPPED = 0,
    ST_START_FLUSH = 1,
    ST_SERVICE = 2,
    ST_STOP_FLUSH = 3,
    ST_TANK_FULL = 4,
    ST_PREFILTER = 5,
    ST_VSD_FAULT = 6,
    ST_FEED_EMPTY = 7,
    ST_EXT_FAULT = 8,
    ST_OVER_P_FAULT = 9,
    ST_D_P_FAULT = 10,
    ST_PROD_FLOW_FAULT = 11,
    ST_BRINE_FLOW_FAULT = 12,
    ST_EC_FAULT = 13,
    ST_HPP_INLET_P_FAULT = 14,
    ST_MANUAL = 15
};
const char stateStr[][30] = {
    "User Stopped",
    "Start Flush",
    "Running",
    "Flush",
    "Product Tank Full",
    "Prefilter Lockout",
    "VSD Fault",
    "Feed Tank Low",
    "External Fault",
    "Overpressure Fault",
    "dP Membrane Fault",
    "Product Flow Fault",
    "Brine Flow Fault",
    "EC Fault",
    "Feed Pressure Fault",
    "Manual" // this probably isn't used
};
enum HMI_Callback_t
{
    Error = 0,
    StartUp = '$',
    UserButton = 'u',
    MnlButton = 'b',
    MnlState = 'k',
    SetFlows = 0xb1,
    SetSensors = 0xb2,
    SetTimes = 0xb3,
    SetPressures = 0xb4,
    ResetFault = 'r',
    HandleWarning = 'w',
    Calibration = 'c'
};

extern SemaphoreHandle_t xSemaphore;

extern Nextion HMI;
extern ADS1115 ADC_1;
// extern VolumeMeter FeedVM;
extern VolumeMeter PermeateVM;
extern PulseMeter PermeatePM;
extern PulseMeter BrinePM;
extern PulseMeter RecyclePM;

extern DigitalInput FeedTankFloat;
extern DigitalInput ProductTankFloat;
extern DigitalInput BackwashRelay;
extern DigitalInput FaultRelay;

extern TempSensor AmbientTemp;

extern ModbusVSD HPP_VSD;
extern ModbusVSD BOOST_VSD;
extern ModbusVSD FP_VSD;

extern Button B_USER_ON_OFF;
extern Icon I_WIFI;

// Main page
extern Icon I_RO_PUMP;
extern Icon I_BOOSTER_PUMP;
extern Icon I_FEED_PUMP;

extern Icon I_BW_VALVE;
extern Icon I_INLET_VALVE;
extern Icon I_FLUSH_VALVE;
extern Icon I_FEED_FLOAT;

extern Sensor I_FEED_PUMP_PRESS;
extern Sensor I_HP_INLET_PRESS;
extern Sensor I_HP_OUTLET_PRESS;
extern Sensor I_POST_MEM_PRESS;
extern Sensor I_DELTA_PRESS;
extern Sensor I_BOOSTER_PRESS;

extern Sensor I_FEED_FLOW;
extern Sensor I_PERM_FLOW;
extern Sensor I_RECYCLE_FLOW;
extern Sensor I_BRINE_FLOW;

extern Sensor I_TEMP;
extern Sensor I_EC_PERM;
extern Sensor I_RECOVERY;

extern PumpValues PVAL_RO_PUMP;
extern PumpValues PVAL_BOOSTER_PUMP;

extern Value VAL_RECOVERY;
extern Value VAL_PERM_VOL;

extern Value VAL_CalibVoltage;
extern Value VAL_CalibCurrent;
extern Value VAL_CalibSmallPM;
extern Value VAL_CalibLargePM;

extern Value VAL_A0_Current;
extern Value VAL_A1_Current;
extern Value VAL_A2_Current;
extern Value VAL_A3_Current;

extern Value VAL_A0_Voltage;
extern Value VAL_A1_Voltage;
extern Value VAL_A2_Voltage;
extern Value VAL_A3_Voltage;

// Blynk strings
// extern char B_State[128];
// extern char B_Warning[128];
// extern char B_Fault[128];
extern char B_Setting[128];

void RunCoreFunctions(void);

bool cbHPP_ReadHreg(Modbus::ResultCode event, uint16_t transactionId, void *data);
bool cbBoosterReadHreg(Modbus::ResultCode event, uint16_t transactionId, void *data);
bool cbFeedReadHreg(Modbus::ResultCode event, uint16_t transactionId, void *data);
void PumpStatusHandler(ModbusVSD &PumpVSD, Icon &I_PumpStatus, PumpValues &VAL_PumpValues, float &Analog1, float &Analog2); //, int BlynkPin);

void CheckSensors(void);
void CheckInputs(void);
void CheckPulseMeters(void);
void StartService(void);
void StartFlush(void);
// void StartBypass(void);
// void StopBypass(void);
void StopAll(void);
void UpdateHMI(void);

bool cbNextionListen(char type, char ID, bool on_off);
void ResetFaults(void);
HMI_Callback_t checkCBType(char type);
void SendHMI_SettingValues(void);
void ManualModeButton(uint button, bool on_off);
void ManualStateSwitch(uint state);
void CalibrationHandler(char type, bool up_down);

void TimerSetup(void);
void BlynkLoop(void *pvParameters);
void RunAnalog(ADS1115 &ADC_ref);

// NON-VOLATILE STORAGE CLASS
class Save
{
private:
    const char openFailed[17] = "File open Failed";
    // Keys for storage using preferences library
    const char exists_k[4] = "exi"; // save files exist or not
    /* settings from nextion */
    /* Flows */
    const char u_but_k[4] = "ubs"; // User button status
    const char minpf_k[4] = "pfm"; // minimum permeate flow
    const char maxpf_k[4] = "pfx"; // maximum permeate flow
    const char minrf_k[4] = "rfm"; // minimum recycle flow
    const char maxrf_k[4] = "rfx"; // maximum recycle flow
    const char minbf_k[4] = "bfm"; // minimum brine flow
    const char maxbf_k[4] = "bfx"; // maximum brine flow
    const char mnpfF_k[4] = "pFm"; // minimum permeate flow FAULT
    const char mxpfF_k[4] = "pFx"; // maximum permeate flow FAULT
    const char mnbfF_k[4] = "bFm"; // minimum brine flow FAULT
    /* Sensors */
    const char R_min_k[4] = "Rmn"; // Recovery min setpoint
    const char R_max_k[4] = "Rmx"; // Recovery max setpoint
    const char ECalm_k[4] = "ECA"; // EC alarm setpoint
    const char ECflt_k[4] = "ECF"; // EC max setpoint FAULT
    /* Times */
    const char start_k[4] = "stf"; // start flush time
    const char stopf_k[4] = "spf"; // stop flush time
    /* Pressures */
    const char mP_fI_k[4] = "fmP"; // minimum feed pressure to HPP
    const char mP_MI_k[4] = "mmP"; // maximum pressure at the membrane inlet
    const char mPwdP_k[4] = "wdP"; // Warning differential pressure across the membrane
    const char mPfdP_k[4] = "fdP"; // maximum differential pressure across the membrane
    /* Logs */
    const char HpMn_k[4] = "HPm"; // total mins that the pump has run
    const char BpMn_k[4] = "BPm"; // total mins that the booster pump has run
    const char FpMn_k[4] = "FPm"; // total mins that the booster pump has run
    const char volF_k[4] = "vlF"; //
    const char volP_k[4] = "vlP"; //

    const char cftr_k[4] = "cft"; // current conversion factor
    const char vftr_k[4] = "vft"; // voltage conversion factor
    const char PMsm_k[4] = "PMs"; // small pulse meter factor
    const char PMlg_k[4] = "PMl"; // large pulse meter factor

    Preferences settings; // File system Instance that saves settings based on a key

    bool begin(void) { return settings.begin("RO_SET", false); } // open in read/write mode
    void format(void) { settings.clear(); }                      // format any settings that may exist
    void saveExist(void) { settings.putBool(exists_k, true); }   // SETS SETTINGS EXISTANCE TO TRUE
    bool exists(void) { return settings.isKey(exists_k); }       // check whether settings exists
    void initialise(void);
    void getAllSettings(void); // get all saved settings
    bool getBoolIfExist(const char *key, bool defVal);
    uint16_t getUshortIfExist(const char *, uint16_t defVal);
    uint getUIntIfExist(const char *key, uint defVal);
    long getLongIfExist(const char *key, long defVal);

public:
    const uint MIN_BETWEEN_SAVE = 6;
    const uint LITRES_BETWEEN_SAVE = 100;
    uint LastSavedRO_PumpMins = 0;
    uint LastSavedBoostPumpMins = 0;
    uint LastSavedPermVol = 0;
    uint LastSavedFeedPumpMins =0;

    //  Functions for saving/retrieving saved variables
    void FileSetup(void);
    void StoreRunButton(bool butStatus) { settings.putBool(u_but_k, butStatus); }
    void getCalibrationSettings(void);

    void StoreMinPermFlow(uint16_t minFlow) { settings.putUShort(minpf_k, minFlow); }
    void StoreMaxPermFlow(uint16_t maxFlow) { settings.putUShort(maxpf_k, maxFlow); }
    void StoreMinRecycFlow(uint16_t minFlow) { settings.putUShort(minrf_k, minFlow); }
    void StoreMaxRecycFlow(uint16_t maxFlow) { settings.putUShort(maxrf_k, maxFlow); }
    void StoreMinBrineFlow(uint16_t minFlow) { settings.putUShort(minbf_k, minFlow); }
    void StoreMaxBrineFlow(uint16_t maxFlow) { settings.putUShort(maxbf_k, maxFlow); }
    void StoreMinFaultPermFlow(uint16_t minFlow) { settings.putUShort(mnpfF_k, minFlow); }
    void StoreMaxFaultPermFlow(uint16_t maxFlow) { settings.putUShort(mxpfF_k, maxFlow); }
    void StoreMinFaultBrineFlow(uint16_t maxFlow) { settings.putUShort(mnbfF_k, maxFlow); }

    void StoreRecoveryMinVal(uint16_t minRec) { settings.putUShort(R_min_k, minRec); }
    void StoreRecoveryMaxVal(uint16_t maxRec) { settings.putUShort(R_max_k, maxRec); }
    void StoreEC_AlarmVal(uint16_t minEC) { settings.putUShort(ECalm_k, minEC); }
    void StoreEC_FaultVal(uint16_t maxEC) { settings.putUShort(ECflt_k, maxEC); }

    void StoreStartFlushTime(uint16_t startFtime) { settings.putUShort(start_k, startFtime); }
    void StoreStopFlushTime(uint16_t stopFtime) { settings.putUShort(stopf_k, stopFtime); }

    void StoreMinFeedPressure(uint16_t feedP) { settings.putUShort(mP_fI_k, feedP); }
    void StoreMaxMemPressure(uint16_t memP) { settings.putUShort(mP_MI_k, memP); }
    void StoreWarnDeltaPressure(uint16_t dP) { settings.putUShort(mPwdP_k, dP); }
    void StoreFaultDeltaPressure(uint16_t dP) { settings.putUShort(mPfdP_k, dP); }

    void StoreRO_PumpMins(uint pumpMins) // check if there have been enough mins since last save and then save new value
    {
        if (pumpMins - LastSavedRO_PumpMins > MIN_BETWEEN_SAVE)
        {
            settings.putUInt(HpMn_k, pumpMins);
            LastSavedRO_PumpMins = pumpMins;
        }
    }
    void StoreBoostPumpMins(uint pumpMins) // check if there have been enough mins since last save and then save new value
    {
        if (pumpMins - LastSavedBoostPumpMins > MIN_BETWEEN_SAVE)
        {
            settings.putUInt(BpMn_k, pumpMins);
            LastSavedBoostPumpMins = pumpMins;
        }
    }
    void StoreFeedPumpMins(uint pumpMins) // check if there have been enough mins since last save and then save new value
    {
        if (pumpMins - LastSavedFeedPumpMins > MIN_BETWEEN_SAVE)
        {
            settings.putUInt(FpMn_k, pumpMins);
            LastSavedRO_PumpMins = pumpMins;
        }
    }
    void StorePermVol(uint Volume)
    {
        if (Volume - LastSavedPermVol > LITRES_BETWEEN_SAVE)
        {
            settings.putUInt(volP_k, Volume);
            LastSavedPermVol = Volume;
        }
    }

    void StoreVoltageFactor(uint16_t factor) { settings.putUShort(vftr_k, factor); }
    void StoreCurrentFactor(uint16_t factor) { settings.putUShort(cftr_k, factor); }
    void StoreSmallPM_Factor(uint16_t factor) { settings.putUShort(PMsm_k, factor); }
    void StoreLargePM_Factor(uint16_t factor) { settings.putUShort(PMlg_k, factor); }
    size_t freeEntries(void) { return settings.freeEntries(); }
};

class StateMachineRO
{
private:
public:
    // variables
    bool inline_mode = true;

    bool runButton = false;
    bool backwashing_flag = false;
    bool productFloat_flag = false;

    bool feedFloat_flag = false;
    bool feedFloatFault = false;

    bool faultRelay_flag = false;
    bool faultRelayFault = false;
    bool overPressFault = false;
    bool deltaPressFault = false;
    bool vsdFault = false;
    bool permeateFlowFault = false;
    bool brineFlowFault = false;
    bool EC_MaxFault = false;
    bool feedPressFault = false;

    bool logBlynkStateChange = false;

    state_t state = ST_STOPPED;
    state_t stateAfterFlush = ST_STOPPED;

    /* ANALOG INPUTS */
    // float PH = SENSOR_MIN_PH;
    // float ORP = SENSOR_MIN_ORP;
    float EC = SENSOR_MIN_0;
    float HPP_InletPressure = SENSOR_MIN_0;    
    float PostMemPressure = SENSOR_MIN_0;

    /* CALCULATED */
    float DeltaMemPressure = SENSOR_MIN_0;

    /* MODBUS INPUTS */
    float HP_PumpPressure = SENSOR_MIN_0;
    float BoostPumpPressure = SENSOR_MIN_0;
    float FeedPumpPressure = SENSOR_MIN_0;

    float FeedFlow = 0.0;
    float Recovery = 0.0;

    // TankLevelSensor ProductTank;

    uint SecNow = 0;
    uint StartFlushTimeSecs = 60;
    uint StopFlushTimeSecs = 60;

    // methods
    // StateMachineRO() : ProductTank(DEF_TANK_LEVEL_SENSOR_MAX) {}
    void RunStateMachine(void);
    void CheckWarningsFaults(void);
    void InServiceChecks(void);
    void StartFlushChecks(void);
    void StopFlushChecks(void);
    void ChangeState(state_t nextState);
};

extern StateMachineRO SM;
extern Save SET;
extern bool analog_flag;

#endif // DEFINITIONS_H_
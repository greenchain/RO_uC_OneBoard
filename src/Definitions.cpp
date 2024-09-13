#include "BlynkRO.h"
#include "Definitions.h"
#include <BlynkEdgent.h>

#define MIN_IN_SEC 60
#define TEN_SEC 10
#define TEN_SEC_IN_MS 10000

StateMachineRO SM;
Save SET;

Nextion HMI(BAUD_RATE, HMI_RX_PIN, HMI_TX_PIN);
ADS1115 ADC_1(0x48);

VolumeMeter PermeateVM(VOLUME_IN);
PulseMeter PermeatePM(FLOW_PERM, LARGE_FLOW_FACTOR);
PulseMeter BrinePM(FLOW_BRINE, SMALL_FLOW_FACTOR);     // 12.5
PulseMeter RecyclePM(FLOW_RECYCLE, LARGE_FLOW_FACTOR); // 120

DigitalInput FeedTankFloat(FEED_AVAILABLE);
DigitalInput ProductTankFloat(PRODUCT_FLOAT);
DigitalInput BackwashRelay(BW_RELAY);
DigitalInput FaultRelay(FAULT_IN);

TempSensor AmbientTemp;
ModbusVSD HPP_VSD(1, cbHPP_ReadHreg);
ModbusVSD BOOST_VSD(2, cbBoosterReadHreg);

Warnings PermFlowWarning("Product Flow", MIN_IN_SEC);
Warnings XFlowWarning("Recycle Flow", MIN_IN_SEC);
Warnings BrineFlowWarning("Brine Flow", MIN_IN_SEC);

Warnings RecoveryWarning("Recovery Rate", MIN_IN_SEC);
Warnings EC_Warning("EC", MIN_IN_SEC);
Warnings PumpSpeedWarning("Max VSD Speed", MIN_IN_SEC);

Warnings dPressWarning("dP Membrane Warning", MIN_IN_SEC);
Warnings FeedTankWarning(stateStr[ST_FEED_EMPTY], TEN_SEC);
Warnings ProductTankWarning("Product Tank Low", TEN_SEC);

// FAULTS
Warnings PermFlowFault(stateStr[ST_PROD_FLOW_FAULT], MIN_IN_SEC);
Warnings BrineFlowFault(stateStr[ST_BRINE_FLOW_FAULT], MIN_IN_SEC);

Warnings EC_Fault(stateStr[ST_EC_FAULT], MIN_IN_SEC);

Warnings FeedPressFault(stateStr[ST_FEED_P_FAULT], TEN_SEC);
Warnings MemPressFault(stateStr[ST_OVER_P_FAULT], TEN_SEC);
Warnings dPressFault(stateStr[ST_D_P_FAULT], MIN_IN_SEC);

Warnings ExternalFault(stateStr[ST_EXT_FAULT], TEN_SEC);
Warnings VSD_Fault(stateStr[ST_VSD_FAULT], TEN_SEC);

bool Check4to20(float &value)
{
    if (value < 4.0F)
    {
        value = 4.0F;
        return false;
    }
    else if (value > 20.0F)
    {
        value = 20.0F;
        return false;
    }
    else
        return true;
}
void CheckSensors(void) // TODO
{
    static float Temp, Temp1, Temp2, Temp3, Temp4 = 0;
    const uint AVERAGE_NUM = 5;
    const uint AVG_LESS1 = AVERAGE_NUM - 1;
    // Check all analog
    if (analog_flag)
    {
        // // ANALOGUE 1
        Temp1 = (float)ADC_1.readADC(0) / CurrentFactor;
        Check4to20(Temp1);
        if (SM.EC > SENSOR_MIN_EC)
            SM.EC = (SM.EC * AVG_LESS1 + map_4to20(Temp1, SENSOR_MIN_EC, SENSOR_MAX_EC)) / AVERAGE_NUM;
        else
            SM.EC = map_4to20(Temp1, SENSOR_MIN_EC, SENSOR_MAX_EC);
        HMI.ConvertToHMI_SensorValue(SM.EC, I_EC_PERM);

        // // ANALOGUE 2
        Temp2 = (float)ADC_1.readADC(1) / CurrentFactor;
        Check4to20(Temp2);
        if (SM.FeedPressure > SENSOR_MIN_FP)
            SM.FeedPressure = (SM.FeedPressure * AVG_LESS1 + map_4to20(Temp2, SENSOR_MIN_FP, SENSOR_MAX_FP)) / AVERAGE_NUM;
        else
            SM.FeedPressure = map_4to20(Temp2, SENSOR_MIN_FP, SENSOR_MAX_FP);
        HMI.ConvertToHMI_SensorValue(SM.FeedPressure, I_FEED_PRESS);

        // // ANALOGUE 3
        // Temp3 = (float)ADC_1.readADC(2) / CurrentFactor;
        // Check4to20(Temp3);

        // // ANALOGUE 4
        // Temp4 = (float)ADC_1.readADC(3) / CurrentFactor;
        // Check4to20(Temp4);

        // SEND SENSOR DATA TO BLYNK
        Blynk.virtualWrite(B_EC, SM.EC);
        Blynk.virtualWrite(B_FEED_PRESS, SM.FeedPressure);
        // debugln("LVL:  " + String(Temp1, 2) + "mA  " + String((float)SM.ProductTank.Level_p, 1) + "\% - CF:" + String(CurrentFactor0));
        // debugln("PH: " + String(Temp2, 2) + "mA  " + String((float)SM.PH, 2) + "pH - CF:" + String(CurrentFactor1));
        // debugln("ORP: " + String(Temp3, 2) + "mA  " + String((float)SM.ORP, 1) + "mV - CF:" + String(CurrentFactor2));
        // debugln("EC: " + String(Temp4, 2) + "mA  " + String((float)SM.EC, 1) + "uS/cm - CF:" + String(CurrentFactor3));
        // debugln();
    }

    // Check Pump values
    float TempVSD1 = 0, TempVSD2 = 0;
    PumpStatusHandler(HPP_VSD, I_RO_PUMP, PVAL_RO_PUMP, TempVSD1, TempVSD2, B_RO_PUMP);
    bool OutOf4to20_alarm = !Check4to20(TempVSD1);
    TempVSD1 = map_4to20(TempVSD1, SENSOR_MIN_HP, SENSOR_MAX_HP); // map to pressure
    HMI.ConvertToHMI_SensorValue(TempVSD1, I_HP_PRESS, HPP_VSD.Running);
    if (OutOf4to20_alarm)
        I_HP_PRESS.alarm = true;
    Blynk.virtualWrite(B_HP_PRESS, TempVSD1);
    // debugln("VSD A1: " + String(TempVSD1, 2));

    OutOf4to20_alarm = !Check4to20(TempVSD2);
    TempVSD2 = map_4to20(TempVSD2, SENSOR_MIN_HP, SENSOR_MAX_HP);
    HMI.ConvertToHMI_SensorValue(TempVSD2, I_POST_MEM_PRESS, HPP_VSD.Running);
    if (OutOf4to20_alarm)
        I_HP_PRESS.alarm = true;
    Blynk.virtualWrite(B_POST_MEM_PRESS, TempVSD2);
    // debugln("VSD A2: " + String(TempVSD2, 2));

    float diffP = TempVSD1 - TempVSD2;
    HMI.ConvertToHMI_SensorValue(diffP, I_DELTA_PRESS, HPP_VSD.Running);
    Blynk.virtualWrite(B_DELTA_PRESS, diffP);
    // debugln("dp: " + String(TempVSD1, 2));

    SET.StoreRO_PumpMins(HPP_VSD.pumpMin);
    // debugln();

    PumpStatusHandler(BOOST_VSD, I_BOOSTER_PUMP, PVAL_BOOSTER_PUMP, TempVSD1, TempVSD2, B_BOOST_PUMP);
    OutOf4to20_alarm = !Check4to20(TempVSD1);
    TempVSD1 = map_4to20(TempVSD1, SENSOR_MIN_FP, SENSOR_MAX_FP);
    HMI.ConvertToHMI_SensorValue(TempVSD1, I_BOOSTER_PRESS, false);
    if (OutOf4to20_alarm || BOOST_VSD.PumpState == ModbusVSD::PumpDisconnect)
        I_BOOSTER_PRESS.alarm = true;
    Blynk.virtualWrite(B_BOOST_PRESS, TempVSD1);

    // Calculate all inputs with flows

    if (PermeateVM.CheckVolumeMeter(false))
    {
        float VolumeM3 = (float)PermeateVM.Volume_L / 1000.0;
        // debugln("PERM VOL: " + String(PermeateVM.Volume_L, 1));
        HMI.ConvertToHMI_Value(VolumeM3, VAL_PERM_VOL);
        SET.StorePermVol(PermeateVM.Volume_L);
        Blynk.virtualWrite(B_PRODUCT_VOLUME, VolumeM3);
    }

    PermeatePM.CalculateFlow();
    HMI.ConvertToHMI_SensorValue(PermeatePM.FlowRate, I_PERM_FLOW, SM.state == ST_SERVICE);
    Blynk.virtualWrite(B_PERM_FLOW, PermeatePM.FlowRate);

    BrinePM.CalculateFlow();
    // debugln("Brine: " + String(BrinePM.FlowRate, 2));
    HMI.ConvertToHMI_SensorValue(BrinePM.FlowRate, I_BRINE_FLOW, SM.state == ST_SERVICE);
    Blynk.virtualWrite(B_BRINE_FLOW, BrinePM.FlowRate);

    RecyclePM.CalculateFlow();
    HMI.ConvertToHMI_SensorValue(RecyclePM.FlowRate, I_RECYCLE_FLOW, SM.state == ST_SERVICE);
    Blynk.virtualWrite(B_RECYCLE_FLOW, RecyclePM.FlowRate);

    // Calculate Feed Flow
    Temp = PermeatePM.FlowRate + BrinePM.FlowRate;
    HMI.ConvertToHMI_SensorValue(Temp, I_FEED_FLOW, SM.state == ST_SERVICE);
    Blynk.virtualWrite(B_FEED_FLOW, Temp);

    if (PermeatePM.FlowRate >= 10 && BrinePM.FlowRate >= 10)
        Temp = PermeatePM.FlowRate / (Temp) * 100;
    else
        Temp = 0;
    HMI.ConvertToHMI_Value(Temp, VAL_RECOVERY, SM.state == ST_SERVICE);
    HMI.ConvertToHMI_SensorValue(Temp, I_RECOVERY, SM.state == ST_SERVICE);
    Blynk.virtualWrite(B_RECOVERY, Temp);

    if (AmbientTemp.ReadTemp())
    {
        if (AmbientTemp.currTemp >= 0)
        {
            HMI.ConvertToHMI_SensorValue(AmbientTemp.currTemp, I_TEMP);
            Blynk.virtualWrite(B_TEMP, AmbientTemp.currTemp);
            debugln("TEMP READ: " + String(AmbientTemp.currTemp, 1));
        }
        else
            debugln("ERR: TEMP READ: " + String(AmbientTemp.currTemp, 1));
        // debugln("TEmp: " + String(FeedTemp.currTemp));
    }
}
void PumpStatusHandler(ModbusVSD &PumpVSD, Icon &I_PumpStatus, PumpValues &VAL_PumpValues, float &Analog1, float &Analog2, int BlynkPin)
{
    enum
    {
        WHITE = 0xFFFFFF,
        GREEN = 0x23C48E,
        RED = 0xD3435C,
        GREY = 0x808080
    };

    // UPDATE PUMP STATE
    // debugln("PumpState: " + String(PumpVSD.PumpState, DEC));
    if (PumpVSD.PumpState == ModbusVSD::PumpRun && I_PumpStatus.status != On)
    {
        I_PumpStatus.ChangeStatus(On);
        Blynk.setProperty(BlynkPin, "color", '#' + String(GREEN, HEX));
    }
    else if (PumpVSD.PumpState == ModbusVSD::PumpStop && I_PumpStatus.status != Off)
    {
        // SM.vsdFault = false;
        I_PumpStatus.ChangeStatus(Off);
        HMI.SendPumpFault(0, I_PumpStatus);
        Blynk.setProperty(BlynkPin, "color", '#' + String(WHITE, HEX));
    }
    else if (PumpVSD.PumpState == ModbusVSD::PumpFault && I_PumpStatus.status != Fault)
    {
        HMI.SendPumpFault(PumpVSD.Fault, I_PumpStatus); // prints the fault on the RO page
        if (millis() - PumpVSD.TimerFault > TEN_SEC_IN_MS)
        {
            I_PumpStatus.ChangeStatus(Fault);
            // SM.vsdFault = true;
            // Blynk.logEvent(BL_FAULT, "Pump VSD: F" + PumpVSD.Fault); Now in the change state
            Blynk.setProperty(BlynkPin, "color", '#' + String(RED, HEX));
        }
        else if (I_PumpStatus.status != Warning)
        {
            I_PumpStatus.ChangeStatus(Warning);
        }
    }
    else if (PumpVSD.PumpState == ModbusVSD::PumpDisconnect && I_PumpStatus.status != Disconnected)
    {
        if (millis() - PumpVSD.TimerDisconnect > TEN_SEC_IN_MS)
        {
            debugln("**********PUMP DISCONNECTED");
            I_PumpStatus.ChangeStatus(Disconnected);
            Blynk.logEvent(BL_WARNING, "VSD Disconnected");
            Blynk.setProperty(BlynkPin, "color", '#' + String(GREY, HEX));
        }
    }

    // RESET TIMERS
    if (PumpVSD.PumpState != ModbusVSD::PumpFault)
        PumpVSD.TimerFault = millis();
    if (PumpVSD.PumpState != ModbusVSD::PumpDisconnect)
        PumpVSD.TimerDisconnect = millis();

    if (I_PumpStatus.status != Disconnected)
    {
        Analog1 = PumpVSD.AnalogIn1 * 2; // this converts 2-10v to 4-20ma
        Analog1 /= VSD_V_FACTOR;         // divide to get the correct scaling

        Analog2 = PumpVSD.AnalogIn2 * 2; // this converts 2-10v to 4-20ma
        Analog2 /= VSD_V_FACTOR;

        VAL_PumpValues.hours.val = PumpVSD.pumpMin / 60;

        HMI.ConvertToHMI_Value((float)PumpVSD.Frequency / 100, VAL_PumpValues.frequency, PumpVSD.Running);
        HMI.ConvertToHMI_Value(PumpVSD.VoltageOut, VAL_PumpValues.volts, PumpVSD.Running);
        HMI.ConvertToHMI_Value((float)PumpVSD.CurrentOut / 10, VAL_PumpValues.current, PumpVSD.Running);
        HMI.ConvertToHMI_Value((float)PumpVSD.PowerOut / 10, VAL_PumpValues.power, PumpVSD.Running);
    }
    else
    {
        Analog1 = 0;
        Analog2 = 0;
    }
}
void CheckInputs(void)
{
    const uint SEC = 1000;
    const uint HUNDREDTH_SEC = 100;

    SM.feedFloat_flag = FeedTankFloat.ReadInputDebounce(SEC);
    // if (SM.feedFloat_flag)
    //     I_FEED_FLOAT.ChangeStatus(Fault);
    // else
    // {
    //     SM.feedFloatFault = false;
    //     I_FEED_FLOAT.ChangeStatus(Off);
    // }
    SM.feedFloatFault = FeedTankFloat.ReadInputDelay(SEC * 5, SM.feedFloat_flag); // THIS IS ADDED INSTEAD OF A WARNING EVERYTIME UF BACKWASHES (ONLY FOR INLINE)

    SM.productFloat_flag = ProductTankFloat.ReadInputDebounce(SEC);

    SM.backwashing_flag = BackwashRelay.ReadInputDebounce(SEC);
    I_BW_VALVE.ChangeStatus((Status_t)SM.backwashing_flag);

    SM.faultRelay_flag = FaultRelay.ReadInputDebounce(HUNDREDTH_SEC);
    // SM.faultRelayFault = FaultRelay.ReadInputDelay(SEC * 10, SM.faultRelay_flag);
}

void StartService(void) // Function to engage valves for service
{
    WriteOutput(INLET_VALVE, HIGH);
    WriteOutput(FLUSH_VALVE, LOW);
    WriteOutput(FEED_START_STOP, HIGH);
    WriteOutput(HPP_START_STOP, HIGH);

    I_INLET_VALVE.ChangeStatus(On);
    I_FLUSH_VALVE.ChangeStatus(Off);
    // I_REMIN_VALVE.ChangeStatus(On);
    // I_RO_PLANT.ChangeStatus(On);
}
void StartFlush(void)
{
    WriteOutput(INLET_VALVE, HIGH);
    WriteOutput(FLUSH_VALVE, HIGH);
    WriteOutput(FEED_START_STOP, HIGH);
    WriteOutput(HPP_START_STOP, LOW);

    I_INLET_VALVE.ChangeStatus(On);
    I_FLUSH_VALVE.ChangeStatus(On);
}
void StopAll(void)
{
    WriteOutput(INLET_VALVE, LOW);
    WriteOutput(FLUSH_VALVE, LOW);
    WriteOutput(FEED_START_STOP, LOW);
    WriteOutput(HPP_START_STOP, LOW);

    I_INLET_VALVE.ChangeStatus(Off);
    I_FLUSH_VALVE.ChangeStatus(Off);
}

void UpdateHMI(void) // called every sec
{
    if (HMI.currPage != P_NONE) // check if nextion is online
    {
        I_WIFI.ChangeStatus((Status_t)Blynk.connected());
        HMI.PrintPage(ONLY_CHANGES);
        if (ezt::timeStatus() == timeSet)
        {
            HMI.SendTimeValues();
        }
        else
        {
            HMI.UpdateTimeVisibility(false);
            if (Blynk.connected())
                debugln("Connected but time not set");
        }
    }
    UpdateBlynkWithInputs();
}
void UpdateBlynkWithInputs(void)
{
    Blynk.virtualWrite(B_FEED_TANK, SM.feedFloat_flag);
    Blynk.virtualWrite(B_PRODUCT_TANK, SM.productFloat_flag);
    Blynk.virtualWrite(B_PREFILTER_LOCKOUT, SM.backwashing_flag);
    Blynk.virtualWrite(B_EXTERNAL_FAULT, SM.faultRelay_flag);
}

HMI_Callback_t checkCBType(char type)
{
    HMI_Callback_t _cb_t = Error;
    switch (type)
    {
    case UserButton:
    case StartUp:
    case MnlButton:
    case MnlState:
    case SetFlows:
    case SetSensors:
    case SetTimes:
    case SetPressures:
    case ResetFault:
    case HandleWarning:
        return (HMI_Callback_t)type;
        break;
    default:
        return Error;
        break;
    }
}
bool cbNextionListen(char type, char ID, bool on_off)
{
    HMI_Callback_t cbType = checkCBType(type);
    switch (cbType)
    {
    case Error:
        debugln("cbError");
        debug(type);
        debugln(ID);
        debugln(HMI.data);
        return false;
        break;
    case StartUp:
        SendHMI_SettingValues();
        HMI.SendStateStr(stateStr[SM.state]);
        HMI.SendStartupValues();
        // UpdateHMI();
        break;
    case UserButton:
        SM.runButton = B_USER_ON_OFF.on_off;
        debugln("button: " + String(SM.runButton));
        SET.StoreRunButton(B_USER_ON_OFF.on_off);
        break;
    case MnlButton:
        ManualModeButton(ID, on_off);
        break;
    case MnlState:
        ManualStateSwitch(ID);
        break;
    case SetFlows:
    {
        String tempStr = "Flow -";
        if (I_PERM_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 0], HMI.data[ID + 1]), Sensor::AlarmMin))
        {
            SET.StoreMinPermFlow(I_PERM_FLOW.minAlarmVal);
            tempStr += (" Product Min: " + String(I_PERM_FLOW.minAlarmVal) + " LPH");
        }
        if (I_PERM_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 2], HMI.data[ID + 3]), Sensor::AlarmMax))
        {
            SET.StoreMaxPermFlow(I_PERM_FLOW.maxAlarmVal);
            tempStr += (" Product Max: " + String(I_PERM_FLOW.maxAlarmVal) + " LPH");
        }
        if (I_RECYCLE_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 4], HMI.data[ID + 5]), Sensor::AlarmMin))
        {
            SET.StoreMinRecycFlow(I_RECYCLE_FLOW.minAlarmVal);
            tempStr += (" Product Min: " + String(I_RECYCLE_FLOW.minAlarmVal) + " LPH");
        }
        if (I_RECYCLE_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 6], HMI.data[ID + 7]), Sensor::AlarmMax))
        {
            SET.StoreMaxRecycFlow(I_RECYCLE_FLOW.maxAlarmVal);
            tempStr += (" Product Max: " + String(I_RECYCLE_FLOW.maxAlarmVal) + " LPH");
        }
        if (I_BRINE_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 8], HMI.data[ID + 9]), Sensor::AlarmMin))
        {
            SET.StoreMinBrineFlow(I_BRINE_FLOW.minAlarmVal);
            tempStr += (" Concentrate Min: " + String(I_BRINE_FLOW.minAlarmVal) + " LPH");
        }
        if (I_BRINE_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 10], HMI.data[ID + 11]), Sensor::AlarmMax))
        {
            SET.StoreMaxBrineFlow(I_BRINE_FLOW.maxAlarmVal);
            tempStr += (" Concentrate Max: " + String(I_BRINE_FLOW.maxAlarmVal) + " LPH");
        }
        if (I_PERM_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 12], HMI.data[ID + 13]), Sensor::FaultMin))
        {
            SET.StoreMinFaultPermFlow(I_PERM_FLOW.minFaultVal);
            tempStr += (" Product Min: " + String(I_PERM_FLOW.minAlarmVal) + " LPH");
        }
        if (I_PERM_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 14], HMI.data[ID + 15]), Sensor::FaultMax))
        {
            SET.StoreMaxFaultPermFlow(I_PERM_FLOW.maxFaultVal);
            tempStr += (" Product Max: " + String(I_PERM_FLOW.maxAlarmVal) + " LPH");
        }
        if (I_BRINE_FLOW.setMinMax(joinLsbMsb(HMI.data[ID + 16], HMI.data[ID + 17]), Sensor::FaultMin))
        {
            SET.StoreMinFaultBrineFlow(I_BRINE_FLOW.minFaultVal);
            tempStr += (" Concentrate Min: " + String(I_BRINE_FLOW.minAlarmVal) + " LPH");
        }
        if (tempStr.length() > 15)
            Blynk.logEvent(BL_SETTINGS, tempStr);
        break;
    }
    case SetSensors:
    {
        String tempStr = "Sensor -"; // Blynk.logEvent(BL_SETTINGS, "Sensors: " + String(HMI.data, HEX).substring(0, 17));
        if (VAL_RECOVERY.setMinMax(joinLsbMsb(HMI.data[ID], HMI.data[ID + 1]) * 10, false))
        {
            I_RECOVERY.setMinMax(VAL_RECOVERY.minVal, Sensor::AlarmMin);
            SET.StoreRecoveryMinVal(VAL_RECOVERY.minVal);
            tempStr += (" Recovery Min: " + String(VAL_RECOVERY.minVal / 10) + "%");
        }
        if (VAL_RECOVERY.setMinMax(joinLsbMsb(HMI.data[ID + 2], HMI.data[ID + 3]) * 10, true))
        {
            I_RECOVERY.setMinMax(VAL_RECOVERY.maxVal, Sensor::AlarmMax);
            SET.StoreRecoveryMaxVal(VAL_RECOVERY.maxVal);
            tempStr += (" Recovery Max: " + String(VAL_RECOVERY.maxVal / 10) + "%");
        }
        if (I_EC_PERM.setMinMax(joinLsbMsb(HMI.data[ID + 8], HMI.data[ID + 9]), Sensor::AlarmMax))
        {
            SET.StoreEC_AlarmVal(I_EC_PERM.maxAlarmVal);
            tempStr += (" EC Warning: " + String(I_EC_PERM.maxAlarmVal) + " mV");
        }
        if (I_EC_PERM.setMinMax(joinLsbMsb(HMI.data[ID + 10], HMI.data[ID + 11]), Sensor::FaultMax))
        {
            SET.StoreEC_FaultVal(I_EC_PERM.maxFaultVal);
            tempStr += (" EC Fault: " + String(I_EC_PERM.maxFaultVal) + " mV");
        }
        if (tempStr.length() > 15)
            Blynk.logEvent(BL_SETTINGS, tempStr);
        break;
    }
    case SetTimes:
    {
        String tempStr = "Times -";
        // Blynk.logEvent(BL_SETTINGS, "Times: " + String(HMI.data, HEX).substring(0, 3));
        uint tempTime = joinLsbMsb(HMI.data[ID], HMI.data[ID + 1]); //* 60;
        if (tempTime != SM.StartFlushTimeSecs)
        {
            SM.StartFlushTimeSecs = tempTime;
            SET.StoreStartFlushTime(SM.StartFlushTimeSecs);
            tempStr += (" Start Flush: " + String(joinLsbMsb(HMI.data[ID], HMI.data[ID + 1])) + " min");
        }
        tempTime = joinLsbMsb(HMI.data[ID + 2], HMI.data[ID + 3]); //* 60;
        if (tempTime != SM.StopFlushTimeSecs)
        {
            SM.StopFlushTimeSecs = tempTime;
            SET.StoreStopFlushTime(SM.StopFlushTimeSecs);
            tempStr += (" Stop Flush: " + String(joinLsbMsb(HMI.data[ID + 2], HMI.data[ID + 3])) + " min");
        }
        if (tempStr.length() > 15)
            Blynk.logEvent(BL_SETTINGS, tempStr);
        break;
    }
    case SetPressures:
    {
        String tempStr = "Pressures -";
        // Blynk.logEvent(BL_SETTINGS, "Pressures: " + String(HMI.data, HEX).substring(0, 5));
        if (I_FEED_PRESS.setMinMax(joinLsbMsb(HMI.data[ID], HMI.data[ID + 1]), Sensor::FaultMin))
        {
            SET.StoreMinFeedPressure(I_FEED_PRESS.minFaultVal);
            tempStr += ("Feed Pressure Fault: " + String((float)I_FEED_PRESS.minFaultVal / 10, 1) + " bar");
        }
        if (I_HP_PRESS.setMinMax(joinLsbMsb(HMI.data[ID + 2], HMI.data[ID + 3]), Sensor::FaultMax))
        {
            SET.StoreMaxMemPressure(I_HP_PRESS.maxFaultVal);
            tempStr += ("HPP Pressure Fault: " + String((float)I_HP_PRESS.maxFaultVal / 10, 1) + " bar");
        }
        if (I_DELTA_PRESS.setMinMax(joinLsbMsb(HMI.data[ID + 4], HMI.data[ID + 5]), Sensor::AlarmMax))
        {
            SET.StoreWarnDeltaPressure(I_DELTA_PRESS.maxAlarmVal);
            tempStr += ("Delta Pressure Warning: " + String((float)I_DELTA_PRESS.maxAlarmVal / 10, 1) + " bar");
        }
        if (I_DELTA_PRESS.setMinMax(joinLsbMsb(HMI.data[ID + 6], HMI.data[ID + 7]), Sensor::FaultMax))
        {
            SET.StoreFaultDeltaPressure(I_DELTA_PRESS.maxFaultVal);
            tempStr += ("Delta Pressure Fault: " + String((float)I_DELTA_PRESS.maxFaultVal / 10, 1) + " bar");
        }
        if (tempStr.length() > 15)
            Blynk.logEvent(BL_SETTINGS, tempStr);
        break;
    }
    case ResetFault:
        ResetFaults();
        break;
    case HandleWarning:
        WriteOutput(WARNING_LIGHT, ID);
        if (!ID)
            ResetFaults();
        break;
    }
    return true;
    // HMI.PrintPage();
}

void ResetFaults(void)
{
    if (SM.faultRelayFault && !SM.faultRelay_flag)
    {
        SM.faultRelayFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.overPressFault && !I_HP_PRESS.fault)
    {
        SM.overPressFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.deltaPressFault && !I_DELTA_PRESS.fault)
    {
        SM.deltaPressFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.permeateFlowFault)
    {
        SM.permeateFlowFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.brineFlowFault)
    {
        SM.brineFlowFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.EC_MaxFault)
    {
        SM.EC_MaxFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
    if (SM.feedPressFault)
    {
        SM.feedPressFault = false;
        WriteOutput(WARNING_LIGHT, false);
    }
}

void SendHMI_SettingValues(void)
{
    HMI.SendSetting("n10", I_PERM_FLOW.minAlarmVal);
    HMI.SendSetting("n11", I_PERM_FLOW.maxAlarmVal);
    HMI.SendSetting("n12", I_RECYCLE_FLOW.minAlarmVal);
    HMI.SendSetting("n13", I_RECYCLE_FLOW.maxAlarmVal);
    HMI.SendSetting("n14", I_BRINE_FLOW.minAlarmVal);
    HMI.SendSetting("n15", I_BRINE_FLOW.maxAlarmVal);
    HMI.SendSetting("n16", I_PERM_FLOW.minFaultVal);
    HMI.SendSetting("n17", I_PERM_FLOW.maxFaultVal);
    HMI.SendSetting("n18", I_BRINE_FLOW.minFaultVal);

    HMI.SendSetting("n20", VAL_RECOVERY.minVal / 10);
    HMI.SendSetting("n21", VAL_RECOVERY.maxVal / 10);
    HMI.SendSetting("n22", I_EC_PERM.maxAlarmVal);
    HMI.SendSetting("n23", I_EC_PERM.maxFaultVal);

    HMI.SendSetting("n30", SM.StartFlushTimeSecs);
    HMI.SendSetting("n31", SM.StopFlushTimeSecs);

    HMI.SendSetting("n40", I_FEED_PRESS.minFaultVal);
    HMI.SendSetting("n41", I_HP_PRESS.maxFaultVal);
    HMI.SendSetting("n42", I_DELTA_PRESS.maxAlarmVal);
    HMI.SendSetting("n43", I_DELTA_PRESS.maxFaultVal);
    // PVAL_RO_PUMP.frequency.maxVal = 500;
}

void ManualModeButton(uint button, bool on_off)
{
    // 0-4 is the valves
    // 10 is the motor/pump
    switch (button)
    {
    case 1:
        WriteOutput(INLET_VALVE, on_off);
        I_INLET_VALVE.ChangeStatus((Status_t)on_off);
        break;
    case 2:
        WriteOutput(FLUSH_VALVE, on_off);
        I_FLUSH_VALVE.ChangeStatus((Status_t)on_off);
        break;
    case 11:
        WriteOutput(HPP_START_STOP, !OutputStatus[HPP_START_STOP]); // Get current status of the output and toggle
        // WriteOutput(HPP_START_STOP, on_off);
        // // TODO check if below is needed? shouldnt be...
        // // if (HPP_VSD.PumpState != ModbusVSD::PumpDisconnect && HPP_VSD.PumpState != ModbusVSD::PumpFault)
        // //     I_RO_PUMP.ChangeStatus((Status_t)on_off);
        break;
    default:
        debugln("mnlButErr");
        break;
    }
    // HMI.PrintPage();
}
void ManualStateSwitch(uint state)
{

    switch (state)
    {
    case 0: // Stop State
        StopAll();
        break;
    case 1: // Flush State
        StartFlush();
        break;
    case 2: // Service state
        StartService();
        break;
    }
    HMI.PrintPage();
}

// NVS FILE SYSTEM
void Save::initialise(void)
{
    debugln("initialising preferences");
    format();
    saveExist(); // shows settings exist when true
    StoreRunButton(false);
    StoreMinPermFlow(DEF_PF_MIN);
    StoreMaxPermFlow(DEF_PF_MAX);
    StoreMinRecycFlow(DEF_RF_MIN);
    StoreMaxRecycFlow(DEF_RF_MAX);
    StoreMinBrineFlow(DEF_BF_MIN);
    StoreMaxBrineFlow(DEF_BF_MAX);
    StoreMinFaultPermFlow(DEF_PF_FAULT_MIN);
    StoreMaxFaultPermFlow(DEF_PF_FAULT_MAX);
    StoreMinFaultBrineFlow(DEF_BF_FAULT_MIN);

    StoreRecoveryMinVal(DEF_REC_MIN);
    StoreRecoveryMaxVal(DEF_REC_MAX);
    StoreEC_AlarmVal(DEF_EC_MAX);
    StoreEC_FaultVal(DEF_EC_FAULT_MAX);

    StoreStartFlushTime(DEF_FLUSH_TIME);
    StoreStopFlushTime(DEF_FLUSH_TIME);

    StoreMinFeedPressure(DEF_FP_FAULT);
    StoreMaxMemPressure(DEF_HP_FAULT);
    StoreWarnDeltaPressure(DEF_DP_WARNING);
    StoreFaultDeltaPressure(DEF_DP_FAULT);

    StoreRO_PumpMins(HPP_VSD.pumpMin);
    StoreBoostPumpMins(BOOST_VSD.pumpMin);
    StorePermVol(PermeateVM.Volume_L);
}
void Save::getAllSettings(void) // get all saved settings
{
    B_USER_ON_OFF.on_off = getBoolIfExist(u_but_k, false);
    SM.runButton = B_USER_ON_OFF.on_off;

    I_PERM_FLOW.minAlarmVal = getUshortIfExist(minpf_k, DEF_PF_MIN);
    I_PERM_FLOW.maxAlarmVal = getUshortIfExist(maxpf_k, DEF_PF_MAX);
    I_RECYCLE_FLOW.minAlarmVal = getUshortIfExist(minrf_k, DEF_RF_MIN);
    I_RECYCLE_FLOW.maxAlarmVal = getUshortIfExist(maxrf_k, DEF_RF_MAX);
    I_BRINE_FLOW.minAlarmVal = getUshortIfExist(minbf_k, DEF_BF_MIN);
    I_BRINE_FLOW.maxAlarmVal = getUshortIfExist(maxbf_k, DEF_BF_MAX);
    I_PERM_FLOW.minFaultVal = getUshortIfExist(mnpfF_k, DEF_PF_FAULT_MIN);
    I_PERM_FLOW.maxFaultVal = getUshortIfExist(mxpfF_k, DEF_PF_FAULT_MAX);
    I_BRINE_FLOW.minFaultVal = getUshortIfExist(mnbfF_k, DEF_BF_FAULT_MIN);

    VAL_RECOVERY.minVal = getUshortIfExist(R_min_k, DEF_REC_MIN);
    I_RECOVERY.minAlarmVal = VAL_RECOVERY.minVal;
    VAL_RECOVERY.maxVal = getUshortIfExist(R_max_k, DEF_REC_MAX);
    I_RECOVERY.maxAlarmVal = VAL_RECOVERY.maxVal;
    I_EC_PERM.maxAlarmVal = getUshortIfExist(ECalm_k, DEF_EC_MAX);
    I_EC_PERM.maxFaultVal = getUshortIfExist(ECflt_k, DEF_EC_FAULT_MAX);

    SM.StartFlushTimeSecs = getUshortIfExist(start_k, DEF_FLUSH_TIME);
    SM.StopFlushTimeSecs = getUshortIfExist(stopf_k, DEF_FLUSH_TIME);

    I_FEED_PRESS.minFaultVal = getUshortIfExist(mP_fI_k, DEF_FP_FAULT);
    I_HP_PRESS.maxFaultVal = getUshortIfExist(mP_MI_k, DEF_HP_FAULT);
    I_DELTA_PRESS.maxAlarmVal = getUshortIfExist(mPwdP_k, DEF_DP_WARNING);
    I_DELTA_PRESS.maxFaultVal = getUshortIfExist(mPfdP_k, DEF_DP_FAULT);

    HPP_VSD.pumpMin = getUIntIfExist(pMin_k, 0);
    PermeateVM.Volume_L = getUIntIfExist(volP_k, 0);
    debugln("Free Entries: " + String(freeEntries(), DEC));
}
bool Save::getBoolIfExist(const char *key, bool defVal)
{
    if (settings.isKey(key))
        return settings.getBool(key);
    else
        settings.putBool(key, defVal);
    debugln(String(key) + " not found");
    return defVal;
}
uint16_t Save::getUshortIfExist(const char *key, uint16_t defVal)
{
    if (settings.isKey(key))
        return settings.getUShort(key);
    else
        settings.putUShort(key, defVal);
    debugln(String(key) + " not found");
    return defVal;
}
uint Save::getUIntIfExist(const char *key, uint defVal)
{
    if (settings.isKey(key))
        return settings.getUInt(key);
    else
        settings.putUInt(key, defVal);
    debugln(String(key) + " not found");
    return defVal;
}

void Save::FileSetup(void)
{
    if (begin()) // open in read/write mode
    {
        if (exists())         // checks if the setting instance has been created
            getAllSettings(); // retrieve all settings
        else                  // format and set defaults when settings instance is not found
            initialise();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    else
        debug("NVS didnt open");
}

void StateMachineRO::RunStateMachine(void)
{
    if (state != ST_MANUAL)
    {
        if (HMI.currPage == P_MANUAL)
            ChangeState(ST_MANUAL);
        else
            CheckWarningsFaults();
    }

    switch (SM.state)
    {
    case ST_STOPPED:
        if (runButton)
        {
            ChangeState(ST_START_FLUSH);
            StartFlushChecks();
        }
        break;
    case ST_START_FLUSH:
        if (SecNow == StartFlushTimeSecs)
            ChangeState(ST_SERVICE);
        else
            HMI.UpdateCountdown(StartFlushTimeSecs - SecNow);
        StartFlushChecks();
        SecNow++;
        break;
    case ST_SERVICE:
        InServiceChecks(); // only way to leave here is with the checks failing
        break;
    case ST_STOP_FLUSH:
        if (SecNow == StopFlushTimeSecs)
            ChangeState(SM.stateAfterFlush);
        else
            HMI.UpdateCountdown(StopFlushTimeSecs - SecNow);
        StopFlushChecks();
        SecNow++;
        break;
    case ST_TANK_FULL:
        if (!productFloat_flag)
            ChangeState(ST_STOPPED);
        break;
    case ST_PREFILTER:
        if (!backwashing_flag)
            ChangeState(ST_STOPPED);
        break;
    case ST_VSD_FAULT:
        if (HPP_VSD.Fault == 0)
            ChangeState(ST_STOPPED);
        break;
    case ST_FEED_EMPTY:
        if (!feedFloatFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_EXT_FAULT:
        if (!faultRelayFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_OVER_P_FAULT:
        if (!overPressFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_D_P_FAULT:
        if (!deltaPressFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_PROD_FLOW_FAULT:
        if (!permeateFlowFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_BRINE_FLOW_FAULT:
        if (!brineFlowFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_EC_FAULT:
        if (!EC_MaxFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_FEED_P_FAULT:
        if (!feedPressFault)
            ChangeState(ST_STOPPED);
        break;
    case ST_MANUAL:
        if (HMI.currPage != P_MANUAL)
            ChangeState(ST_STOPPED);
        break;
    default:
        break;
    }
}
void StateMachineRO::CheckWarningsFaults(void)
{
    Warn_t type = W_High;
    //  WARNINGS
    // if (FeedFlowWarning.CheckIfWarningTriggered(I_FEED_FLOW.alarm))
    // {
    //     if (I_FEED_FLOW.checkAlarmMin())
    //         type = W_Low;
    //     HMI.AddWarning(FeedFlowWarning, type);
    // }
    if (PermFlowWarning.CheckIfWarningTriggered(I_PERM_FLOW.alarm))
    {
        if (I_PERM_FLOW.checkAlarmMin())
            type = W_Low;
        HMI.AddWarning(PermFlowWarning, type);
    }
    if (XFlowWarning.CheckIfWarningTriggered(I_RECYCLE_FLOW.alarm))
    {
        if (I_RECYCLE_FLOW.checkAlarmMin())
            type = W_Low;
        HMI.AddWarning(XFlowWarning, type);
    }
    if (BrineFlowWarning.CheckIfWarningTriggered(I_BRINE_FLOW.alarm))
    {
        if (I_BRINE_FLOW.checkAlarmMin())
            type = W_Low;
        HMI.AddWarning(BrineFlowWarning, type);
    }
    if (EC_Warning.CheckIfWarningTriggered(I_EC_PERM.alarm))
    {
        if (I_EC_PERM.checkAlarmMin())
            type = W_Low;
        HMI.AddWarning(EC_Warning, type);
    }
    if (PumpSpeedWarning.CheckIfWarningTriggered(state == ST_SERVICE && PVAL_RO_PUMP.frequency.checkMax()))
    {
        debugln("pump freq: " + String(PVAL_RO_PUMP.frequency.val) + " max: " + String(PVAL_RO_PUMP.frequency.maxVal));
        HMI.AddWarning(PumpSpeedWarning, W_None);
        HMI.ActivateCIPWarning();
    }
    if (RecoveryWarning.CheckIfWarningTriggered(VAL_RECOVERY.alarm))
    {
        if (VAL_RECOVERY.checkMin())
            type = W_Low;
        HMI.AddWarning(RecoveryWarning, type);
    }
    if (dPressWarning.CheckIfWarningTriggered(I_DELTA_PRESS.alarm))
    {
        HMI.AddWarning(dPressWarning, W_None);
        HMI.ActivateCIPWarning();
    }
    // if (FeedTankWarning.CheckIfWarningTriggered(feedFloat_flag))
    // {
    //     feedFloatFault = true;
    //     // HMI.AddFault(FeedTankWarning, W_None);
    //     HMI.AddWarning(FeedTankWarning, W_None); // Changed to warning to allow a reset
    // }

    // FAULTS
    if (PermFlowFault.CheckIfWarningTriggered(I_PERM_FLOW.fault))
    {
        if (I_PERM_FLOW.checkFaultMin())
            type = W_Low;
        HMI.AddFault(PermFlowFault, type);
        permeateFlowFault = true;
    }
    if (BrineFlowFault.CheckIfWarningTriggered(I_BRINE_FLOW.fault))
    {
        HMI.AddFault(BrineFlowFault, W_None);
        brineFlowFault = true;
    }
    if (EC_Fault.CheckIfWarningTriggered(I_EC_PERM.fault))
    {
        HMI.AddFault(EC_Fault, W_None);
        EC_MaxFault = true;
    }
    if (FeedPressFault.CheckIfWarningTriggered(I_FEED_PRESS.fault))
    {
        HMI.AddFault(FeedPressFault, type);
        feedPressFault = true;
    }
    if (MemPressFault.CheckIfWarningTriggered(I_HP_PRESS.fault))
    {
        overPressFault = true;
    }
    if (dPressFault.CheckIfWarningTriggered(I_DELTA_PRESS.fault))
    {
        deltaPressFault = true;
    }
    if (ExternalFault.CheckIfWarningTriggered(faultRelay_flag))
    {
        faultRelayFault = true;
    }
}
void StateMachineRO::ChangeState(state_t nextState)
{
    state = nextState;
    SecNow = 0;
    HMI.RemoveCountdown();
    switch (state)
    {
    case ST_START_FLUSH:
        StartFlush();
        break;
    case ST_SERVICE:
        StartService();
        break;
    case ST_STOP_FLUSH:
        StartFlush();
        break;
    default:
        StopAll();
        switch (state)
        {
        case ST_STOPPED:
            WriteOutput(WARNING_LIGHT, false);
            break;
        case ST_TANK_FULL:
            break;
        case ST_PREFILTER:
            break;
        case ST_VSD_FAULT:
            HMI.AddFault(VSD_Fault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Pump VSD: F" + String(HPP_VSD.Fault));
            break;
        case ST_FEED_EMPTY:
            // WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Feed Tank Low");
            break;
        case ST_EXT_FAULT:
            HMI.AddFault(ExternalFault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "External Fault Input");
            break;
        case ST_OVER_P_FAULT:
            HMI.AddFault(MemPressFault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Over Max Pressure" + String((float)I_HP_PRESS.value / 10, 1) + " bar");
            break;
        case ST_D_P_FAULT:
            HMI.AddFault(dPressFault);
            HMI.ActivateCIPWarning();
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Max Delta Pressure " + String((float)I_DELTA_PRESS.value / 10, 1) + " bar");
            break;
        case ST_PROD_FLOW_FAULT:
            HMI.AddFault(PermFlowFault, (I_PERM_FLOW.checkFaultMin() ? W_Low : W_High));
            // HMI.ActivateCIPWarning();    TODO check if this should be here
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Permeate Flow " + String(I_PERM_FLOW.value) + " LPH");
            break;
        case ST_BRINE_FLOW_FAULT:
            HMI.AddFault(BrineFlowFault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Check Brine Valve - Flow Too Low");
            break;
        case ST_EC_FAULT:
            HMI.AddFault(EC_Fault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "EC High Fault");
            break;
        case ST_FEED_P_FAULT:
            HMI.AddFault(FeedPressFault);
            WriteOutput(WARNING_LIGHT, true);
            Blynk.logEvent(BL_FAULT, "Max Membrane Delta Pressure");
            break;
        case ST_MANUAL:
            runButton = false;
            B_USER_ON_OFF.on_off = runButton;
            HMI.SendButtonPicVal(B_USER_ON_OFF);
            // SET.StoreRunButton(B_USER_ON_OFF.on_off);
            WriteOutput(WARNING_LIGHT, false);
            break;
        default:
            debug("Error non existent state:");
            debugln(state);
            break;
        }
        break;
    }

    if (HMI.currPage != P_NONE)
    {
        if (state != ST_STOP_FLUSH)
            HMI.SendStateStr(stateStr[state]);
        else
        {
            if (stateAfterFlush == ST_STOPPED)
                HMI.SendStateStr((String(stateStr[state]) + " - User").c_str());
            else if (stateAfterFlush == ST_TANK_FULL)
                HMI.SendStateStr((String(stateStr[state]) + " - Tank").c_str());
            else if (stateAfterFlush == ST_PREFILTER)
                HMI.SendStateStr((String(stateStr[state]) + " - PreF").c_str());
            else
                HMI.SendStateStr((String(stateStr[state]) + " - Fault").c_str());
        }
        HMI.PrintPage(ONLY_CHANGES);
    }
    Blynk.logEvent(BL_STATE_CHANGE, stateStr[state]);
    Blynk.virtualWrite(B_STATE, stateStr[state]);
}
void StateMachineRO::StartFlushChecks(void)
{
    if (!runButton)
        ChangeState(ST_STOPPED);
    if (feedFloatFault)
        ChangeState(ST_FEED_EMPTY);
    if (faultRelayFault)
        ChangeState(ST_EXT_FAULT);
    if (backwashing_flag)
        ChangeState(ST_PREFILTER);
    if (productFloat_flag)
        ChangeState(ST_TANK_FULL);
    if (overPressFault)
        ChangeState(ST_OVER_P_FAULT);
    if (deltaPressFault)
        ChangeState(ST_D_P_FAULT);
    if (vsdFault)
        ChangeState(ST_VSD_FAULT);
}
void StateMachineRO::InServiceChecks(void)
{
    stateAfterFlush = ST_STOP_FLUSH;

    if (feedFloatFault)
        stateAfterFlush = ST_FEED_EMPTY;
    else if (faultRelayFault)
        stateAfterFlush = ST_EXT_FAULT;
    else if (overPressFault)
        stateAfterFlush = ST_OVER_P_FAULT;
    else if (deltaPressFault)
        stateAfterFlush = ST_D_P_FAULT;
    else if (vsdFault)
        ChangeState(ST_VSD_FAULT);
    else if (backwashing_flag)
        stateAfterFlush = ST_PREFILTER;
    else if (productFloat_flag)
        stateAfterFlush = ST_TANK_FULL;
    else if (!runButton)
        stateAfterFlush = ST_STOPPED;
    else if (permeateFlowFault)
        stateAfterFlush = ST_PROD_FLOW_FAULT;
    else if (brineFlowFault)
        stateAfterFlush = ST_BRINE_FLOW_FAULT;
    else if (EC_MaxFault)
        stateAfterFlush = ST_EC_FAULT;
    else if (feedPressFault)
        stateAfterFlush = ST_FEED_P_FAULT;

    if (stateAfterFlush != ST_STOP_FLUSH)
        ChangeState(ST_STOP_FLUSH);
}
void StateMachineRO::StopFlushChecks(void)
{
}

bool cbHPP_ReadHreg(Modbus::ResultCode event, uint16_t transactionId, void *data)
{ // Callback from Modbus to monitor errors
    if (event != Modbus::EX_SUCCESS)
    {
        debug("Request result: 0x");
        debugBase(event, HEX);
        debug("\tMB state: ");
        debugln(HPP_VSD.MBstate);

        HPP_VSD.PumpState = ModbusVSD::PumpDisconnect;
        HPP_VSD.IncrementVSD_Index();
        // EM_VSD.MBstate = ModbusVSD::MB_Idle;
    }
    else if (HPP_VSD.PumpState == ModbusVSD::PumpDisconnect)
    {
        HPP_VSD.PumpState = ModbusVSD::PumpStop;
    }
    // debugln("MB_callback");
    // debugln();
    return true;
}

bool cbBoosterReadHreg(Modbus::ResultCode event, uint16_t transactionId, void *data)
{ // Callback from Modbus to monitor errors
    if (event != Modbus::EX_SUCCESS)
    {
        debug("Request result: 0x");
        debugBase(event, HEX);
        debug("\tMB state: ");
        debugln(BOOST_VSD.MBstate);

        BOOST_VSD.PumpState = ModbusVSD::PumpDisconnect;
        BOOST_VSD.IncrementVSD_Index();
        // EM_VSD.MBstate = ModbusVSD::MB_Idle;
    }
    else if (BOOST_VSD.PumpState == ModbusVSD::PumpDisconnect)
    {
        BOOST_VSD.PumpState = ModbusVSD::PumpStop;
    }
    // debugln("MB_callback");
    // debugln();
    return true;
}

void RunStateMachine(void) { SM.RunStateMachine(); };
void TimerSetup(void)
{
    edgentTimer.setInterval(1000, RunStateMachine);
    edgentTimer.setInterval(10, CheckInputs);
    edgentTimer.setInterval(500, CheckSensors);
    edgentTimer.setInterval(1000, UpdateHMI);
    edgentTimer.setInterval(86400000, RequestBlynkTimeUpdate); // update time daily
    Blynk.logEvent(BL_STATE_CHANGE, "Controller Startup");
    CheckInputs();
    CheckSensors();
    SM.ChangeState(ST_STOPPED);
}
void RunAnalog(ADS1115 &ADC_ref)
{
    // if(ADC_ref.isConnected()&&ADC_ref)
    // {
    // }
}
void BlynkLoop(void *pvParameters)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    BlynkEdgent.begin();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    for (;;) // loop
    {
        BlynkEdgent.run();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
void RequestBlynkTimeUpdate(void)
{
    Blynk.sendInternal("utc", "time"); // Unix timestamp (with msecs)
}

BLYNK_CONNECTED()
{
    Blynk.sendInternal("utc", "time");    // Unix timestamp (with msecs)
    Blynk.sendInternal("utc", "tz_rule"); // POSIX TZ rule
}
BLYNK_WRITE(InternalPinUTC)
{
    String cmd = param[0].asStr();
    if (cmd == "time")
    {
        const uint64_t utc_time = param[1].asLongLong();
        UTC.setTime(utc_time / 1000, utc_time % 1000);
        // Serial.print("Unix time (UTC): "); Serial.println(utc_time);
    }
    else if (cmd == "tz_rule")
    {
        String tz_rule = param[1].asStr();
        local.setPosix(tz_rule);
        // Serial.print("POSIX TZ rule:   "); Serial.println(tz_rule);
    }
}

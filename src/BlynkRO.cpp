#include "BlynkRO.h"
#include <BlynkEdgent.h>

// char B_State[128] = {};
// char B_Warning[128] = {};
// char B_Fault[128] = {};
char B_Setting[128] = {};

void SendToBlynk(void)
{

    static uint StackFree = UINT_MAX;
    if (StackFree > uxTaskGetStackHighWaterMark(BlynkTH))
    {
        StackFree = uxTaskGetStackHighWaterMark(BlynkTH);
        Blynk.logEvent(BL_SETTINGS, "STACK_HW_MARK: " + String(StackFree));
        Serial.println("STACK_HW_MARK: " + String(StackFree));
    }

    // Sensors
    if (xSemaphoreTake(xSemaphore, (10 * portTICK_PERIOD_MS))) // TODO need to give semaphore?
    {
        // Update connected status
        I_WIFI.ChangeStatus((Status_t)Blynk.connected());

        // SEND SENSOR DATA TO BLYNK
        Blynk.virtualWrite(B_EC, SM.EC);
        Blynk.virtualWrite(B_HP_IN_PRESS, SM.HPP_InletPressure);

        Blynk.virtualWrite(B_HP_OUT_PRESS, SM.HP_PumpPressure);
        Blynk.virtualWrite(B_POST_MEM_PRESS, SM.PostMemPressure);
        Blynk.virtualWrite(B_DELTA_PRESS, SM.DeltaMemPressure);
        Blynk.virtualWrite(B_BOOST_PRESS, SM.BoostPumpPressure);
        Blynk.virtualWrite(B_FEED_PUMP_PRESS, SM.FeedPumpPressure);
        
        Blynk.virtualWrite(B_PRODUCT_VOLUME, PermeateVM.Volume_m3);
        Blynk.virtualWrite(B_TEMP, AmbientTemp.currTemp);

        CheckPumpStates();
        UpdateBlynkWithInputs();

        Blynk.virtualWrite(B_FEED_FLOW, SM.FeedFlow);
        Blynk.virtualWrite(B_PERM_FLOW, PermeatePM.FlowRate);
        Blynk.virtualWrite(B_RECYCLE_FLOW, RecyclePM.FlowRate);
        Blynk.virtualWrite(B_BRINE_FLOW, BrinePM.FlowRate);

        Blynk.virtualWrite(B_RECOVERY, SM.Recovery);

        if (B_Setting[0] != 0)
        {
            Blynk.logEvent(BL_SETTINGS, B_Setting);
            B_Setting[0] = 0;
        }

        CheckForStateChange(SM.logBlynkStateChange);
    }
}
void CheckPumpStates(void)
{
    static ModbusVSD::PumpState_t HP_PumpState = ModbusVSD::PumpConnect;
    if (HP_PumpState != HPP_VSD.PumpState)
    {
        HP_PumpState = HPP_VSD.PumpState;
        SendPumpState(HP_PumpState, B_HP_PUMP);
    }
    static ModbusVSD::PumpState_t Boost_PumpState = ModbusVSD::PumpConnect;
    if (Boost_PumpState != BOOST_VSD.PumpState)
    {
        Boost_PumpState = BOOST_VSD.PumpState;
        SendPumpState(Boost_PumpState, B_BOOST_PUMP);
    }
    static ModbusVSD::PumpState_t FP_PumpState = ModbusVSD::PumpConnect;
    if (FP_PumpState != FP_VSD.PumpState)
    {
        FP_PumpState = FP_VSD.PumpState;
        SendPumpState(FP_PumpState, B_BOOST_PUMP);
    }
}
void UpdateBlynkWithInputs(void)
{
    Blynk.virtualWrite(B_FEED_TANK, SM.feedFloat_flag);
    Blynk.virtualWrite(B_PRODUCT_TANK, SM.productFloat_flag);
    Blynk.virtualWrite(B_PREFILTER_LOCKOUT, SM.backwashing_flag);
    Blynk.virtualWrite(B_EXTERNAL_FAULT, SM.faultRelay_flag);
}
void CheckForStateChange(bool &changed)
{
    if (changed)
    {
        changed = false;
        Blynk.virtualWrite(B_STATE, stateStr[SM.state]);
        char tempStr[64] = {'\0'};
        switch (SM.state)
        {
        case ST_VSD_FAULT:
            sprintf(tempStr, "Pump VSD: F%u", (HPP_VSD.Fault == 0 ? BOOST_VSD.Fault : HPP_VSD.Fault));
            break;
        case ST_FEED_EMPTY:
            if (SM.inline_mode)
                Blynk.logEvent(BL_STATE_CHANGE, stateStr[SM.state]);
            else
                sprintf(tempStr, "Feed Tank Low");
            break;
        case ST_EXT_FAULT:
            sprintf(tempStr, "External Fault Input");
            break;
        case ST_OVER_P_FAULT:
            sprintf(tempStr, "Over Max Pressure: %0.2f bar , %0.2f bar", SM.HP_PumpPressure, SM.PostMemPressure);
            break;
        case ST_D_P_FAULT:
            sprintf(tempStr, "High Delta Membrane Pressure: %0.2f bar", SM.DeltaMemPressure);
            break;
        case ST_PROD_FLOW_FAULT:
            sprintf(tempStr, "Product Flow Fault: %0.2f LPH", PermeatePM.FlowRate);
            break;
        case ST_BRINE_FLOW_FAULT:
            sprintf(tempStr, "Brine Flow Too Low: %0.2f LPH", BrinePM.FlowRate);
            break;
        case ST_EC_FAULT:
            sprintf(tempStr, "EC Fault: %0.2f bar", SM.EC);
            break;
        case ST_HPP_INLET_P_FAULT:
            sprintf(tempStr, "Low Feed Pressure: %0.2f bar", SM.HPP_InletPressure);
            break;

        default:
            Blynk.logEvent(BL_STATE_CHANGE, stateStr[SM.state]);
            break;
        }
        if (tempStr[0] != '\0')
        {
            Blynk.logEvent(BL_FAULT, tempStr);
            tempStr[0] = '\0';
        }
    }
}

/* GENERIC FUNCTIONS */
void TimerSetup(void)
{
    // edgentTimer.setInterval(1000, RunStateMachine);
    // edgentTimer.setInterval(10, CheckInputs);
    // edgentTimer.setInterval(500, CheckSensors);
    edgentTimer.setInterval(1000, SendToBlynk);
    edgentTimer.setInterval(86400000, RequestBlynkTimeUpdate); // update time daily
    Blynk.logEvent(BL_STATE_CHANGE, "Controller Startup");
    CheckInputs();
    CheckSensors();
    SM.ChangeState(ST_STOPPED);
}
void BlynkLoop(void *pvParameters)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    BlynkEdgent.begin();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    for (;;) // loop
    {
        BlynkEdgent.run();
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
void SendPumpState(ModbusVSD::PumpState_t state, int BlynkPin)
{
    B_Colour stateColour = GREY;
    switch (state)
    {
    case ModbusVSD::PumpStop:
        stateColour = WHITE;
        break;
    case ModbusVSD::PumpRun:
        stateColour = GREEN;
        break;
    case ModbusVSD::PumpAlarm:
        stateColour = ORANGE;
        break;
    case ModbusVSD::PumpFault:
        stateColour = RED;
        break;
    case ModbusVSD::PumpDisconnect:
        Blynk.logEvent(BL_WARNING, "VSD Disconnected");
        stateColour = GREY;
        break;
    }
    Blynk.setProperty(BlynkPin, "color", '#' + String(stateColour, HEX));
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

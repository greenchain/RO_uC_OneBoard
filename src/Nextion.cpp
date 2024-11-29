#include "Nextion.h"

// ICONS, BUTTONS AND SENSORS
// Button
Button B_USER_ON_OFF(Button_Off, Button_On);
Icon I_WIFI(I_ICON, '0', WifiPics);

// Main page
Icon I_RO_PUMP(I_PUMP, '1', LrgPumpPics);
Icon I_BOOSTER_PUMP(I_PUMP, '2', LrgPumpPics);
Icon I_FEED_PUMP(I_PUMP, '3', SmlPumpPics);
// Icon I_FEED_PUMP(I_PUMP, '3', SmlButtonPics);

Icon I_BW_VALVE(I_BW_HEAD, '1', FRP_Pics);

Icon I_INLET_VALVE(I_VALVE, '1', ValvePics);
Icon I_FLUSH_VALVE(I_VALVE, '2', ValvePics);

Icon I_FEED_FLOAT(I_FLOAT, '0', FloatPics);

Sensor I_FEED_PUMP_PRESS(I_PRESSURE, '5', SensorBkgndPics, ONE_DECIMAL, INT_MIN, INT_MAX, DEF_FP_FAULT, INT_MAX);
Sensor I_HP_INLET_PRESS(I_PRESSURE, '0', SensorBkgndPics, ONE_DECIMAL, INT_MIN, INT_MAX, DEF_FP_FAULT, INT_MAX);
Sensor I_HP_OUTLET_PRESS(I_PRESSURE, '1', SensorBkgndPics, ONE_DECIMAL, INT_MIN, INT_MAX, INT_MIN, DEF_HP_FAULT);
Sensor I_POST_MEM_PRESS(I_PRESSURE, '2', SensorBkgndPics, ONE_DECIMAL);
Sensor I_DELTA_PRESS(I_PRESSURE, '3', SensorBkgndPics, ONE_DECIMAL, INT_MIN, DEF_DP_WARNING, INT_MIN, DEF_DP_FAULT);
Sensor I_BOOSTER_PRESS(I_PRESSURE, '4', SensorBkgndPics);

Sensor I_FEED_FLOW(I_FLOW, '0', SensorBkgndPics, NO_DECIMAL); // THIS is value only for Gauges page
Sensor I_PERM_FLOW(I_FLOW, '1', SensorBkgndPics, NO_DECIMAL, DEF_PF_MIN, DEF_PF_MAX, DEF_PF_FAULT_MIN, DEF_PF_FAULT_MAX);
Sensor I_RECYCLE_FLOW(I_FLOW, '2', SensorBkgndPics, NO_DECIMAL, DEF_RF_MIN, DEF_RF_MAX);
Sensor I_BRINE_FLOW(I_FLOW, '3', SensorBkgndPics, NO_DECIMAL, DEF_BF_MIN, DEF_BF_MAX, DEF_BF_FAULT_MIN);

Sensor I_TEMP(I_SENSOR, '2', SensorBkgndPics, ONE_DECIMAL);
Sensor I_EC_PERM(I_SENSOR, '3', SensorBkgndPics, NO_DECIMAL, INT_MIN, DEF_EC_MAX, INT_MIN, DEF_EC_FAULT_MAX);
Sensor I_RECOVERY(I_OTHER, '5', SensorBkgndPics, ONE_DECIMAL, DEF_REC_MIN, DEF_REC_MAX);

// Gauges
PumpValues PVAL_FEED_PUMP(12, 13, 14, 15, 16);
PumpValues PVAL_RO_PUMP(0, 1, 2, 3, 4);
PumpValues PVAL_BOOSTER_PUMP(7, 8, 9, 10, 11);

Value VAL_RECOVERY('x', 5, ONE_DECIMAL, DEF_REC_MIN, DEF_REC_MAX);
Value VAL_PERM_VOL('x', 6, ONE_DECIMAL);

Value VAL_CalibVoltage('c', 0, NO_DECIMAL);
Value VAL_CalibCurrent('c', 1, NO_DECIMAL);
// Value VAL_CalibSmallPM('c', 2, ONE_DECIMAL);
// Value VAL_CalibLargePM('c', 3, TWO_DECIMAL);
Value VAL_CalibPermPM('c', 2, TWO_DECIMAL);
Value VAL_CalibBrinePM('c', 3, TWO_DECIMAL);
Value VAL_CalibRecyclePM('c', 4, TWO_DECIMAL);

Value VAL_A0_Current('A', 0, ONE_DECIMAL);
Value VAL_A1_Current('A', 1, ONE_DECIMAL);
Value VAL_A2_Current('A', 2, ONE_DECIMAL);
Value VAL_A3_Current('A', 3, ONE_DECIMAL);

Value VAL_A0_Voltage('V', 0, ONE_DECIMAL);
Value VAL_A1_Voltage('V', 1, ONE_DECIMAL);
Value VAL_A2_Voltage('V', 2, ONE_DECIMAL);
Value VAL_A3_Voltage('V', 3, ONE_DECIMAL);

void Nextion::CommsHandler(void)
{
    switch (header)
    {
    case H_PAGE_NUM: //     'p' - Page has changed
        if (ChangeIfPageExists(data[0]))
            PrintPage(RESEND_ALL);
        break;
    case H_USER_BUTTON: //  'u'
        if (currPage == P_MAIN)
        {
            UpdateUserButton(data[0]);
            cbNextion(header, true, data[0]);
        }
        else
            debugln("errorUserBtn");
        break;
    case H_MNL_BUTTON: //   'b' - a button has been pressed on the manual page
        if (currPage == P_MANUAL)
        {
            char id;
            bool on_off;
            if (data[0] == 'm') // if a pump motor
            {
                id = 10 + data[1] - '0';
                // on_off = (data[2] == PumpPics[0] ? true : false);
            }
            else // it's a valve
            {
                id = data[1] - '0';
                on_off = (data[2] == ValvePics[0] ? true : false);
            }
            debugln("pic:" + String(data[2], DEC) + "  ID:" + String(id, DEC) + "  on_off:" + String(on_off, DEC));
            cbNextion(header, id, on_off); // e.g.
            PrintPage(ONLY_CHANGES);
        }
        else
            debugln("errorBtn");
        break;
    case H_MANUAL_STATE:
        if (currPage == P_MANUAL)
            cbNextion(header, data[0], true);
        else
            debugln("Mnl state when not on Mnl Page");
        break;
    case H_ALL_SETTINGS:           // 's'
        cbNextion(0xb1, 0, true);  // flows indexes 0,2,4,6,8,10,12,14,16
        cbNextion(0xb2, 18, true); // sensors indexes 18,20,22,24
        cbNextion(0xb3, 26, true); // times indexes 26,28
        cbNextion(0xb4, 30, true); // pressures indexes 30,32,34,36
        break;
    case H_FLOWS_SET:               // 0xb1
        cbNextion(header, 0, true); // flows indexes 0,2,4,6,8,10,12,14,16
        break;
    case H_SENSORS_SET:             // 0xb2
        cbNextion(header, 0, true); // sensors indexes 0,2,4,6
        break;
    case H_TIME_SET:                // 0xb3
        cbNextion(header, 0, true); // times indexes 0,2
        break;
    case H_PRESS_SET:               // 0xb4 --
        cbNextion(header, 0, true); // pressures indexes 0,2,4,6
        break;
    // case H_SYSTEM_SET: // 0xb5 -- changed these to stay on the nextion (currently just sleep -- 10-05-24)
    //     break;
    case H_WARNING:
        if (data[0] == 'c')
            ClearWarnings();
        else if (data[0] == '+' || data[0] == '-')
            NextWarning(data[0] == '+');
        // cbNextion(header, data[0], true); // can be clear, next warning or previous warning
        break;
    // case H_RTC:
    //     // RTC_Handler(data[0]);
    //     break;
    case H_RESET:
        cbNextion(header, true, true);
        ChangePage(currPage);
        break;
    case H_CALIBRATION:
        cbNextion(header, data[0], (data[1] == '+'));
        break;
    default:
        debugln("Header error");
        break;
    }
}
bool Nextion::ChangeIfPageExists(char page)
{
    bool PageExists = true;
    switch (page)
    {
    case P_SPLASH:
    case P_MAIN:
    case P_MANUAL:
    case P_GAUGES:
    case P_SETTINGS:
    case P_NONE:
    case P_POPUP:
    case P_SLEEP:
    case P_GLOBAL:
    case P_CALIBRATE:
        currPage = (Page_t)page;
        break;
    default:
        PageExists = false;
        debug("ErrPageExist: ");
        debuglnBase(currPage, HEX);
    }
    return PageExists;
}
void Nextion::ChangeCropPic(const char *item, uint8_t picture)
{
    Comms.print(item);
    Comms.print(".picc=");
    Comms.print(picture);
    _endTrans();
}
void Nextion::ChangePic(const char *item, uint8_t picture)
{
    Comms.print(item);
    Comms.print(".pic=");
    Comms.print(picture);
    _endTrans();
}
void Nextion::ChangeValue(const char *item, uint val)
{
    Comms.print(item);
    Comms.print(".val=");
    Comms.print(val);
    _endTrans();
}
void Nextion::ChangeForegroundColour(const char *item, uint colour) // pco
{
    Comms.print(item);
    Comms.print(".pco=");
    Comms.print(colour);
    _endTrans();
}
void Nextion::ChangeBackgroundColour(const char *item, uint colour) // pco
{
    Comms.print(item);
    Comms.print(".bco=");
    Comms.print(colour);
    _endTrans();
}
void Nextion::SendGlobalStr(const char *item, const char *str)
{
    Comms.printf("gbl.%s.txt=\"%s\"", item, str);
    _endTrans();
}
void Nextion::SendGlobalVal(const char *item, uint val)
{
    Comms.printf("gbl.%s.val=%u", item, val);
    _endTrans();
}
void Nextion::ChangeVis(const char *item, bool visible)
{
    Comms.printf("vis %s,%u", item, visible);
    _endTrans();
}
void Nextion::_endTrans(void) // Sends the Nextion required end of transmission
{
    Comms.write(0xff);
    Comms.write(0xff);
    Comms.write(0xff);
}

Nextion::Nextion(uint baud, uint Nextion_TX, uint Nextion_RX)
{
    Comms.begin(baud, SERIAL_8N1, Nextion_TX, Nextion_RX);
    // Comms.print("STARTING");
    _endTrans();
    while (Comms.available()) // clean the serial input buffer out
        Comms.read();         // clear the comms between the Nextion and the uC
}
void Nextion::ChangePage(uint next_page)
{
    Comms.print("page "); // Comms.print(F("page "));
    Comms.print(next_page - 0xa0);
    _endTrans();
}
void Nextion::UpdateUserButton(bool newStatus)
{
    B_USER_ON_OFF.on_off = newStatus;
    SendButtonPicVal(B_USER_ON_OFF);
}
void Nextion::Startup(void) // Display Setup
{
    ulong timeStamp = millis();
    uint i = 0;
    debugln("Starting HMI");
    ChangePage(P_SPLASH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (Comms.available())
        Listen();
    while (currPage != P_SPLASH)
    {
        debug(" .");
        if (millis() - timeStamp > START_UP_TIMEOUT) // no startup before timeout
        {
            i++;
            timeStamp = millis();
            debugln("HMI splash timeout");
            if (i > 2)
            {
                i = 0;
                debugln("Not able to connect to Nextion");
                // ESP.restart();
                break;
            }
            ChangePage(P_SPLASH);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
        if (Comms.available())
            Listen();
        else
            ChangePage(P_SPLASH);
    }
    // if (currPage) // Check if the page is changed or there was a timeout
    // {
    //     cbNextion('$', true, true);
    // }
    while (millis() - timeStamp < SPLASH_PAGE_TIME) // wait for splash page to be displayed for splash time
        vTaskDelay(10 / portTICK_PERIOD_MS);
    debug(millis() - timeStamp);
    ChangePage(P_MAIN);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    timeStamp = millis();
    while (currPage == P_SPLASH) // Wait for the page to change to main and resend the command if page doesn't change in 5 secs
    {
        if (Comms.available())
            Listen();
        else if (millis() - timeStamp >= START_UP_TIMEOUT) // resend main page command after Timeout
        {
            ChangePage(P_MAIN);
            if (i > 2)
            {
                debugln("Page not changing from Splash");
                break;
            }
            timeStamp = millis();
        }
        else
            vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // if (currPage == P_MAIN)
    //     PrintPage();
}
void Nextion::SendStartupValues(void)
{
    SendButtonPicVal(B_USER_ON_OFF);
    SendGlobalVal("cip", false);
    if (WarningCntr != 0)
    {
        WarningIndex = WarningCntr - 1;
        SendCurrWarning();
    }
    SendGlobalStr("ErrT", "");
    SendGlobalVal("W_c", 0);
    SendGlobalVal("W_i", 0);
    lastSentDay = -1;
    lastSentMin = -1;
}
void Nextion::SendSetting(const char *setting, uint settingVal)
{
    Comms.print("set.");
    ChangeValue(setting, settingVal);
}
void Nextion::SendTimeStr(void)
{
    SendGlobalStr("time", local.dateTime(TIME_STRING).c_str());
}
void Nextion::SendDateStr(void)
{
    SendGlobalStr("date", local.dateTime(DATE_STRING).c_str());
}
void Nextion::SendStateStr(const char *state)
{
    SendGlobalStr("state", state);
    // Comms.print("gbl.state.txt=\""); // Comms.print(F("gbl.state.txt="));
    // Comms.print(state);
    // Comms.print(QUOTE);
    // _endTrans();
}
void Nextion::SendTimeValues(void)
{
    UpdateTimeVisibility(true);
    if (lastSentDay != local.day())
    {
        lastSentDay = local.day();
        SendDateStr();
    }
    if (lastSentMin != local.minute())
    {
        lastSentMin = local.minute();
        SendTimeStr();
    }
}
void Nextion::SendButtonPicVal(Button button)
{
    SendGlobalVal("but", button.pic[button.on_off]);
}
void Nextion::SendIconStatus(Icon &icon, bool SkipPrecheck)
{
    if (SkipPrecheck || icon.lastStatus != icon.status)
    {
        char iconID[3] = {icon.type, icon.number, 0};
        ChangePic(iconID, icon.statusPics[icon.status]);
        icon.lastStatus = icon.status;
    }
}
void Nextion::SendSensorValueStatus(Sensor &sensor, bool SkipPrecheck)
{
    SendSensorValue(sensor, SkipPrecheck);
    SendSensorStatus(sensor, SkipPrecheck);
}
void Nextion::SendSensorValue(Sensor &sensor, bool SkipPrecheck)
{
    if (SkipPrecheck || sensor.value != sensor.lastSentValue)
    {
        char sensorID[3] = {sensor.type, sensor.number, 0};
        ChangeValue(sensorID, sensor.value);
        sensor.lastSentValue = sensor.value;
    }
}
void Nextion::SendSensorStatus(Sensor &sensor, bool SkipPrecheck)
{
    char sensorID[3] = {sensor.type, sensor.number, 0};
    if (SkipPrecheck)
    {
        if (sensor.fault)
            ChangeCropPic(sensorID, sensor.pic[(uint)sensor.fault * 2]);
        else
            ChangeCropPic(sensorID, sensor.pic[sensor.alarm]);
    }
    else if (sensor.lastFaultStatus != sensor.fault)
    {
        ChangeCropPic(sensorID, sensor.pic[(uint)sensor.fault * 2]);
        sensor.lastFaultStatus = sensor.fault;
    }
    else if (sensor.lastStatus != sensor.alarm)
    {
        ChangeCropPic(sensorID, sensor.pic[sensor.alarm]);
        sensor.lastStatus = sensor.alarm;
    }
}
void Nextion::SendTankLevelStatus(TankBar &tank, bool SkipPrecheck)
{
    char tankID[3] = {'j', tank.number, 0};
    if (SkipPrecheck || tank.value != tank.lastSentValue)
    {
        ChangeValue(tankID, tank.value);
        tank.lastSentValue = tank.value;
    }
    if (SkipPrecheck || tank.alarm != tank.lastStatus)
    {
        if (tank.alarm)
            ChangeForegroundColour(tankID, tank.c_ALARM);
        else
            ChangeForegroundColour(tankID, tank.c_NORMAL);
        tank.lastStatus = tank.alarm;
    }
}
void Nextion::SendPumpValues(PumpValues &pump, bool SkipPrecheck)
{
    SendValue(pump.hours, SkipPrecheck);
    // SendStatus(pump.hours, SkipPrecheck);
    SendValue(pump.volts, SkipPrecheck);
    // SendStatus(pump.volts, SkipPrecheck);
    SendValue(pump.current, SkipPrecheck);
    // SendStatus(pump.current, SkipPrecheck);
    SendValue(pump.frequency, SkipPrecheck);
    // SendStatus(pump.frequency, SkipPrecheck);
    SendValue(pump.power, SkipPrecheck);
    // SendStatus(pump.power, SkipPrecheck);
}
void Nextion::SendValue(Value &value, bool SkipPrecheck)
{
    if (SkipPrecheck || value.val != value.lastVal)
    {
        char valueID[4] = {value.type, 0, 0, 0};
        if (value.num < 10)
            valueID[1] = value.num + '0';
        else
        {
            valueID[1] = (char)(value.num / 10 + '0');
            valueID[2] = (char)(value.num % 10 + '0');
        }
        ChangeValue(valueID, value.val);
        value.lastVal = value.val;
    }
}
void Nextion::SendStatus(Value &value, bool SkipPrecheck)
{
    if (SkipPrecheck || value.alarm != value.lastSentAlarm)
    {
        char valueID[4] = {value.type, 0, 0, 0};
        if (value.num < 10)
            valueID[1] = value.num + '0';
        else
        {
            valueID[1] = (char)(value.num / 10 + '0');
            valueID[2] = (char)(value.num % 10 + '0');
        }
        ChangeBackgroundColour(valueID, value.colour[value.alarm]);
        value.lastSentAlarm = value.alarm;
    }
}
void Nextion::AddWarning(Warnings &addedWarn, Warn_t type)
{
    if (!addedWarn.isLogged())
    {
        addedWarn.LogWarning(type);
        WarningIndex = WarningCntr;
        if (WarningCntr < MAX_NUM_WARNINGS)
        {
            ActiveWarnings[WarningCntr] = &addedWarn;
            (WarningCntr == MAX_NUM_WARNINGS ? WarningCntr = MAX_NUM_WARNINGS : WarningCntr++);
        }
        debugln("W_c = " + String(WarningCntr));
        debugln(ActiveWarnings[WarningCntr - 1]->OutputStr);
        SendCurrWarning();
        debug("Logged warning : ");
        debugln(WarningCntr);
        cbNextion('w', true, true);
    }
    else
        debugln("Err: warning already logged vvvv");
}
void Nextion::AddFault(Warnings &addedWarn, Warn_t type)
{
    AddWarning(addedWarn, type);
    ChangePage(P_POPUP);
    debugln("Logged Fault");
}
void Nextion::ClearWarnings(void)
{
    for (size_t i = 0; i < WarningCntr; i++)
    {
        ActiveWarnings[i]->ClearLog();
    }
    WarningCntr = 0;
    WarningIndex = 0;
    SendGlobalVal("cip", false);
    cbNextion('w', false, false);
    SendGlobalVal("W_c", 0);
    // SendGlobalVal("W_i", 1); // not strictly necessary
    ChangeVis("e0", false);  // vis e0,0
    ChangeVis("e1", false);  // vis e1,0
    ChangeVis("e2", false);  // vis e2,0
    ChangeVis("CIP", false); // vis CIP,0
    ChangeVis("wc", false);  // vis wc,0
    ChangeVis("wi", false);  // vis wi,0
    ChangeVis("sl", false);  // vis sl,0
    SendGlobalStr("ErrT", "");
}
void Nextion::NextWarning(bool INCREMENT_nDECREMENT)
{
    if (INCREMENT_nDECREMENT)
    {
        if (WarningIndex < WarningCntr - 1)
            WarningIndex++;
        else
            WarningIndex = 0;
    }
    else
    {
        if (WarningIndex > 0)
            WarningIndex--;
        else
            WarningIndex = WarningCntr - 1;
    }
    SendCurrWarning();
}
void Nextion::SendCurrWarning(void)
{
    if (ActiveWarnings[WarningIndex] != nullptr)
    {
        SendGlobalStr("ErrT", ActiveWarnings[WarningIndex]->getOutputStr());
        SendGlobalVal("W_c", WarningCntr);
        SendGlobalVal("W_i", WarningIndex + 1);
    }
    else
        debugln("No warning at this index");
}
void Nextion::SendPumpFault(uint fault, Icon &PumpIcon)
{
    if (PumpIcon.lastSentPumpFault != fault)
    {
        char tempStr[6] = {0};
        if (fault)
            snprintf(tempStr, sizeof(tempStr), "F-%u", fault);
        char TempPumpStr[4] = {'m', PumpIcon.number, 'F', 0};
        SendGlobalStr(TempPumpStr, tempStr);
        PumpIcon.lastSentPumpFault = fault;
    }
}
void Nextion::ActivateCIPWarning(void)
{
    SendGlobalVal("cip", true);
}
void Nextion::UpdateWifiStatus(bool on_off) // DEPRECATED
{
    if (wifiStatus != on_off)
    {
        SendGlobalVal("wifi", on_off);
        wifiStatus = on_off;
    }
}
void Nextion::UpdateTimeVisibility(bool visible)
{
    if (timeEnabled != visible)
    {
        ChangeVis("tD", visible);
        ChangeVis("tT", visible);
        timeEnabled = visible;
    }
}
// Listens for Commands and data coming from display
void Nextion::Listen(void)
{
    /* format: minimum 5 bytes
     * 1 start byte     - '<'
     * 1 header bytes   - e.g. 'p' for page
     * 1 length byte    - length of data e.g. 1 -- must be < MAX_DATA_SIZE
     * n data bytes     - message or payload
     * 1 stop byte      - '>'
     */
    if (ser_cnt == 0) // keep coming in here if a start byte isnt received
    {
        char first_byte = Comms.read();
        if (first_byte == START) // check if the start byte is received
        {
            ser_cnt++;
            header = H_NONE;
            length = 0;
            for (size_t i = 0; i < MAX_DATA_SIZE; i++) // clear the buffer - might not be necessary
                data[i] = 0;
        }
    }
    else // start bit was received carry on to check for header and data
    {
        if (ser_cnt == 1 && Comms.available()) // get the header and length
        {
            header = Comms.read();
            ser_cnt++;
        }
        if (ser_cnt == 2 && Comms.available()) // get the length of the data
        {
            length = Comms.read();
            ser_cnt++;
        }
        if (ser_cnt >= 3 && Comms.available())
        {
            uint data_cnt = ser_cnt - 3; // counter minus the start,header and length bytes
            if (length > data_cnt && length <= MAX_DATA_SIZE)
            {
                data[data_cnt] = Comms.read();
                ser_cnt++;
            }
            else if (Comms.peek() == STOP)
            {
                Comms.read();
                ser_cnt = 0;
                CommsHandler();
            }
            else // error stop byte not last byte
            {
                ser_cnt = 0;

                debug("errSerial: ");
                debugBase(header, HEX);
                debugBase(length, HEX);
                debugln(data);
                // delay(100);
                while (Comms.available())
                {
                    debugBase(Comms.read(), HEX);
                    debug(" ");
                }
                debugln();
            }
        }
    }
}
void Nextion::PrintPage(bool firstEntry) // set firstEntry to true for setup of page (i.e. page has changed or when starting up)
{
    switch (currPage)
    {
    case P_SPLASH:
        cbNextion('$', true, true);
        if (!startUp)
        {
            ChangePage(P_MAIN);
            debugln("nextion reset");
        }
        else
            startUp = false;
        break;
    case P_MAIN:
        if (firstEntry)
        {
            ChangeVis("tD", timeEnabled);
            ChangeVis("tT", timeEnabled);
        }
        SendIconStatus(I_WIFI, firstEntry);

        SendIconStatus(I_BW_VALVE, firstEntry);
        SendIconStatus(I_INLET_VALVE, firstEntry);
        SendIconStatus(I_FLUSH_VALVE, firstEntry);

        SendIconStatus(I_FEED_PUMP, firstEntry);
        SendIconStatus(I_RO_PUMP, firstEntry);

        SendSensorValue(I_FEED_PUMP_PRESS, firstEntry);
        SendSensorValue(I_HP_INLET_PRESS, firstEntry);
        SendSensorValue(I_HP_OUTLET_PRESS, firstEntry);
        SendSensorValue(I_DELTA_PRESS, firstEntry);
        SendSensorValue(I_POST_MEM_PRESS, firstEntry);

        SendSensorValueStatus(I_PERM_FLOW, firstEntry);
        SendSensorValueStatus(I_RECYCLE_FLOW, firstEntry);
        SendSensorValueStatus(I_BRINE_FLOW, firstEntry);

        SendSensorValueStatus(I_TEMP, firstEntry);
        SendSensorValueStatus(I_EC_PERM, firstEntry);

        SendSensorValueStatus(I_RECOVERY, firstEntry);
        // SendValue(VAL_RECOVERY, firstEntry);
        // SendStatus(VAL_RECOVERY, firstEntry);
        break;
    case P_GAUGES:
        if (firstEntry)
        {
            ChangeVis("tD", timeEnabled);
            ChangeVis("tT", timeEnabled);
        }
        SendIconStatus(I_WIFI, firstEntry);

        SendIconStatus(I_FEED_PUMP, firstEntry);
        SendIconStatus(I_RO_PUMP, firstEntry);
        SendIconStatus(I_BOOSTER_PUMP, firstEntry);

        SendSensorValue(I_FEED_PUMP_PRESS, firstEntry);
        SendSensorValue(I_HP_INLET_PRESS, firstEntry);
        SendSensorValue(I_HP_OUTLET_PRESS, firstEntry);
        SendSensorValue(I_DELTA_PRESS, firstEntry);
        SendSensorValue(I_POST_MEM_PRESS, firstEntry);
        SendSensorValue(I_BOOSTER_PRESS, firstEntry);

        SendSensorValue(I_FEED_FLOW, firstEntry);
        SendSensorValue(I_PERM_FLOW, firstEntry);
        SendSensorValue(I_RECYCLE_FLOW, firstEntry);
        SendSensorValue(I_BRINE_FLOW, firstEntry);

        SendValue(VAL_RECOVERY, firstEntry);
        SendStatus(VAL_RECOVERY, firstEntry);
        SendValue(VAL_PERM_VOL, firstEntry);

        SendPumpValues(PVAL_FEED_PUMP, firstEntry);
        SendPumpValues(PVAL_RO_PUMP, firstEntry);
        SendPumpValues(PVAL_BOOSTER_PUMP, firstEntry);
        break;
    case P_SETTINGS:
        if (firstEntry)
        {
            ChangeVis("tD", timeEnabled);
            ChangeVis("tT", timeEnabled);
        }
        SendIconStatus(I_WIFI, firstEntry);
        break;
    case P_MANUAL:
        if (firstEntry)
        {
            ChangeVis("tD", timeEnabled);
            ChangeVis("tT", timeEnabled);
        }
        SendIconStatus(I_WIFI, firstEntry);

        SendIconStatus(I_BW_VALVE, firstEntry);
        SendIconStatus(I_INLET_VALVE, firstEntry);
        SendIconStatus(I_FLUSH_VALVE, firstEntry);

        SendIconStatus(I_RO_PUMP, firstEntry);
        SendIconStatus(I_FEED_PUMP, firstEntry);

        SendSensorValue(I_FEED_PUMP_PRESS, firstEntry);
        SendSensorValue(I_HP_INLET_PRESS, firstEntry);
        SendSensorValue(I_HP_OUTLET_PRESS, firstEntry);
        SendSensorValue(I_DELTA_PRESS, firstEntry);
        SendSensorValue(I_POST_MEM_PRESS, firstEntry);

        SendSensorValueStatus(I_PERM_FLOW, firstEntry);
        SendSensorValueStatus(I_RECYCLE_FLOW, firstEntry);
        SendSensorValueStatus(I_BRINE_FLOW, firstEntry);

        SendSensorValueStatus(I_TEMP, firstEntry);
        SendSensorValueStatus(I_EC_PERM, firstEntry);

        SendSensorValueStatus(I_RECOVERY, firstEntry);
        break;
    case P_CALIBRATE:
        SendValue(VAL_CalibVoltage, firstEntry);
        SendValue(VAL_CalibCurrent, firstEntry);
        SendValue(VAL_CalibPermPM, firstEntry);
        SendValue(VAL_CalibBrinePM, firstEntry);
        SendValue(VAL_CalibRecyclePM, firstEntry);

        SendValue(VAL_A0_Voltage, firstEntry);
        SendValue(VAL_A1_Voltage, firstEntry);
        SendValue(VAL_A2_Voltage, firstEntry);
        SendValue(VAL_A3_Voltage, firstEntry);

        SendValue(VAL_A0_Current, firstEntry);
        SendValue(VAL_A1_Current, firstEntry);
        SendValue(VAL_A2_Current, firstEntry);
        SendValue(VAL_A3_Current, firstEntry);

        SendSensorValueStatus(I_PERM_FLOW, firstEntry);
        SendSensorValueStatus(I_RECYCLE_FLOW, firstEntry);
        SendSensorValueStatus(I_BRINE_FLOW, firstEntry);
        break;
    case P_NONE:
    case P_POPUP:
    case P_SLEEP:
    case P_GLOBAL:
    default:
        debug("ErrPrintPage: ");
        debuglnBase(currPage, HEX);
    }
}
void Nextion::ConvertToHMI_SensorValue(float val, Sensor &sensor, bool checkMaxMin)
{
    sensor.value = val * sensor.mult;
    // debug(String((char)sensor.type) + String(sensor.number) + " - val: " + String(sensor.value));
    if (checkMaxMin && (sensor.checkAlarmMax() || sensor.checkAlarmMin()))
    {
        sensor.alarm = true;
        // debug("   alarm  min: " + String(sensor.minAlarmVal, DEC) + "  max: " + String(sensor.maxAlarmVal, DEC));
    }
    else
    {
        sensor.alarm = false;
    }
    if (checkMaxMin && (sensor.checkFaultMax() || sensor.checkFaultMin()))
    {
        // debug("   fault  min: " + String(sensor.minFaultVal, DEC) + "  max: " + String(sensor.maxFaultVal, DEC));
        sensor.fault = true;
    }
    else
        sensor.fault = false;
    // debugln();
}
void Nextion::ConvertToHMI_Value(float val, Value &value, bool checkMaxMin)
{
    value.val = val * value.mult;
    // debug(String((char)value.type) + String(value.num) + " - val: " + String(value.val));
    // debug("min: " + String(value.minVal, DEC) + "  max: " + String(value.maxVal, DEC));
    if (checkMaxMin && (value.checkMax() || value.checkMin()))
    {
        value.alarm = true;
        // debug("   alarm !! ");
    }
    else
        value.alarm = false;
    // debugln();
}
void Nextion::ConvertToHMI_TankLevel(float val, TankBar &tank, bool alarm)
{
    tank.value = val / 1.1F; // * tank.mult; // Slider is 110% to fit the overflow condition into the slider but only represented by values 0-100
    // debugln(tank.value);
    if (alarm) //(tank.checkMax() || tank.checkMin() || alarm)
        tank.alarm = true;
    else
        tank.alarm = false;
}

void Nextion::UpdateCountdown(uint seconds)
{
    if (currPage == P_MAIN)
    {
        Comms.print("Secs.txt=\" - ");
        if (seconds >= 60)
        {
            Comms.print(seconds / 60);
            Comms.print("min");
        }
        Comms.print(seconds % 60);
        Comms.print("s\"");
        _endTrans();
    }
}
void Nextion::RemoveCountdown(void)
{
    Comms.print("Secs.txt=\"\"");
    _endTrans();
}

Icon::Icon(IconType_t IconType, uint8_t SeqNo, const uint8_t *Pics) //, uint8_t OffPic, uint8_t OnPic, uint8_t WarningPic = 0, uint8_t FaultPic = 0, uint8_t DisconnectPic = 0)
{
    type = IconType;
    number = SeqNo;
    statusPics = Pics;
}

Sensor::Sensor(SensorType_t SensType, char SeqNo, const uint8_t *Pics, uint Multiplier, uint _minVal, uint _maxVal, uint _Fmin, uint _Fmax)
{
    type = SensType;
    number = SeqNo;
    pic[0] = Pics[Off];
    pic[1] = Pics[Warning];
    pic[2] = Pics[Fault];
    mult = Multiplier;
    minAlarmVal = _minVal;
    maxAlarmVal = _maxVal;
    minFaultVal = _Fmin;
    maxFaultVal = _Fmax;
}
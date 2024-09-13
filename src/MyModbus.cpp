#include "MyModbus.h"

ModbusRTU mb;
#define ONE_MIN 60000
uint GBL_VSD_COUNTER = 0;
uint currentVSD_Index = 1;

ModbusVSD::ModbusVSD(uint slaveID, cbTransaction callBack) : _cbFunction(callBack), _slave_ID(slaveID), _localIndex(++GBL_VSD_COUNTER) {};

void ModbusVSD::ModbusSetup(HardwareSerial *MBSTREAM)
{
    MBSTREAM->begin(9600, SERIAL_8N2, MB_RX_PIN, MB_TX_PIN);
    mb.begin(MBSTREAM);
    mb.master();
}

void ModbusVSD::RunModbus(void)
{
    if (currentVSD_Index == _localIndex)
    {
        static bool requestSent = false;
        switch (MBstate)
        {
        case MB_Idle:
            if (millis() - _timeStamp >= MODBUS_PERIOD)
            {
                // debugln("MB-Start");
                MBstate = MB_ReadState;
                _timeStamp = millis();
            }
            break;
        case MB_ReadState:
            if (!mb.slave()) // check if MB line is free (will return 0)
            {
                // debugln(PumpState);
                if (!requestSent) // check if the state request has been sent
                {
                    // debugln("before here - 1");
                    requestSent = mb.readHreg(_slave_ID, _mbStateAddr, _data, 1, _cbFunction);
                    // debugln("after here - 1 ");

                    if (!requestSent)
                        Failed();
                }
                else
                {
                    // debugln("here return 1");
                    if (PumpState == PumpDisconnect)
                    {
                        MBstate = MB_Idle;
                    }
                    else
                    {
                        State = _data[0];
                        MBstate = MB_ReadFault;
                    }
                    requestSent = false;
                }
            }
            mb.task();
            break;
        case MB_ReadFault:
            if (!mb.slave()) // check if MB line is free (will return 0)
            {
                if (!requestSent) // check if the state request has been sent
                {
                    requestSent = mb.readHreg(_slave_ID, _mbFaultAddr, _data, 1, _cbFunction);
                    if (!requestSent)
                        Failed();
                }
                else
                {
                    if (PumpState == PumpDisconnect)
                    {
                        MBstate = MB_Idle;
                    }
                    else
                    {
                        Fault = _data[0];
                        if (Fault == 0)
                        {
                            if (State == 1 or State == 2) // running
                                PumpState = PumpRun;
                            else
                                PumpState = PumpStop;
                        }
                        else if (State == 2)
                        {
                            PumpState = PumpFault;
                        }

                        MBstate = MB_ReadValues;
                    }
                    Running = (PumpState == PumpRun);
                    if (Running)
                        CalculateMins();
                    // debugln("Pump.Running =" + String(Running, BIN));
                    requestSent = false;
                }
            }
            mb.task();
            break;
        case MB_ReadValues:
            if (!mb.slave()) // check if MB line is free (will return 0)
            {
                if (!requestSent) // check if the state request has been sent
                {
                    requestSent = mb.readHreg(_slave_ID, _mbValueAddr, _data, _numValues, _cbFunction);
                    if (!requestSent)
                        Failed();
                }
                else
                {
                    if (PumpState != PumpDisconnect)
                    {
                        Frequency = _data[0];  // U0-00
                        VoltageOut = _data[3]; // U0-03
                        CurrentOut = _data[4]; // U0-04
                        PowerOut = _data[5];   // U0-05
                        AnalogIn1 = _data[9];  // U0-09
                        AnalogIn2 = _data[10]; // U0-10
                        AnalogIn3 = _data[11]; // U0-11}
                    }
                    MBstate = MB_Idle;
                    requestSent = false;
                    // Go to next VSD
                    IncrementVSD_Index();
                }
            }
            mb.task();
            break;

        default:
            debugln("MB state err");
            break;
        }
    }
}

void ModbusVSD::Failed(void)
{
    debugln("MB failed to send");
    MBstate = MB_Idle;
    // MBnextState = MB_ReadState;
}

void ModbusVSD::CalculateMins(void)
{
    if (Running)
    {
        if ((millis() - _pumpTimeStamp) >= ONE_MIN)
        {
            pumpMin++;
            debugln("PumpMins:" + String(pumpMin, DEC));
            _pumpTimeStamp = millis();
        }
    }
    else
        _pumpTimeStamp = millis();
}

void ModbusVSD::IncrementVSD_Index(void)
{
    currentVSD_Index = _localIndex + 1;
    if (currentVSD_Index > GBL_VSD_COUNTER)
        currentVSD_Index = 1;
    // debugln("MB_ID: " + String(currentVSD_Index));
}
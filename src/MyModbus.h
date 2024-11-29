#ifndef MY_MODBUS_H_
#define MY_MODBUS_H_

#include "OB_Define.h"
#include <ModbusRTU.h>

// extern ModbusRTU mb;
// bool cbFunc(Modbus::ResultCode event, uint16_t transactionId, void *data);

class ModbusVSD
{
private:
    uint _slave_ID;
    const uint MODBUS_PERIOD = 1000; // minimum frequency (this may take longer depending on errors, baud rate, no of operations, etc.)
    const uint _mbStateAddr = 0x3000;
    const uint _mbFaultAddr = 0x8000;
    const uint _mbValueAddr = 0x7000;
#define N_MAX_EMHEATER 12       // 12 is the maximum allowable registers to be read in one transmission.
    const uint _numValues = 12; // number of values to read

    bool requestSent = false; 
    
    uint _localIndex; // index on the list of VSDs
    uint16_t _data[N_MAX_EMHEATER] = {};
    ulong _timeStamp = millis();
    ulong _pumpTimeStamp = _timeStamp;
    uint State = 0;

    cbTransaction _cbFunction = nullptr;

public:
    // static uint GBL_VSD_COUNTER;
    enum MBstate_t
    {
        MB_Idle,
        MB_ReadState,
        MB_ReadFault,
        MB_ReadValues
    } MBstate = MB_Idle;
    // MBstate_t MBnextState = MB_ReadState;
    enum PumpState_t
    {
        PumpStop,
        PumpRun,
        PumpAlarm,
        PumpFault,
        PumpDisconnect,
        PumpConnect
    } PumpState = PumpStop;

    bool Running = 0;
    uint Fault = 0;
    uint pumpMin = 0;
    uint Frequency = 0;  // U0-00
    uint VoltageOut = 0; // u0-03
    uint CurrentOut = 0; // U0-04
    uint PowerOut = 0;   // U0-05
    uint AnalogIn1 = 0;  // U0-09
    uint AnalogIn2 = 0;  // U0-10
    uint AnalogIn3 = 0;  // U0-11
    ulong TimerFault = millis();
    ulong TimerDisconnect = millis();

    ModbusVSD(uint slaveID, cbTransaction callBack); 
    // bool ReadFault(void) { return MB485.readHoldingRegisters(_MbFaultAddr, 1); }
    // bool ReadValues(void) { return MB485.readHoldingRegisters(_MbValueAddr, _numValues); }
    void ModbusSetup(HardwareSerial *MBSTREAM);
    void RunModbus(void);
    void Failed(void);
    void CalculateMins(void);
    void IncrementVSD_Index(void);
};

#endif
#include <OB_Define.h>

OneWire OWL(TEMP); // Initialise temperature sensor (a 4.7K pullup resistor is required)
// think the dallas library could be optimised for our use case
DallasTemperature DS18S20(&OWL); // Pass our oneWire reference to Dallas Temperature sensor class
Timezone local;

void PinSetup(void)
{
    // pinMode(STATUS_LED, OUTPUT);

    pinMode(D_OUT_1, OUTPUT);
    pinMode(D_OUT_2, OUTPUT);
    pinMode(D_OUT_3, OUTPUT);
    pinMode(D_OUT_4, OUTPUT);
    pinMode(D_OUT_5, OUTPUT);
    pinMode(D_OUT_6, OUTPUT);
    pinMode(D_OUT_7, OUTPUT);
    pinMode(D_OUT_8, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    digitalWrite(D_OUT_1, LOW);
    digitalWrite(D_OUT_2, LOW);
    digitalWrite(D_OUT_3, LOW);
    digitalWrite(D_OUT_4, LOW);
    digitalWrite(D_OUT_5, LOW);
    digitalWrite(D_OUT_6, LOW);
    digitalWrite(D_OUT_7, LOW);
    digitalWrite(D_OUT_8, LOW);

    pinMode(D_IN_1, INPUT_PULLUP);
    pinMode(D_IN_2, INPUT_PULLUP);
    pinMode(D_IN_3, INPUT_PULLUP);
    pinMode(D_IN_4, INPUT_PULLUP);
    pinMode(D_IN_5, INPUT_PULLUP);
    pinMode(D_IN_6, INPUT_PULLUP);
    pinMode(D_IN_7, INPUT_PULLUP);
    pinMode(D_IN_8, INPUT_PULLUP);
}

bool AnalogSetup(ADS1115 &AnalogDC)
{
    if (Wire.begin(S_DATA, S_CLK))
    {
        if (AnalogDC.begin())
        {
            AnalogDC.setGain(0);     //  +/-4 volt -  2mV unit
            AnalogDC.setDataRate(7); //  [0-7] = {8, 16, 32, 64, 128, 250, 475, 860} SPS
            AnalogDC.setMode(1);     //  continuous mode = 0 , single = 1
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool ReadInput(uint inputPin) { return digitalRead(inputPin); }
bool OutputStatus[OUTPUT_MAX] = {0};
void WriteOutput(uint outputPin, bool val)
{
    bool pinCheck = false;
    switch (outputPin)
    {
    case D_OUT_1:
    case D_OUT_2:
    case D_OUT_3:
    case D_OUT_4:
    case D_OUT_5:
    case D_OUT_6:
    case D_OUT_7:
    case D_OUT_8:
        pinCheck = true;
        break;
    }
    if (pinCheck)
    {
        OutputStatus[outputPin] = val;
        digitalWrite(outputPin, val);
    }
    else
        debugln("Invalid Output pin");
}

// Checks for a change in state on a pin and times difference of one cycle to work out flow.
// Set input to true to calculate flow or false for only volume
// returns true if a new flow value is available
bool VolumeMeter::CheckVolumeMeter(bool calculateFlow)
{
    static bool same = false;
    bool newVal = false;
    bool current_status = ReadInput(_inputPin);
    ulong currentMillis = millis();
    // debugln("current: " + String(current_status) + "  last: " + String(_last_status));

    if (current_status == true && _last_status == false) // RISING PULSE
    {
        Volume_L += _L_PER_PULSE;
        Volume_m3 = (float)Volume_L / 1000.0;
        if (calculateFlow)
        {
            if (firstEntry)
            {
                _timeStamp = currentMillis;
                firstEntry = false;
                same = false;
            }
            else
            {
                if (same)
                {
                    FlowRate = CalculateFlow(currentMillis - _timeStamp);
                    newVal = true;
                    _timeStamp = currentMillis;
                    same = false;
                }
                else
                {
                    same = true;
                }
            }
        }
    }
    else if ((currentMillis - _timeStamp) > _MAX_PULSE_TIME)
    {
        FlowRate = 0;
        firstEntry = true;
        newVal = true;
    }
    _last_status = current_status;
    return newVal;
}

float VolumeMeter::CalculateFlow(uint timeDiff)
{
    const uint msPerHour = 3600000;
    const uint litresPerCube = 1000;
    const uint NumPulses = 2;
    return (float)_M3_PER_PULSE * litresPerCube * msPerHour / timeDiff * NumPulses; // m3/pulse * litres/m3 * ms/hr * pulse/ms / Num of pulses = litres/hr
}

// this is written for any time period. Can be made much simpler by calling at a set period and reducing the float calculations and therefore processor overhead
void PulseMeter::CalculateFlow(bool calcVolume)
{
    if (Pulses)
    {
        uint timeDiff = millis() - _timeStamp;
        _timeStamp = millis();
        uint tempPulses = Pulses;
        Pulses = 0;
        float temp = (float)tempPulses * 1000; // this is for the ms to second conversion of timeDiff
        if (calcVolume)
        {
            volumePulses += tempPulses;
            while (volumePulses >= 288)
            {
                Volume++;
                volumePulses -= 288;
            }
        }
        // temp /= timeDiff; // pulses / timeDiff (ms) * 1000 (ms/s) = PulsesPerSecond
        temp *= *_FLOW_FACTOR / timeDiff; // TODO compare this with UF
        FlowRate = (FlowRate * (_div - 1) + temp) / _div;
        // cummulativeFlow += temp - (cummulativeFlow / div);
        // FlowRate = cummulativeFlow / div; // flow = pulsesPerSecond * Flow_Factor - answer in litres per hour}
    }
    else
    {
        // cummulativeFlow = 0;
        FlowRate = 0.0F;
    }
}

void PulseMeter::CalculateFlowNew(bool calcVolume)
{
    uint timeDiff = millis() - _timeStamp;
    if (millis() - _pulseTimeStamp < _timeOut)
    {
        if (time_for_pulses_ms)
        {
            float localTimeMs = time_for_pulses_ms;
            time_for_pulses_ms = 0;
            float freq = 1000.0F * PULSE_SAMPLES / localTimeMs;
            float temp = freq * 60.0F / *_FLOW_FACTOR; // conver to lph from lpm and divide by the manufacturer flow factor
            FlowRate = (FlowRate * (_div - 1) + (temp)) / _div;
            // if (_inputPin == D_IN_4)
            //     debug("Product");
            // else if (_inputPin == D_IN_5)
            //     debug("Waste");
            // else
            //     debug("X-flow");
            // debug(" - time: ");
            // debug((uint)localTimeMs);
            // debug("ms\tfreq: ");
            // debug(freq);
            // debug("Hz\tFF: ");
            // debug(*_FLOW_FACTOR);
            // debug("\tFlowRate: ");
            // debugln(temp);
            // debugln();
            if (calcVolume)
            {
                volumePulses += PULSE_SAMPLES;
                while (volumePulses >= _pulsePerLitre)
                {
                    Volume++;
                    volumePulses -= _pulsePerLitre;
                }
            }
        }
    }
    else
    {
        // _pulseTimeStamp = millis();
        // Pulses = 0;
        FlowRate = 0.0F;
    }
}

bool DigitalInput::ReadInputDebounce(uint debounce_ms)
{
    bool current_status = ReadInput(_inputPin);

    if (current_status != _last_status)
    {
        if (millis() - _debounceStamp >= debounce_ms)
            _last_status = current_status;
    }
    else
        _debounceStamp = millis();

    return _last_status;
}

bool DigitalInput::ReadInputDelay(uint delay_ms, bool input)
{
    bool current_status = input;
    if (current_status != _last_delay_status)
    {
        if (millis() - _delayStamp >= delay_ms)
            _last_delay_status = current_status;
    }
    else
        _delayStamp = millis();

    return _last_delay_status;
}

bool InputTimer::DelayTimer(uint delay_ms, bool input)
{
    bool current_status = input;
    if (current_status != _last_delay_status)
    {
        if (millis() - _delayStamp >= delay_ms)
            _last_delay_status = current_status;
    }
    else
        _delayStamp = millis();

    return _last_delay_status;
}

uint TempSensor::_indexCount = 0;

// void TempSensor::Begin(void)
TempSensor::TempSensor()
{
    DS18S20.begin();        // Initialise Temperature sensor
    _index = _indexCount++; // store index and increment global index
    // _indexCount++;
    delay(100);
    if (!DS18S20.getDeviceCount()) // if no temperature sensor found
    {
        debugln("No temperature sensor found. Retrying...");
        DS18S20.begin(); // Initialise Temperature sensor
        delay(1000);
        if (!DS18S20.getDeviceCount())
        {
            debugln("No temperature sensor found.");
            //   Blynk.virtualWrite(VIRTUAL_STATUS, "No temperature sensor found.");
        }
    }
    if (DS18S20.getDeviceCount())
    {
        DS18S20.getAddress(_deviceAddress, _index); // store address
        DS18S20.setResolution(11);                  // Set resolution for temperature sensor to 11 bits .13C resolution
        DS18S20.setWaitForConversion(false);
        // delay(200);
        DS18S20.requestTemperaturesByAddress(_deviceAddress);
        _timeStamp = millis();
        // _readingPeriod = DS18S20.millisToWaitForConversion()
        _readingPeriod = (DS18S20.getResolution() < 12 ? 520 : 750); // in testing the library/datasheet numbers were too short for anyhting below 12bit..  (11 - 9 bit ~ 508ms)
        debugln("Temperature sensor found");
        connected = true;
    }
}

// Reads requested temperature
bool TempSensor::ReadTemp(void) // Checked
{
    if (connected && millis() - _timeStamp >= _readingPeriod)
    {
        // debugln("*********** temp here");
        if (DS18S20.isConversionComplete())
        {
            // debug(millis() - _timeStamp);
            _timeStamp = millis();
            currTemp = DS18S20.getTempC((uint8_t *)_deviceAddress);
            // debug(" Temp= ");
            // debugln(currTemp);
            DS18S20.requestTemperaturesByAddress(_deviceAddress);
            return true;
        }
    }
    return false;
}

bool TankLevelSensor::ComputeTankLevel(float signal4to20mA)
{
    const float mA_4 = 4.0F;
    const float mA_20 = 20.0F;
    const float diff_mA = mA_20 - mA_4;
    if (signal4to20mA < mA_4)
    {
        Level = 0;
        Level_p = 0;
        // debugln("<4mA tanklvl");
        return false; // signal level too low
    }
    else if (signal4to20mA > mA_20)
    {
        Level = sensorMax;
        Level_p = TankMax_p;
        // debugln(">20mA tanklvl");
        return false;
    }
    else
    {
        Level = map_4to20(signal4to20mA, 0.0F, sensorMax);
        Level_p = map_f(Level, tankEmpty_mm, tankFull_mm, 0.0F, TankFull_p);
        if (Level_p >= TankHigh_p)
            tankState = T_HIGH;
        else if (Level_p >= TankFull_p)
            tankState = T_FULL;
        else if (Level_p >= TankLow_p)
            tankState = T_MID;
        else
        {
            if (Level_p > TankEmpty_p)
                tankState = T_LOW;
            else
                tankState = T_EMPTY;
        }
        return true;
    }
}
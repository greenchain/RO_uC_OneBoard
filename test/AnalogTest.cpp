/*  RO controller
 *   -------------
 *   VERSION 0.1 -
 */

#include <Arduino.h>
#include <unity.h>
#include <OB_Define.h>

bool ADC_ok = 0;

ADS1115 ADC_1(0x48);

//********************** CALIBRATION **********************
#define VoltageFactor 2510 //
//  1V = 2500
#define CurrentFactor0 1200
#define CurrentFactor1 1235
#define CurrentFactor2 1215
#define CurrentFactor3 1200
/*
      Values     V      I
      board 1    2483   1185
      board 2    2500   ?
      board 3    2510   ?
      board 4    2510   ?
      board 5    2480   ?
*/
TankLevelSensor ProductTank;
bool Check4to20(float &value);
extern bool AnalogSetup(ADS1115 &AnalogDC);

//********************** SETUP **********************
void setup()
{
  Serial.begin(115200, SERIAL_8N1, 44, 43);
  delay(2000);
  UNITY_BEGIN();
  
  // PinSetup();
  ADC_ok = AnalogSetup(ADC_1);
}

//********************** MAIN **********************
void loop()
{
  static int Counter = 0;
  ulong t_stamp;
  float A0, A1, A2, A3, PH, EC, ORP ;

  // Serial.println(Counter++);
  if (ADC_ok)
  {
    t_stamp = millis();
    // Serial.printf("Start time: %lu \n", millis());
    static bool switchAnalog = true;
    if (switchAnalog)
    {
      A0 = (float)ADC_1.readADC(0) / CurrentFactor0;
      Check4to20(A0);
      // ProductTank.ComputeTankLevel(A0);
      // debug("Tank state: " + String(SM.ProductTank.tankState) + " ");
      debugln("LVL:  " + String(A0, 2) + "mA  " + String((float)ProductTank.Level_p, 1) + "\% - CF:" + String(CurrentFactor0));

      A1 = (float)ADC_1.readADC(1) / CurrentFactor1;
      Check4to20(A1);
      PH = map_4to20(A1, 0, 14);
      debugln("PH: " + String(A1, 2) + "mA  " + String((float)PH, 2) + "pH - CF:" + String(CurrentFactor1));
    }
    else
    {
      A2 = (float)ADC_1.readADC(2) / CurrentFactor2;
      Check4to20(A2);
      ORP = map_4to20(A2, -1000, 1000);
      debugln("ORP: " + String(A2, 2) + "mA  " + String((float)ORP, 1) + "mV - CF:" + String(CurrentFactor2));

      A3 = (float)ADC_1.readADC(3) / CurrentFactor3;
      Check4to20(A3);
      // static float EC = 0;
      // SM.EC = (SM.EC * 9 + map_4to20(Temp4, SENSOR_MIN_EC, SENSOR_MAX_EC)) / 10;
      EC = map_4to20(A3, 0, 2000);
      debugln("EC: " + String(A3, 2) + "mA  " + String((float)EC, 1) + "uS/cm - CF:" + String(CurrentFactor3));
      debugln();
    }
    switchAnalog = !switchAnalog;
  }
  else
    debugln("Wire error: check ADS1115");
  delay(1010);
}

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



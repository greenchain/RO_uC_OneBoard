/*  RO controller
 *   -------------
 *   VERSION 1.0 - Ro Controller originally built for Marham in UK
 *  !! Always remember to update version in BlynkRO.h !!
 *
 * v1.0
 * - added blynk alarms, faults and state and setting events. Temperature added to dash
 * - added warning and fault lights to the outputs to be displayed on control panel
 * - added a Wifi symbol to HMI for display when connected
 * v1.0.1
 * - no visibility of time when not set by server
 * - changed the way faults are cancelled by adding a check
 * v1.0.2
 * - edited the clear warnings callback to change vis of certain components
 * - moved Blynk calls to reduce wifi noise
 * - removed bypass from RO sate maching and only opens when tank is low and closes when >100% OR in service and >25%
 * - added warning for low feed tank
 * v1.0.3
 * - changed flush timers to seconds for better control
 */

#include "Definitions.h"
// #include <nvs_flash.h>

/* General Functions */
void InterruptSetup(void);
void IRAM_ATTR permeatePulse(void) { PermeatePM.Pulses++; }
void IRAM_ATTR brinePulse(void) { BrinePM.Pulses++; }
void IRAM_ATTR recyclePulse(void) { RecyclePM.Pulses++; }

TaskHandle_t BlynkTH;
bool analog_flag;

// TempSensor TestTemp;
//********************** SETUP **********************
void setup()
{
  // nvs_flash_erase(); // erase the NVS partition
  // nvs_flash_init(); // initialize the NVS partition
  // while(true);
  PinSetup();
  Serial.begin(115200);
  Serial.println("Start");
  analog_flag = AnalogSetup(ADC_1); // TODO make a class for analog sensors
  HPP_VSD.ModbusSetup(&Serial1);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  SET.FileSetup(); // File System setup and reads
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  HMI.setCallback(cbNextionListen);
  HMI.Startup();
  vTaskDelay(100 / portTICK_PERIOD_MS);

  TimerSetup();
  InterruptSetup();
  // disableCore1WDT();
  xTaskCreatePinnedToCore(BlynkLoop, "Blynk", 10000, NULL, 1, &BlynkTH, 1);
}

//********************** MAIN **********************
void loop()
{
  HPP_VSD.RunModbus();
  BOOST_VSD.RunModbus();
  // RunAnalog(ADC_1); //TODO change this to a method in OB class once ADC class created
  while (Comms.available()) // listen if there are items on serial
  {
    HMI.Listen();
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

//********************** FUNTIONS **********************
void InterruptSetup(void)
{
  attachInterrupt(digitalPinToInterrupt(FLOW_PERM), permeatePulse, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_BRINE), brinePulse, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_RECYCLE), recyclePulse, RISING);
}
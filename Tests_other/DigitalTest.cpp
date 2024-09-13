/*  RO controller
 *   -------------
 *   VERSION 0.1 -
 */

#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_TEMPLATE_ID "TMPL2E91bPiCL"
#define BLYNK_TEMPLATE_NAME "RO"
// #define BLYNK_PRINT Serial

#include <BlynkEdgent.h>
#include "Definitions.h"
#include "Nex_Ultra.h"

#define DEBUG 0

/* Globals */
// uint state = S_STOPPED;                      // current state
// uint page = 0;                               // current HMI page
// uint minuteCounter = 0;                      // keeps count of minutes passed when in Service or CEB soak
// uint airScourTime = DEF_AIR_TIME * THOUSAND; // keeps number of ms to run air scour
// uint rinseTime = DEF_RINSE_TIME * THOUSAND;  // keeps number of ms to run rinse
// uint bwashTime = DEF_BWASH_TIME * THOUSAND;  // keeps number of ms to run bwash
// uint HP_CleanCount = 0;                      // stores the number of cleans done before
// volatile uint pulses = 0;                    // stores pulses of the flow meter
// ulong now = 0;                               // current timestamp for loop (avoids repeated calls to millis())
// ulong entryTimeStamp = 0;                    // time stamp for entry to states

/* Non-volatile Storage */
// Save pref;
/* HMI */
// nextion HMI(BAUD_RATE);

/* Flags */
// bool startup_flag = true;      // startup flag
// bool entry_flag = true;        // true on entry to state to set initial conditions
// bool CEB_state_flag = false;   // true when the CEB needs to be run
// bool tank_full = false;        // true if permeate tank float switch is open (tank is full)
// bool feed_low = false;         // true if the feed tank float is open (feed tank is empty)
// bool low_pressure_feed = true; // true if in good status(there is no HP on feed)

/* General Functions */
void Interrupts(void);
// void NextStartup(void);
// void ChangeState(uint stateTo);
// void CheckFloat(void);
// void CheckHP_Switch(void);
// void CalculateFlow(void);
// void Error(char ErrCode);
// void IRAM_ATTR flowInterrupt(void);

/* Nextion Functions */
// void Listen(void);      // listens for items on serial port from Nextion touch display
// void SetState(void);    // sets state on display
// void SetTime(uint val); // sets time remaining on main display page
// void SetTotal(const char *time_var, uint val);
// void SetFlow(int flow_val);
// void EndTrans(void);
// void NexPrintMain(void);
// void PageHandler(uint page);
// void ManualButtonHandler(uint type, uint n, bool on_off);
// void SendManualButtonStatus(uint button = 10);
// void CEB_ButtonHandler(uint button);
// void TimeValueHandler(int time);
// void RTC_Handler(uint rtc6);
// void SendStateIconStatus(uint state_for_status);
bool ADC_ok = 0;

TaskHandle_t C0;
void core0loop(void *pvParameters);

ADS1115 ADC_1(0x48);

//********************** SETUP **********************
void setup()
{
  Serial.begin(115200, SERIAL_8N1, 44, 43);
  PinSetup();
  Interrupts();
  ADC_ok = AnalogSetup(ADC_1);
  //  pref.FileSetup();                            // File System setup and reads

  // EndTrans(); // Ensure serial comms is clean on the nextion
  // delay(100);
  // while (Comms.available()) // clean the serial input buffer out
  //   Comms.read(); nextion
  // CheckFloat();
  // startup_flag = 0;   // set the startup flag to 0
  // ChangeState(state); // start state here
  // debug("State: ");
  // debugln(state);
  // disableCore1WDT();
  xTaskCreatePinnedToCore(core0loop, "Core_0", 10000, NULL, 1, &C0, 0);
}

//********************** CALIBRATION **********************
#define VoltageFactor 2510 //
//  1V = 2500
#define CurrentFactor 1185
/*
      Values     V      I
      board 1    2483   1185
      board 2    2500   ?
      board 3    2510   ?
      board 4    2510   ?
      board 5    2480   ?
*/

//********************** MAIN **********************
void loop()
{
  static int Counter = 0;
  ulong t_stamp;
  int A0, A1, A2, A3;

  // Serial.println(Counter++);
  //  if (ADC_ok)
  //  {
  //    t_stamp = millis();
  //    Serial.printf("Start time: %lu \n", millis());
  //    A0 = ADC_1.readADC(0);
  //    t_stamp = millis();
  //    Serial.printf("ADC 0: %d \t Voltage: %fV \t Current: %fmA \t time: %lu \n ", A0, float(A0)/VoltageFactor ,float(A0)/CurrentFactor, millis()-t_stamp);
  //    A0 = ADC_1.readADC(1);
  //    t_stamp = millis();
  //    Serial.printf("ADC 1: %d \t Voltage: %fV \t Current: %fmA \t time: %lu \n ", A0, float(A0)/VoltageFactor, float(A0)/CurrentFactor, millis()-t_stamp);
  //    A0 = ADC_1.readADC(2);
  //    t_stamp = millis();
  //    Serial.printf("ADC 2: %d \t Voltage: %fV \t Current: %fmA \t time: %lu \n ", A0, float(A0)/VoltageFactor, float(A0)/CurrentFactor, millis()-t_stamp);
  //    A0 = ADC_1.readADC(3);
  //    t_stamp = millis();
  //    Serial.printf("ADC 3: %d \t Voltage: %fV \t Current: %fmA \t time: %lu \n ", A0, float(A0)/VoltageFactor, float(A0)/CurrentFactor, millis()-t_stamp);
  //  }
  //  else
  //    debugln("Wire error: check ADS1115");

  if (ReadInput(D_IN_1))
    WriteOutput(D_OUT_1, true);
  else
    WriteOutput(D_OUT_1, false);

  if (ReadInput(D_IN_2))
    WriteOutput(D_OUT_2, true);
  else
    WriteOutput(D_OUT_2, false);

  if (ReadInput(D_IN_3))
    WriteOutput(D_OUT_3, true);
  else
    WriteOutput(D_OUT_3, false);

  if (ReadInput(D_IN_4))
    WriteOutput(D_OUT_4, true);
  else
    WriteOutput(D_OUT_4, false);

  if (ReadInput(D_IN_5))
    WriteOutput(D_OUT_5, true);
  else
    WriteOutput(D_OUT_5, false);

  if (ReadInput(D_IN_6))
    WriteOutput(D_OUT_6, true);
  else
    WriteOutput(D_OUT_6, false);

  if (ReadInput(D_IN_7))
    WriteOutput(D_OUT_7, true);
  else
    WriteOutput(D_OUT_7, false);

  if (ReadInput(D_IN_8))
    WriteOutput(D_OUT_8, true);
  else
    WriteOutput(D_OUT_8, false);

  delay(1010);
}

void core0loop(void *pvParameters)
{
  delay(20);
  BlynkEdgent.begin();
  delay(100);
  while (1)
  {
    BlynkEdgent.run();
    delay(1);
  }
}

void Interrupts(void)
{
  // attachInterrupt(digitalPinToInterrupt(FLOW), flowInterrupt, RISING);
}
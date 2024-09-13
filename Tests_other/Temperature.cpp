#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_TEMPLATE_ID "TMPL2E91bPiCL"
#define BLYNK_TEMPLATE_NAME "RO"

#include <unity.h>
#include <BlynkEdgent.h>
#include "Definitions.h"
#include "Nextion.h"


void setup()
{
    // use Serial (port 0); initialize Modbus communication baud rate
    Serial.begin(115200);
    Serial.println("starting");
    delay(500);
    TestTemp.Begin();
}

void loop()
{
    ulong time1 = 0;
    time1 = millis();
    if (TestTemp.ReadTemp())
    {
        Serial.println(TestTemp.currTemp);
        time1 -= millis();
        Serial.print("time");
        Serial.println(time1);
    }
    delay(1);
}
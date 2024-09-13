#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_TEMPLATE_ID "TMPL2E91bPiCL"
#define BLYNK_TEMPLATE_NAME "RO"

#include <ModbusMaster.h>
#include <unity.h>
#include <BlynkEdgent.h>
#include "Definitions.h"
#include "Nextion.h"

// instantiate ModbusMaster object
ModbusMaster node;

#define MODBUS_SLAVE_ID 1

void setup()
{
    // use Serial (port 0); initialize Modbus communication baud rate
    Serial.begin(115200);
    //Serial1.begin(19200, SERIAL_8E1, 13, 14);
    Serial1.begin(19200, SERIAL_8E1, 11, 12);

    // communicate with Modbus slave ID 2 over Serial (port 0)
    node.begin(MODBUS_SLAVE_ID, Serial1);
    Serial.println("starting");
    delay(500);
}

void loop()
{
    static uint32_t i;
    uint8_t j, result;
    uint16_t data[6];

    i++;

    //   // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
    Serial.println(node.setTransmitBuffer(0, i++));

    //   // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    Serial.println(node.setTransmitBuffer(1, i++));

        //   // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
    Serial.println(node.setTransmitBuffer(2, i++));

    //   // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    Serial.println(node.setTransmitBuffer(3, i++));

    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    result = node.writeMultipleRegisters(5, 4);
    Serial.print("Write multi:");
    Serial.println(result);
    delay(1000);
    Serial.print("Write multi manual:");
    Serial.println(result);
    // result = node.writeSingleRegister(0, 22);
    // Serial.print("Write:");
    // Serial.println(result);
    // // slave: read (6) 16-bit registers starting at register 2 to RX buffer
    // result = node.readHoldingRegisters(0, 6);
    // Serial.print("Read:");
    // Serial.println(result);
    // // do something with data if read is successful
    // if (result == node.ku8MBSuccess)
    // {
    //     for (j = 0; j < 6; j++)
    //     {
    //         data[j] = node.getResponseBuffer(j);
    //         Serial.println(data[j]);
    //     }
    // }
    delay(5000);
    Serial.println("starting");
}
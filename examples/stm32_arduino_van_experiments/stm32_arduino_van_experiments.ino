#include <Arduino.h>
#include "stm32_arduino_van_bus.h"

Stm32ArduinoVanBus* vanBus;

uint32_t prevTime = 0;
bool filter = false;

void ShowPopupMessage(int messageId)
{
    uint8_t packet[14] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, messageId, 0xFF, 0xFF, 0xFF, 0xFF };

    vanBus->SendNormalFrame(0x524, packet, 14, 0);
}

void SendExternalTemperature(int temperature)
{
    uint8_t packet[7] = { 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, temperature * 2  + 0x50};

    vanBus->SendNormalFrame(0x8A4, packet, 7, 0);
}

void setup()
{
    Serial.begin(500000);

    vanBus = new Stm32ArduinoVanBus();

    //vanBus->Init(PA6, PA7, PC13, VAN_LINE_LEVEL_LOW, VAN_NETWORK_TYPE_COMFORT);//1.4
    vanBus->Init(PA6, PA7, PB14, VAN_LINE_LEVEL_LOW, VAN_NETWORK_TYPE_COMFORT);// fake V2C
    //vanBus->Init(PA6, PA7, PC13, VAN_LINE_LEVEL_LOW, VAN_NETWORK_TYPE_BODY);

    Serial.println("RESET");
}

void loop()
{
    uint32_t currTime = millis();
    if (currTime - prevTime > 10)
    {
        prevTime = currTime;

        uint8_t vanMessage[32];
        uint8_t msgLength;
        char tmp[3];

        if (vanBus->Receive(vanMessage, &msgLength))
        {
          if (filter){
            if (vanMessage[1] == 0x56){
              if (!vanBus->IsCrcOk(vanMessage, msgLength)) {
                  Serial.print("CRC ERROR: ");
              }
              for (int i = 0; i < msgLength; i++)
              {
                  snprintf(tmp, 3, "%02X", vanMessage[i]);
                  Serial.print(tmp);
                  Serial.print(" ");
              }
              Serial.println();
            }
          }
          else{
              if (!vanBus->IsCrcOk(vanMessage, msgLength)) {
                  Serial.print("CRC ERROR: ");
              }
              for (int i = 0; i < msgLength; i++)
              {
                  snprintf(tmp, 3, "%02X", vanMessage[i]);
                  Serial.print(tmp);
                  Serial.print(" ");
              }
              Serial.println();
            
          }
        }
    }
    if (Serial.available() > 0)
    {
        int inChar = Serial.read();
        Serial.println(inChar);
        switch (inChar)
        {
            case 'd':
            {
                ShowPopupMessage(0x09);
                break;
            }
            case 'f':
            {
                ShowPopupMessage(0x18);
                break;
            }
            case 't': {
                SendExternalTemperature(25);
                break;
            }
            case 'e': {
                vanBus->SendQueryFrame(0x564);
                break;
            }
            case 'r': {
                vanBus->SendQueryFrameForDeferredReply(0x564);
                break;
            }
            case 'q': {
                filter = !filter;
                break;
            }
        }
    }
}

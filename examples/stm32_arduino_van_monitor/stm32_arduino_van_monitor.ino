#include <Arduino.h>
#include <stm32_arduino_van_bus.h>

Stm32ArduinoVanBus* vanBus;
uint32_t prevTime = 0;
	
void setup()
{
    Serial.begin(500000);

    vanBus = new Stm32ArduinoVanBus();
    vanBus->Init(PA6, PA7, PC13, VAN_LINE_LEVEL_LOW, VAN_NETWORK_TYPE_COMFORT);
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
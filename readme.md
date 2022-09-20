# Vehicle Area Network (VAN bus) Reader/Writer for STM32

## Description

VAN bus is pretty similar to CAN bus. It was used in many cars (Peugeot, Citroen) made by PSA from 2001 to 2005.

This is a VAN bus library for the STM32 boards. It was developed on a STM32F103 BluePill, but it should work with other STM32 based boards as well. It supports reading and also writing to the bus. It works on the 125 kbits/sec bus (VAN_COMFORT), and the 62.5 kbits/sec bus (VAN_BODY) as well (however only reading was tested on the BODY bus)

It doesn't support boards other than the STM32 family yet, but it shouldn't be difficult to add a new board which is powerful enough. For other boards like the ESP32, ESP8266 or a generic SPI library with a TSS463 IC you can find libraries in the **See also** section. 

## Schematics

![schema](https://github.com/morcibacsi/stm32_arduino_van_bus/blob/master/extras/schema/stm32_mcp2551-iso-a.jpg)

## Example

```cpp
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
```

## Sending data on the VAN bus

**Sending has been implemented, but it doesn't really work in a car, so I strongly advise  not to use it.** However if you   can make it reliable, a PR is very welcome. For examples on how to send data on the bus you should check the examples folder. 

## Sample output

![output](https://github.com/morcibacsi/stm32_arduino_van_bus/blob/master/extras/stm32_van_capture.gif)

## Installation

You need to install the official STM32 core based on the info from here: https://github.com/stm32duino/Arduino_Core_STM32#getting-started

Copy the following files to your **documents\Arduino\libraries\stm32_arduino_van_bus** folder

- stm32_arduino_van_bus.h
- stm32_arduino_van_bus.cpp
- keywords.txt
- library.properties

Don't forget to install the **RingBuffer** library from the next section as well!

## Used library

You need to have the following library installed:
- https://github.com/Locoduino/RingBuffer/

## Thanks

A **HUGE thanks** goes **to XuHaojie** for his work. His STM32 standard peripheral library was the base of this Arduino library.

## See also

- [VAN Bus reader library for ESP32](https://github.com/morcibacsi/esp32_rmt_van_rx)
- [VAN Bus library for ESP8266](https://github.com/0xCAFEDECAF/VanBus)
- [VAN Bus library based on Atmels TSS463/TSS461 IC](https://github.com/morcibacsi/arduino_tss463_van)
- [VAN Bus Analyzer for Saleae Logic Analyzer](https://github.com/morcibacsi/VanAnalyzer)

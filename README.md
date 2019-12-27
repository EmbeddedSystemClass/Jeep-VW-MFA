# VW MFA for my engine swapped Jeep

This project implements a VAG compatible MFA on M5Stack.

Things needed for this project:
1) M5Stack Core (LCD/ESP32)
2) M5Stack Base (Battery/dock interface)
3) M5Stack GPS (used for speed)
4) M5Stack Proto (CAN interface goes here)
5) 3.3v compatible CAN transciever
6) M5Stack BTC Standing Base (Used for holding the display)

You could get away with only a Core and a Proto base, and mount the setup with a mag base or something like that.

I have the following pin config:
|pin|function|shield|
|----|--------|-------|
| 5  | CAN TX | proto |
| 2  | CAN RX | proto |
| 16 | GPS RX | GPS   |
| 17 | GPS TX | GPS   |

I will be adding an Adafruit i2c ADC for getting fuel level, oil pressure, oil temperature, and steering wheel buttons. I will also be adding an Adafruit K-Type thermocouple amp/interface for EGTs. Once all of that is done I will FINALLY get CCD reimplemented to drive my gauges and use the M5Stack LCD for datalogging and monitoring. I have also tossed around the idea of controlling my webasto via LoRaWan, so keep tuned for potential remote capabilities!

The M5Stack setup is pretty slick. I am using platformio (Arduino) and vscode for my development environment, as I couldn't get the M5Stack library converted the the CMake format that esp-idf 4+ uses. If that ever gets updated, I might rewrite this outside of platformio.
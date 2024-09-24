# EasyDMX
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/tesa-klebeband/library/EasyDMX.svg)](https://registry.platformio.org/libraries/tesa-klebeband/EasyDMX)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/tesa-klebeband/EasyDMX)
![GitHub License](https://img.shields.io/github/license/tesa-klebeband/EasyDMX)

![image](https://github.com/user-attachments/assets/2e123411-b053-4351-9662-d673952ae8de)

EasyDMX is a library designed for sending and receiving DMX512 with an ESP32, aimed at providing simplicity and ease of use

## Features
- Easily send and receive DMX512 with an ESP32 and MAX485 (it takes just one line to get everything set up)
- Operate a transmitter and receiver at the same time
- Ability to transmit the received DMX data automatically

## Installation
Create a new PlatformIO project and add the following to your `platformio.ini` file (append to the existing `lib_deps` if it already exists by creating a new line with the same indentation):
```ini
lib_deps =
    https://github.com/tesa-klebeband/EasyDMX
```

## Using EasyDMX
Examples on different operating modes are provided in the `examples` folder. To use EasyDMX in your project, include the library and create an instance of the `EasyDMX` class. The following example demonstrates how to set up a simple dmx receiver that logs the received channels to the serial monitor:
```cpp
#include <easydmx.h>

EasyDMX dmx;

void setup() {
    Serial.begin(115200);
    dmx.begin(DMXMode::Receive, DMXPin::Serial2Rx, DMXPin::NoTx);
}

void loop() {
    for (int i = 1; i <= 512; i++) {
        Serial.print(dmx.getChannel(i));
        Serial.print(" ");
    }
    Serial.println();
    delay(1000);
}
```

## License
All files within this repo are released under the GNU GPL V3 License as per the LICENSE file stored in the root of this repo.

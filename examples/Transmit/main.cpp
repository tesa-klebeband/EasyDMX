/*
 * This file is part of EasyDMX (https://github.com/tesa-klebeband/EasyDMX).
 * Copyright (c) 2024 tesa-klebeband.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <easydmx.h>

// Create an instance of the EasyDMX class, so we can use its functions
EasyDMX dmx;

void setup() {
    Serial.begin(115200);
    /**
     * Start the DMX driver in transmit mode on Serial2 (pin 17)
     * The MAX485's DI pin should be connected to the TX pin of the ESP32
     *  and RE and DE should be connected to 3.3V.
    */
    dmx.begin(DMXMode::Transmit, DMXPin::NoRx, DMXPin::Serial2Tx);
}

void loop() {
    // Set channel 1 to 255
    dmx.setChannel(1, 255);
    delay(1000);
    // Set channel 1 to 0
    dmx.setChannel(1, 0);
    delay(1000);
}
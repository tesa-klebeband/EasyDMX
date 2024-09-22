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
     * Start the DMX driver in receive mode on Serial2 (pin 16)
     * The MAX485's RO pin should be connected to the RX pin of the ESP32
     *  and RE and DE should be connected to GND.
    */
    dmx.begin(DMXMode::Receive, DMXPin::Serial2Rx, DMXPin::NoTx);
}

void loop() {
    // Print the value of channel 1
    Serial.println(dmx.getChannel(1));
    delay(1000);
}
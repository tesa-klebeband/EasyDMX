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
     * Starts the DMX driver in dual (both transmit and receive) mode on Serial2 (pins 16 and 17).
     * This example requires two MAX485 chips.
     * The receiving MAX485's RO pin should be connected to the RX pin of the ESP32 (pin 16)
     *  and RE and DE should be connected to GND.
     * The transmitting MAX485's DI pin should be connected to the TX pin of the ESP32 (pin 17)
     *  and RE and DE should be connected to 3.3V.
     */
    dmx.begin(DMXMode::Both, DMXPin::Serial2Rx, DMXPin::Serial2Tx);
}

void loop() {
    // Receive Channel 1, modify it by +1 and transmit it back
    dmx.setChannel(1, dmx.getChannel(1) + 1);
}
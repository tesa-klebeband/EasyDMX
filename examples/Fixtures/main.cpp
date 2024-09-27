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

// Define the start address of the light fixture
#define LIGHT_START_ADDRESS 1

// Create an instance of the EasyDMX class, so we can use its functions
EasyDMX dmx;

// Construct a DMX Universe and give it access to the DMX driver
DMXUniverse universe(&dmx);

// Describe a simple RGB fixture with 3 channels
DMXFixtureDescriptor light_descriptor(3, DMXChannelType::Red, DMXChannelType::Green, DMXChannelType::Blue);

// Create a fixture based on the descriptor
DMXFixture light(&light_descriptor, LIGHT_START_ADDRESS);

void setup() {
    Serial.begin(115200);

    // Add the fixture to the universe
    universe.addFixture(&light);

    /**
     * Start the DMX driver in transmit mode on Serial2 (pin 17)
     * The MAX485's DI pin should be connected to the TX pin of the ESP32
     *  and RE and DE should be connected to 3.3V.
    */
    dmx.begin(DMXMode::Transmit, DMXPin::NoRx, DMXPin::Serial2Tx);
}

void loop() {
    // Set the RGB values of the light fixture and update the universe to transmit the data
    light.setChannel(DMXChannelType::Red, 255);
    light.setChannel(DMXChannelType::Green, 0);
    light.setChannel(DMXChannelType::Blue, 0);
    universe.update();
    delay(1000);

    light.setChannel(DMXChannelType::Red, 0);
    light.setChannel(DMXChannelType::Green, 255);
    light.setChannel(DMXChannelType::Blue, 0);
    universe.update();
    delay(1000);

    light.setChannel(DMXChannelType::Red, 0);
    light.setChannel(DMXChannelType::Green, 0);
    light.setChannel(DMXChannelType::Blue, 255);
    universe.update();
    delay(1000);
}
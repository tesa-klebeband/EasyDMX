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

#ifndef EASYDMX_H
#define EASYDMX_H

#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <vector>

#ifndef UART_NUM_2
#define DMX_UART_NUM UART_NUM_1
#else
#define DMX_UART_NUM UART_NUM_2
#endif

#define UART_BUF_SIZE (1024 * 2)

/**
 * This enum represents the different modes the DMX driver can operate in.
 * The mode determines whether the driver will transmit, receive or both transmit and receive DMX data.
 */
enum class DMXMode {
    Transmit,   // Transmit only
    Receive,    // Receive only
    Both,       // Both transmit and receive
    BothKeepRx  // Both transmit and receive, but forbid writing to the transmit buffer and instead use the receive buffer for transmitting
};

/**
 * This enum represents the different pins that can be used for DMX communication.
 * The pins are used to configure the UART module for DMX communication.
 */
enum DMXPin {
    NoRx = 0,
    NoTx = 0,
    Serial2Rx = 16,  // Standard RX pin for Serial2
    Serial2Tx = 17   // Standard TX pin for Serial2
};

/**
 * This enum represents the different states the DMX receiver can be in.
 * The state determines whether the receiver is waiting for a break, start code or data.
 */
enum class DMXStateRx {
    Idle,   // Waiting for a break
    Break,  // Break received, waiting for the start code
    Data    // Receiving channels
};

/**
 * This enum represents the different types of DMX channels.
 * The values of the enum are used to identify the type of a channel in a DMXFixtureDescriptor.
 */
enum DMXChannelType {
    None,   // No channel, this channel value is always 0 and cannot be changed
    Dimmer,
    Red,
    Green,
    Blue,
    White,
    Amber,
    UV,
    Strobe,
    UVStrobe,
    ColorWheel,
    Pan,
    Tilt,
    PanFine,
    TiltFine,
    Zoom,
    Focus,
    Iris,
    Shutter,
    Speed,
    Fog,
    Rotation,
    Prism,
    PrismRotation,
    Gobo,
    GoboRotation,
    Custom1,
    Custom2,
    Custom3,
    Custom4,
    Custom5,
};

/**
 * The EasyDMX main class.
 * This class provides functions to control DMX communication, send and receive DMX data and transmit Universes.
 * Use begin(DMXMode mode, int rx_pin, int tx_pin) to initialize the DMX driver.
 */
class EasyDMX {
public:
    /**
     * Initializes the DMX driver with the given pins.
     * Two max485 modules are required if you want to both transmit and receive DMX data.
     * Switching the direction of one module is not supported.
     * It is mandatory to call this function before using any other function of the library.
     * Make sure to pull down DE and RE for the receiver and pull them up for the transmitter.
     * Rx should be connected to RO and Tx should be connected to DI.
     *
     * Operating modes:
     *
     *  - Transmit: The driver will only transmit DMX data. Channels are set via setChannel and their values can be read via getChannelTx.
     *
     *  - Receive: The driver will only receive DMX data. Incoming channels can be read via getChannel.
     *
     *  - Both: The driver will both transmit and receive DMX data. Channels can be set via setChannel and their values can be read via getChannelTx. Incoming channels can be read via getChannel.
     *
     *  - BothKeepRx: The driver will read incoming DMX data and transmit it. Channels can be read via getChannel or getChannelTx. Setting channels via setChannel is not allowed.
     *
     * @param mode The mode of the DMX driver. Can be Transmit, Receive, Both or BothKeepRx. Depending on the mode, the pins can be set to NoRx or NoTx.
     * @param rx_pin The pin connected to the RO pin of the max485. If mode is set to Transmit, this pin has to be set to NoRx.
     * @param tx_pin The pin connected to the DI pin of the max485. If mode is set to Receive, this pin has to be set to NoTx.
     */
    void begin(DMXMode mode, int rx_pin, int tx_pin);

    /**
     * Stops the DMX driver and its associated tasks.
     */
    void end();

    /**
     * Sets a DMX channel to a specific value.
     * @param channel The channel to set. Must be between 1 and 512.
     * @param value The value to set the channel to. Must be between 0 and 255.
     */
    void setChannel(int channel, uint8_t value);

    /**
     * Gets the value of a DMX channel.
     * If the mode is set to BothKeepRx, this function does nothing.
     * @param channel The channel to get. Must be between 1 and 512.
     * @return The value of the channel as a uint8_t.
     */
    uint8_t getChannel(int channel);

    /**
     * Gets the value of a DMX channel in the transmit buffer.
     * If the mode is set to BothKeepRx, this function will return the same value as getChannel.
     * @param channel The channel to get. Must be between 1 and 512.
     * @return The value of the channel as a uint8_t.
     */
    uint8_t getChannelTx(int channel);

private:
    void* dmxTxTask();
    void* dmxRxTask();
    TaskHandle_t dmx_tx_task_handle;
    TaskHandle_t dmx_rx_task_handle;
    int rx_pin;
    int tx_pin;
    uint8_t dmx_data_tx[513];
    uint8_t dmx_data_rx[513];
    DMXMode mode;
    QueueHandle_t uart_queue;
};

/**
 * This class is used to describe a DMX fixture.
 * The constructor takes a variable number of arguments, each representing a channel type.
 * The order of the channel types represents the order of the channels in the fixture.
 *
 * Example:
 *
 * - DMXFixtureDescriptor descriptor(3, DMXChannelType::Red, DMXChannelType::Green, DMXChannelType::Blue);
 *
 * @param num_channels The number of channels the fixture has.
 * @param va_list The list of channel types, each represented as a DMXChannelType.
 */
class DMXFixtureDescriptor {
public:
    DMXFixtureDescriptor(uint16_t num_channels, ...);

    /**
     * Destructor for the DMXFixtureDescriptor.
     */
    ~DMXFixtureDescriptor();

private:
    DMXChannelType* channel_types;
    uint16_t num_channels;

    friend class DMXFixture;
    friend class DMXUniverse;
};

/**
 * Constructs a DMX fixture with the given descriptor and start address.
 * @param descriptor A pointer to the descriptor of the fixture. The descriptor must stay in scope for the entire lifetime of the fixture.
 * @param address The address of the fixture. Must be between 1 and 512.
 */
class DMXFixture {
public:
    DMXFixture(DMXFixtureDescriptor* descriptor, uint16_t address);

    /**
     * Sets the value of all channels with the given type to the given value.
     * @param type The type of the channels to set. Must be a valid DMXChannelType.
     * @param value The value to set the channels to. Must be between 0 and 255.
     */
    void setChannel(DMXChannelType type, uint8_t value);

    /**
     * Sets the value of the channel at the given index.
     * A call to setChannel(DMXChannelType, uint8_t) may overwrite the value set by this function.
     * @param channel The index of the channel to set. Must be between 0 and the number of channels in the fixture.
     */
    inline void setChannel(uint16_t channel, uint8_t value) {
        if (channel >= descriptor->num_channels) {
            return;
        }
        channels[channel] = value;
    }

    /**
     * Gets the value of the first channel with the given type.
     * To retrieve the value of a specific channel independent of its type, use getChannel(int channel).
     * @param type The type of the channel to get. Must be a valid DMXChannelType.
     * @return The value of the channel as a uint8_t.
     */
    uint8_t getChannel(DMXChannelType type);

    /**
     * Gets the value of the channel at the given index.
     * @param channel The index of the channel to get. Must be between 0 and the number of channels in the fixture.
     * @return The value of the channel as a uint8_t.
     */
    inline uint8_t getChannel(uint16_t channel) {
        if (channel >= descriptor->num_channels) {
            return -1;
        }
        return channels[channel];
    }

    /**
     * Changes the start address of the fixture.
     * @param address The new address of the fixture.
     */
    inline void setAddress(uint16_t address) {
        if (address < 1 || address > 512) {
            return;
        }
        this->address = address;
    }

    /**
     * Returns the start address of the fixture.
     * @return The start address of the fixture.
     */
    inline uint16_t getAddress() {
        return address;
    }

private:
    DMXFixtureDescriptor* descriptor;
    uint8_t* channels;
    uint16_t address;

    friend class DMXUniverse;
};

/**
 * Constructs a DMX universe with the given EasyDMX instance.
 * @param dmx A pointer to the EasyDMX instance to use for communication.
 */
class DMXUniverse {
public:
    DMXUniverse(EasyDMX* dmx);

    /**
     * Adds a fixture to the DMX universe.
     * Fixtures have to stay in the object's scope the entire time. If a fixture is deleted or goes out of scope, it needs to be removed from the universe before that.
     * @param fixture A pointer to the fixture to add.
     */
    void addFixture(DMXFixture* fixture);

    /**
     * Removes a fixture from the DMX universe by pointer.
     * @param fixture A pointer to the fixture to remove. If the fixture is not in the universe, this function does nothing.
     */
    void removeFixture(DMXFixture* fixture);

    /**
     * Removes the fixture at the given address from the DMX universe.
     * @param address The address of the fixture to remove.
     */
    void removeFixture(uint16_t address);

    /**
     * Updates the DMX universes data by going through all fixtures and updating their channels.
     */
    void update();

private:
    std::vector<DMXFixture*> fixtures;
    EasyDMX* dmx;
    uint8_t dmx_data[513];
};

#endif

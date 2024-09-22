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

#define DMX_UART_NUM UART_NUM_2
#define UART_BUF_SIZE 1024

enum class DMXMode {
    Transmit,   // Transmit only
    Receive,    // Receive only
    Both,       // Both transmit and receive
    BothKeepRx  // Both transmit and receive, but forbid writing to the transmit buffer and instead use the receive buffer for transmitting
};

enum DMXPin {
    NoRx = 0,
    NoTx = 0,
    Serial2Rx = 16,  // Standard RX pin for Serial2
    Serial2Tx = 17   // Standard TX pin for Serial2
};

enum class DMXStateRx {
    Idle,   // Waiting for a break
    Break,  // Break received, waiting for the start code
    Data    // Receiving channels
};

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

#endif
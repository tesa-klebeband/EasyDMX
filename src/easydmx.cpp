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

#include <easydmx.h>

/**
 * Starts the DMX driver with the given pins.
 * For more information, see the documentation in easydmx.h.
 */
void EasyDMX::begin(DMXMode mode, int rx_pin, int tx_pin) {
    this->rx_pin = rx_pin;
    this->tx_pin = tx_pin;
    this->mode = mode;

    // Configure the UART for DMX
    uart_config_t uart_config = {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(DMX_UART_NUM, &uart_config);
    uart_set_pin(DMX_UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(DMX_UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_queue, 0);

    // Based on the operating mode, create the appropriate task
    if (mode == DMXMode::Transmit || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        xTaskCreatePinnedToCore([](void* pvParameters) {
            EasyDMX* dmx = static_cast<EasyDMX*>(pvParameters);
            dmx->dmxTxTask();
        },
                                "dmxTxTask", 1024, this, 1, &dmx_tx_task_handle, 1);
    }

    if (mode == DMXMode::Receive || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        xTaskCreatePinnedToCore([](void* pvParameters) {
            EasyDMX* dmx = static_cast<EasyDMX*>(pvParameters);
            dmx->dmxRxTask();
        },
                                "dmxRxTask", 2048, this, 1, &dmx_rx_task_handle, 1);
    }

    memset(dmx_data_tx, 0, 513);
    memset(dmx_data_rx, 0, 513);
}

/**
 * Stops the DMX driver and its associated tasks.
 * For more information, see the documentation in easydmx.h.
 */
void EasyDMX::end() {
    // Stop the tasks based on the operating mode
    if (mode == DMXMode::Transmit || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        vTaskDelete(dmx_tx_task_handle);
    } else if (mode == DMXMode::Receive || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        vTaskDelete(dmx_rx_task_handle);
    }
}

/**
 * Sets a DMX channel to a specific value.
 * For more information, see the documentation in easydmx.h.
 */
void EasyDMX::setChannel(int channel, uint8_t value) {
    // Don't allow setting the channel if the mode is set to BothKeepRx
    if (mode == DMXMode::BothKeepRx) {
        return;
    }
    // Do not allow setting the start code
    if (channel < 1 || channel > 512) {
        return;
    }
    dmx_data_tx[channel] = value;
}

/**
 * Gets the value of a DMX channel.
 * For more information, see the documentation in easydmx.h.
 */
uint8_t EasyDMX::getChannel(int channel) {
    // Prevent access faults
    if (channel < 1 || channel > 512) {
        return 0;
    }

    return dmx_data_rx[channel];
}

/**
 * Gets the value of a DMX channel in the transmit buffer.
 * For more information, see the documentation in easydmx.h.
 */
uint8_t EasyDMX::getChannelTx(int channel) {
    // Prevent access faults
    if (channel < 1 || channel > 512) {
        return 0;
    }

    return dmx_data_tx[channel];
}

/**
 * The task that sends the DMX data.
 */
void* EasyDMX::dmxTxTask() {
    uint8_t start_code = 0;
    while (true) {
        uart_wait_tx_done(DMX_UART_NUM, 1000);                              // Wait for the UART to finish sending
        uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV);           // Invert the TX line for the break (LOW)
        ets_delay_us(100);                                                  // Wait for the break to be sent
        uart_set_line_inverse(DMX_UART_NUM, 0);                             // Revert the TX line back to normal (HIGH)
        ets_delay_us(14);                                                   // Wait for the Mark After Break (MAB) to finish
        uart_write_bytes(DMX_UART_NUM, (const char*)&start_code, 1);        // Send the start code
        uart_write_bytes(DMX_UART_NUM, (const char*)dmx_data_tx + 1, 512);  // Send the DMX data
    }
}

/**
 * The task that receives the DMX data.
 */
void* EasyDMX::dmxRxTask() {
    // Create the necessary structures and allocate memory
    uart_event_t event;
    uint8_t* data = (uint8_t*)malloc(UART_BUF_SIZE);
    DMXStateRx state = DMXStateRx::Idle;
    int current_rx_addr = 0;

    while (true) {
        if (xQueueReceive(uart_queue, (void*)&event, portMAX_DELAY)) {           // Wait for an event to be received
            memset(data, 0, UART_BUF_SIZE);                                      // Clear the data buffer
            if (event.type == UART_DATA) {                                       // Check if the event is a data event
                uart_read_bytes(DMX_UART_NUM, data, event.size, portMAX_DELAY);  // Read the data from the UART

                if (state == DMXStateRx::Break) {  // Handle a break, reset the state and the current address
                    if (data[0] == 0) {
                        state = DMXStateRx::Data;
                        current_rx_addr = 0;
                    }
                }
                if (state == DMXStateRx::Data) {  // Read the data to the current address in the buffer
                    for (int i = 0; i < event.size; i++) {
                        if (current_rx_addr < 513) {
                            if (mode == DMXMode::BothKeepRx) {
                                dmx_data_tx[current_rx_addr] = data[i];
                            }
                            dmx_data_rx[current_rx_addr++] = data[i];
                        }
                    }
                }
            } else {                                                                        // Either break, idle or other events (such as errors)
                uart_flush(DMX_UART_NUM);                                                   // Flush the UART buffer
                xQueueReset(uart_queue);                                                    // Reset the queue
                state = (event.type == UART_BREAK) ? DMXStateRx::Break : DMXStateRx::Idle;  // Set the state based on the event type
            }
        }
    }
}
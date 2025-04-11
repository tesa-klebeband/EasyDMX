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
  * List of UART numbers in descending order. This way the UART used typically for serial communication is used last.
  * If 3 (for some chips only 2) EasyDMX instances are required, the Arduino Serial library shouldn't be used anymore.
  */
const uart_port_t UART_MAP[] = {
#ifndef UART_NUM_2
    UART_NUM_1,
    UART_NUM_0
#else
    UART_NUM_2,
    UART_NUM_1,
    UART_NUM_0
#endif
};

int next_uart = 0;

/**
 * Starts the DMX driver with the given pins.
 */
int EasyDMX::begin(DMXMode mode, int rx_pin, int tx_pin) {
    if (initialized) {
        return -1;
    }
    initialized = true;

    if (next_uart >= sizeof(UART_MAP) / sizeof(UART_MAP[0])) {
        return -1;
    }

    dmx_uart_num = UART_MAP[next_uart++];

    this->rx_pin = rx_pin;
    this->tx_pin = tx_pin;
    this->mode = mode;

    // Configure the UART for DMX
    uart_config_t uart_config = {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(dmx_uart_num, &uart_config);
    uart_set_pin(dmx_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(dmx_uart_num, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_queue, 0);

    // Based on the operating mode, create the appropriate task
    if (mode == DMXMode::Transmit || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        dmx_data_tx = (uint8_t*)malloc(513);
        memset(dmx_data_tx, 0, 513);
        dmx_tx_mutex = xSemaphoreCreateMutex();

        xTaskCreate([](void* pvParameters) {
            EasyDMX* dmx = static_cast<EasyDMX*>(pvParameters);
            dmx->dmxTxTask();
            },
            "dmxTxTask", 1024, this, 1, &dmx_tx_task_handle);
    }

    if (mode == DMXMode::Receive || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        dmx_data_rx = (uint8_t*)malloc(513);
        memset(dmx_data_rx, 0, 513);
        rx_buffer = (uint8_t*)malloc(UART_BUF_SIZE);
        dmx_rx_mutex = xSemaphoreCreateMutex();

        xTaskCreate([](void* pvParameters) {
            EasyDMX* dmx = static_cast<EasyDMX*>(pvParameters);
            dmx->dmxRxTask();
            },
            "dmxRxTask", 2048, this, 1, &dmx_rx_task_handle);
    }

    return 0;
}

/**
 * Stops the DMX driver and its associated tasks.
 */
void EasyDMX::end() {
    if (!initialized) {
        return;
    }

    // Stop the tasks based on the operating mode
    if (mode == DMXMode::Transmit || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        vTaskDelete(dmx_tx_task_handle);
        free(dmx_data_tx);
        dmx_data_tx = nullptr;
    }
    if (mode == DMXMode::Receive || mode == DMXMode::Both || mode == DMXMode::BothKeepRx) {
        vTaskDelete(dmx_rx_task_handle);
        free(dmx_data_rx);
        dmx_data_rx = nullptr;
    }
    
    uart_driver_delete(dmx_uart_num);
    free(rx_buffer);
    rx_buffer = nullptr;
    initialized = false;
}

/**
 * Sets a DMX channel to a specific value.
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

    if (xSemaphoreTake(dmx_tx_mutex, portMAX_DELAY)) {
        dmx_data_tx[channel] = value;
        xSemaphoreGive(dmx_tx_mutex);
    }
}

/**
 * Gets the value of a DMX channel.
 */
uint8_t EasyDMX::getChannel(int channel) {
    // Prevent access faults
    if (channel < 1 || channel > 512) {
        return 0;
    }

    uint8_t value = 0;
    if (xSemaphoreTake(dmx_rx_mutex, portMAX_DELAY)) {
        value = dmx_data_rx[channel];
        xSemaphoreGive(dmx_rx_mutex);
    }
    return value;
}

/**
 * Gets the value of a DMX channel in the transmit buffer.
 */
uint8_t EasyDMX::getChannelTx(int channel) {
    // Prevent access faults
    if (channel < 1 || channel > 512) {
        return 0;
    }

    uint8_t value = 0;
    if (xSemaphoreTake(dmx_tx_mutex, portMAX_DELAY)) {
        value = dmx_data_tx[channel];
        xSemaphoreGive(dmx_tx_mutex);
    }
    return value;
}

/**
 * The task that sends the DMX data.
 */
void EasyDMX::dmxTxTask() {
    uint8_t start_code = 0;
    while (true) {
        if (xSemaphoreTake(dmx_tx_mutex, portMAX_DELAY)) {
            uart_wait_tx_done(dmx_uart_num, 1000);                              // Wait for the UART to finish sending
            uart_set_line_inverse(dmx_uart_num, UART_SIGNAL_TXD_INV);           // Invert the TX line for the break (LOW)
            ets_delay_us(100);                                                  // Wait for the break to be sent
            uart_set_line_inverse(dmx_uart_num, 0);                             // Revert the TX line back to normal (HIGH)
            ets_delay_us(14);                                                   // Wait for the Mark After Break (MAB) to finish
            uart_write_bytes(dmx_uart_num, (const char*)&start_code, 1);        // Send the start code
            uart_write_bytes(dmx_uart_num, (const char*)dmx_data_tx + 1, 512);  // Send the DMX data
            xSemaphoreGive(dmx_tx_mutex);                                        // Release the mutex
        }
    }
}

/**
 * The task that receives the DMX data.
 */
void EasyDMX::dmxRxTask() {
    // Create the necessary structures and allocate memory
    uart_event_t event;
    DMXStateRx state = DMXStateRx::Idle;
    int current_rx_addr = 0;

    while (true) {
        if (xQueueReceive(uart_queue, (void*)&event, portMAX_DELAY)) {                  // Wait for an event to be received
            memset(rx_buffer, 0, UART_BUF_SIZE);                                        // Clear the receive buffer
            if (event.type == UART_DATA) {                                              // Check if the event is a data event
                uart_read_bytes(dmx_uart_num, rx_buffer, event.size, portMAX_DELAY);    // Read the data from the UART

                if (state == DMXStateRx::Break) {  // Handle a break, reset the state and the current address
                    if (rx_buffer[0] == 0) {
                        state = DMXStateRx::Data;
                        current_rx_addr = 0;
                    }
                }
                if (state == DMXStateRx::Data) {  // Read the data to the current address in the buffer
                    if (xSemaphoreTake(dmx_rx_mutex, portMAX_DELAY)) {
                        for (int i = 0; i < event.size; i++) {
                            if (current_rx_addr < 513) {
                                if (mode == DMXMode::BothKeepRx) {
                                    dmx_data_tx[current_rx_addr] = rx_buffer[i];
                                }
                                dmx_data_rx[current_rx_addr++] = rx_buffer[i];
                            }
                        }
                        xSemaphoreGive(dmx_rx_mutex);
                    }
                }
            }
            else {                                                                          // Either break, idle or other events (such as errors)
                uart_flush(dmx_uart_num);                                                   // Flush the UART buffer
                xQueueReset(uart_queue);                                                    // Reset the queue
                state = (event.type == UART_BREAK) ? DMXStateRx::Break : DMXStateRx::Idle;  // Set the state based on the event type
            }
        }
    }
}

/**
 * Constructs a DMX fixture descriptor with the given number of channels and channel types.
 */
DMXFixtureDescriptor::DMXFixtureDescriptor(uint16_t num_channels, ...) {
    this->num_channels = num_channels;
    channel_types = new DMXChannelType[num_channels];

    va_list args;
    va_start(args, num_channels);
    for (int i = 0; i < num_channels; i++) {
        channel_types[i] = (DMXChannelType)va_arg(args, int);
    }
    va_end(args);
}

/**
 * Destructor for the DMX fixture descriptor. Frees the memory allocated for the channel types.
 */
DMXFixtureDescriptor::~DMXFixtureDescriptor() {
    delete[] channel_types;
}

/**
 * Constructs a DMX fixture with the given descriptor and start address.
 */
DMXFixture::DMXFixture(DMXFixtureDescriptor* descriptor, uint16_t address) {
    this->descriptor = descriptor;
    this->address = address;

    channels = new uint8_t[descriptor->num_channels];
    memset(channels, 0, descriptor->num_channels);
}

/**
 * Sets the value of all channels with the given type to the given value.
 */
void DMXFixture::setChannel(DMXChannelType type, uint8_t value) {
    for (int i = 0; i < descriptor->num_channels; i++) {
        if (descriptor->channel_types[i] == type) {
            channels[i] = value;
        }
    }
}

/**
 * Gets the value of the first channel with the given type.
 */
uint8_t DMXFixture::getChannel(DMXChannelType type) {
    for (int i = 0; i < descriptor->num_channels; i++) {
        if (descriptor->channel_types[i] == type) {
            return channels[i];
        }
    }
    return 0;
}

/**
 * Constructs a DMX universe with the given EasyDMX instance.
 */
DMXUniverse::DMXUniverse(EasyDMX* dmx) {
    this->dmx = dmx;
}

/**
 * Adds a fixture to the DMX universe.
 */
void DMXUniverse::addFixture(DMXFixture* fixture) {
    fixtures.push_back(fixture);
}

/**
 * Removes a fixture from the DMX universe by pointer.
 */
void DMXUniverse::removeFixture(DMXFixture* fixture) {
    fixtures.erase(std::remove(fixtures.begin(), fixtures.end(), fixture), fixtures.end());
}

/**
 * Removes the fixture at the given address from the DMX universe.
 */
void DMXUniverse::removeFixture(uint16_t address) {
    for (int i = 0; i < fixtures.size();) {
        if (fixtures[i]->getAddress() == address) {
            fixtures.erase(fixtures.begin() + i);
        }
        else { // In case of multiple fixtures with the same address this prevents skipping
            i++;
        }
    }
}

/**
 * Updates the DMX universes data by going through all fixtures and updating their channels.
 */
void DMXUniverse::update() {
    for (int i = 0; i < fixtures.size(); i++) {
        for (int j = 0; j < fixtures[i]->descriptor->num_channels; j++) {
            if ((fixtures[i]->getAddress() + j > 512) || (fixtures[i]->getAddress() + j < 1)) {
                break;
            }
            dmx->setChannel(fixtures[i]->getAddress() + j, fixtures[i]->channels[j]);
        }
    }
}

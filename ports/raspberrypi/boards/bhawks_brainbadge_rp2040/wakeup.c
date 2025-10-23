// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 ben-hawks
//
// SPDX-License-Identifier: MIT

#include "wakeup.h"
#include "shared-bindings/busio/I2C.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "py/runtime.h"
#include "hardware/gpio.h"

// PCF85063A RTC I2C address
#define PCF85063A_I2C_ADDR 0x51
#define PCF85063A_CONTROL_2_REG 0x01

// RTC alarm pin (GPIO8)
#define RTC_ALARM_PIN 8

// EN_3V3 pin (GPIO10) - keep board awake on battery
#define EN_3V3_PIN 10

// Button pins
#define SW_DOWN_PIN 11
#define SW_A_PIN 12
#define SW_B_PIN 13
#define SW_C_PIN 14
#define SW_UP_PIN 15

// Store the GPIO state captured at boot time
static uint32_t wakeup_gpio_state = 0;

void wakeup_init(void) {
    // Initialize wakeup subsystem
    // 1. Configure button pins with pull-downs (MUST BE FIRST - before reading)
    // 2. Capture GPIO state for button detection
    // 3. Configure RTC alarm pin and disable RTC clock output (saves power)

    // Configure button pins with pull-downs to match MicroPython behavior
    // Buttons pull HIGH when pressed, so we need pull-downs to read them correctly
    gpio_init(SW_DOWN_PIN);
    gpio_set_dir(SW_DOWN_PIN, GPIO_IN);
    gpio_pull_down(SW_DOWN_PIN);

    gpio_init(SW_A_PIN);
    gpio_set_dir(SW_A_PIN, GPIO_IN);
    gpio_pull_down(SW_A_PIN);

    gpio_init(SW_B_PIN);
    gpio_set_dir(SW_B_PIN, GPIO_IN);
    gpio_pull_down(SW_B_PIN);

    gpio_init(SW_C_PIN);
    gpio_set_dir(SW_C_PIN, GPIO_IN);
    gpio_pull_down(SW_C_PIN);

    gpio_init(SW_UP_PIN);
    gpio_set_dir(SW_UP_PIN, GPIO_IN);
    gpio_pull_down(SW_UP_PIN);

    // Configure GPIO8 (RTC_ALARM) as input with pull-down
    // The RTC alarm is active-HIGH: it pulls HIGH when alarm triggers
    gpio_init(RTC_ALARM_PIN);
    gpio_set_dir(RTC_ALARM_PIN, GPIO_IN);
    gpio_pull_down(RTC_ALARM_PIN);

    // Small delay to allow pull resistors to stabilize
    busy_wait_us(100);

    // Capture GPIO state IMMEDIATELY - read it and again after a short delay
    // This matches the MicroPython implementation which reads twice with a delay
    // to catch brief button presses
    wakeup_gpio_state = gpio_get_all();
    busy_wait_us(5000);  // 5ms delay
    wakeup_gpio_state |= gpio_get_all();

    // Check if RTC alarm pin is HIGH (alarm triggered)
    bool rtc_alarm_active = (wakeup_gpio_state & (1 << RTC_ALARM_PIN)) != 0;

    // Only modify RTC CONTROL_2 register if no alarm is pending
    // This preserves the AF (Alarm Flag) for Python code to read
    // If alarm is active, Python code will disable clock output after clearing AF
    if (!rtc_alarm_active) {
        // Initialize I2C for RTC communication
        busio_i2c_obj_t i2c_obj;
        common_hal_busio_i2c_construct(&i2c_obj, &pin_GPIO5, &pin_GPIO4, 100000, 0);
        common_hal_busio_i2c_never_reset(&i2c_obj);

        // Turn off CLOCK_OUT in CONTROL_2 (0x01) register
        // IMPORTANT: We must preserve the AIE and AF flags (bits 6-7)
        // by doing a read-modify-write operation
        uint8_t control_2_reg = PCF85063A_CONTROL_2_REG;
        uint8_t control_2_value = 0;

        // Lock I2C bus for our transaction
        if (common_hal_busio_i2c_try_lock(&i2c_obj)) {
            // Read current CONTROL_2 value
            common_hal_busio_i2c_write_read(&i2c_obj, PCF85063A_I2C_ADDR,
                &control_2_reg, 1,
                &control_2_value, 1);

            // Modify: Set bits 0-2 to 0b111 to disable clock output
            // Preserve all other bits (including AIE and AF)
            control_2_value = (control_2_value & 0xF8) | 0b00000111;

            // Write back the modified value
            uint8_t buffer[2] = {PCF85063A_CONTROL_2_REG, control_2_value};
            common_hal_busio_i2c_write(&i2c_obj, PCF85063A_I2C_ADDR, buffer, 2);

            // Unlock I2C bus
            common_hal_busio_i2c_unlock(&i2c_obj);
        }

        // Deinitialize I2C - it will be reinitialized by user code if needed
        common_hal_busio_i2c_deinit(&i2c_obj);
    }
    // else: Skip RTC I2C access to preserve AF flag - Python will handle it
}

uint32_t wakeup_get_gpio_state(void) {
    return wakeup_gpio_state;
}

void wakeup_reset_gpio_state(void) {
    wakeup_gpio_state = 0;
}

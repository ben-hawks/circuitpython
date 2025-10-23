// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 ben-hawks
//
// SPDX-License-Identifier: MIT

#ifndef WAKEUP_H
#define WAKEUP_H

#include <stdint.h>

// Initialize wakeup subsystem - called during board_init()
void wakeup_init(void);

// Get the captured GPIO state from boot time
uint32_t wakeup_get_gpio_state(void);

// Reset the GPIO state (for testing)
void wakeup_reset_gpio_state(void);

#endif // WAKEUP_H

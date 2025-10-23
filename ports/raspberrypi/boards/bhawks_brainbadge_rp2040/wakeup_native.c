// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 ben-hawks
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"
#include "wakeup.h"

// Get the captured GPIO state from boot time
static mp_obj_t wakeup_native_get_gpio_state(void) {
    return MP_OBJ_NEW_SMALL_INT(wakeup_get_gpio_state());
}
static MP_DEFINE_CONST_FUN_OBJ_0(wakeup_native_get_gpio_state_obj, wakeup_native_get_gpio_state);

// Reset the GPIO state (for testing)
static mp_obj_t wakeup_native_reset_gpio_state(void) {
    wakeup_reset_gpio_state();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(wakeup_native_reset_gpio_state_obj, wakeup_native_reset_gpio_state);

// Module definition
static const mp_rom_map_elem_t wakeup_native_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__wakeup_native) },
    { MP_ROM_QSTR(MP_QSTR_get_gpio_state), MP_ROM_PTR(&wakeup_native_get_gpio_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_gpio_state), MP_ROM_PTR(&wakeup_native_reset_gpio_state_obj) },
};
static MP_DEFINE_CONST_DICT(wakeup_native_module_globals, wakeup_native_module_globals_table);

const mp_obj_module_t wakeup_native_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&wakeup_native_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__wakeup_native, wakeup_native_module);

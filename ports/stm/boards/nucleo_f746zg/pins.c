#include "shared-bindings/board/__init__.h"

STATIC const mp_rom_map_elem_t board_module_globals_table[] = {
    CIRCUITPYTHON_BOARD_DICT_STANDARD_ITEMS

    { MP_ROM_QSTR(MP_QSTR_A0), MP_ROM_PTR(&pin_PA03) },
    { MP_ROM_QSTR(MP_QSTR_A1), MP_ROM_PTR(&pin_PC00) },
    { MP_ROM_QSTR(MP_QSTR_A2), MP_ROM_PTR(&pin_PC03) },
    { MP_ROM_QSTR(MP_QSTR_A3), MP_ROM_PTR(&pin_PF03) },
    { MP_ROM_QSTR(MP_QSTR_A4), MP_ROM_PTR(&pin_PF05) },
    { MP_ROM_QSTR(MP_QSTR_A5), MP_ROM_PTR(&pin_PF10) },
    { MP_ROM_QSTR(MP_QSTR_D0), MP_ROM_PTR(&pin_PG09) },
    { MP_ROM_QSTR(MP_QSTR_D1), MP_ROM_PTR(&pin_PG14) },
    { MP_ROM_QSTR(MP_QSTR_D2), MP_ROM_PTR(&pin_PF15) },
    { MP_ROM_QSTR(MP_QSTR_D3), MP_ROM_PTR(&pin_PE13) },
    { MP_ROM_QSTR(MP_QSTR_D4), MP_ROM_PTR(&pin_PF14) },
    { MP_ROM_QSTR(MP_QSTR_D5), MP_ROM_PTR(&pin_PE11) },
    { MP_ROM_QSTR(MP_QSTR_D6), MP_ROM_PTR(&pin_PE09) },
    { MP_ROM_QSTR(MP_QSTR_D7), MP_ROM_PTR(&pin_PF13) },
    { MP_ROM_QSTR(MP_QSTR_D8), MP_ROM_PTR(&pin_PF12) },
    { MP_ROM_QSTR(MP_QSTR_D9), MP_ROM_PTR(&pin_PD15) },
    { MP_ROM_QSTR(MP_QSTR_D10), MP_ROM_PTR(&pin_PD14) },
    { MP_ROM_QSTR(MP_QSTR_D11), MP_ROM_PTR(&pin_PA07) },
    { MP_ROM_QSTR(MP_QSTR_D12), MP_ROM_PTR(&pin_PA06) },
    { MP_ROM_QSTR(MP_QSTR_D13), MP_ROM_PTR(&pin_PA05) },
    { MP_ROM_QSTR(MP_QSTR_D14), MP_ROM_PTR(&pin_PB09) },
    { MP_ROM_QSTR(MP_QSTR_D15), MP_ROM_PTR(&pin_PB08) },
    { MP_ROM_QSTR(MP_QSTR_D16), MP_ROM_PTR(&pin_PC06) },
    { MP_ROM_QSTR(MP_QSTR_D17), MP_ROM_PTR(&pin_PB15) },
    { MP_ROM_QSTR(MP_QSTR_D18), MP_ROM_PTR(&pin_PB13) },
    { MP_ROM_QSTR(MP_QSTR_D19), MP_ROM_PTR(&pin_PB12) },
    { MP_ROM_QSTR(MP_QSTR_D20), MP_ROM_PTR(&pin_PA15) },
    { MP_ROM_QSTR(MP_QSTR_D21), MP_ROM_PTR(&pin_PC07) },
    { MP_ROM_QSTR(MP_QSTR_D22), MP_ROM_PTR(&pin_PB05) },
    { MP_ROM_QSTR(MP_QSTR_D23), MP_ROM_PTR(&pin_PB03) },
    { MP_ROM_QSTR(MP_QSTR_D24), MP_ROM_PTR(&pin_PA04) },
    { MP_ROM_QSTR(MP_QSTR_D25), MP_ROM_PTR(&pin_PB04) },
    { MP_ROM_QSTR(MP_QSTR_LED1), MP_ROM_PTR(&pin_PB00) },
    { MP_ROM_QSTR(MP_QSTR_LED2), MP_ROM_PTR(&pin_PB07) },
    { MP_ROM_QSTR(MP_QSTR_LED3), MP_ROM_PTR(&pin_PB14) },
    { MP_ROM_QSTR(MP_QSTR_SW), MP_ROM_PTR(&pin_PC13) },
    { MP_ROM_QSTR(MP_QSTR_TP1), MP_ROM_PTR(&pin_PH02) },
    { MP_ROM_QSTR(MP_QSTR_TP2), MP_ROM_PTR(&pin_PI08) },
    { MP_ROM_QSTR(MP_QSTR_TP3), MP_ROM_PTR(&pin_PH15) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_INT), MP_ROM_PTR(&pin_PD06) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SDA), MP_ROM_PTR(&pin_PH08) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SCL), MP_ROM_PTR(&pin_PH07) },
    { MP_ROM_QSTR(MP_QSTR_EXT_SDA), MP_ROM_PTR(&pin_PB09) },
    { MP_ROM_QSTR(MP_QSTR_EXT_SCL), MP_ROM_PTR(&pin_PB08) },
    { MP_ROM_QSTR(MP_QSTR_EXT_RST), MP_ROM_PTR(&pin_PG03) },
    { MP_ROM_QSTR(MP_QSTR_SD_SW), MP_ROM_PTR(&pin_PC13) },
    { MP_ROM_QSTR(MP_QSTR_LCD_BL_CTRL), MP_ROM_PTR(&pin_PK03) },
    { MP_ROM_QSTR(MP_QSTR_LCD_INT), MP_ROM_PTR(&pin_PI13) },
    { MP_ROM_QSTR(MP_QSTR_LCD_SDA), MP_ROM_PTR(&pin_PH08) },
    { MP_ROM_QSTR(MP_QSTR_LCD_SCL), MP_ROM_PTR(&pin_PH07) },
    { MP_ROM_QSTR(MP_QSTR_OTG_FS_POWER), MP_ROM_PTR(&pin_PD05) },
    { MP_ROM_QSTR(MP_QSTR_OTG_FS_OVER_CURRENT), MP_ROM_PTR(&pin_PD04) },
    { MP_ROM_QSTR(MP_QSTR_OTG_HS_OVER_CURRENT), MP_ROM_PTR(&pin_PE03) },
    { MP_ROM_QSTR(MP_QSTR_USB_VBUS), MP_ROM_PTR(&pin_PA09) },
    { MP_ROM_QSTR(MP_QSTR_USB_ID), MP_ROM_PTR(&pin_PA10) },
    { MP_ROM_QSTR(MP_QSTR_USB_DM), MP_ROM_PTR(&pin_PA11) },
    { MP_ROM_QSTR(MP_QSTR_USB_DP), MP_ROM_PTR(&pin_PA12) },
// As we use these for the debug_console, we won't enable them here.
// { MP_ROM_QSTR(MP_QSTR_VCP_TX), MP_ROM_PTR(&pin_PD08) },
// { MP_ROM_QSTR(MP_QSTR_VCP_RX), MP_ROM_PTR(&pin_PD09) },
    { MP_ROM_QSTR(MP_QSTR_UART2_TX), MP_ROM_PTR(&pin_PD05) },
    { MP_ROM_QSTR(MP_QSTR_UART2_RX), MP_ROM_PTR(&pin_PD06) },
    { MP_ROM_QSTR(MP_QSTR_UART2_RTS), MP_ROM_PTR(&pin_PD04) },
    { MP_ROM_QSTR(MP_QSTR_UART2_CTS), MP_ROM_PTR(&pin_PD03) },
    { MP_ROM_QSTR(MP_QSTR_UART6_TX), MP_ROM_PTR(&pin_PG14) },
    { MP_ROM_QSTR(MP_QSTR_UART6_RX), MP_ROM_PTR(&pin_PG09) },
    { MP_ROM_QSTR(MP_QSTR_SPI_B_NSS), MP_ROM_PTR(&pin_PA04) },
    { MP_ROM_QSTR(MP_QSTR_SPI_B_SCK), MP_ROM_PTR(&pin_PB03) },
    { MP_ROM_QSTR(MP_QSTR_SPI_B_MOSI), MP_ROM_PTR(&pin_PB05) },
};
MP_DEFINE_CONST_DICT(board_module_globals, board_module_globals_table);
#include "py/objtuple.h"
#include "shared-bindings/board/__init__.h"

STATIC const mp_rom_obj_tuple_t camera_data_tuple = {
    {&mp_type_tuple},
    8,
    {
        MP_ROM_PTR(&pin_GPIO46),
        MP_ROM_PTR(&pin_GPIO45),
        MP_ROM_PTR(&pin_GPIO41),
        MP_ROM_PTR(&pin_GPIO42),
        MP_ROM_PTR(&pin_GPIO39),
        MP_ROM_PTR(&pin_GPIO40),
        MP_ROM_PTR(&pin_GPIO21),
        MP_ROM_PTR(&pin_GPIO38),
    }
};

STATIC const mp_rom_map_elem_t board_module_globals_table[] = {
    CIRCUITPYTHON_BOARD_DICT_STANDARD_ITEMS

    { MP_ROM_QSTR(MP_QSTR_IO0), MP_ROM_PTR(&pin_GPIO0) },
    { MP_ROM_QSTR(MP_QSTR_IO1), MP_ROM_PTR(&pin_GPIO1) },
    { MP_ROM_QSTR(MP_QSTR_IO2), MP_ROM_PTR(&pin_GPIO2) },
    { MP_ROM_QSTR(MP_QSTR_IO3), MP_ROM_PTR(&pin_GPIO3) },
    { MP_ROM_QSTR(MP_QSTR_IO4), MP_ROM_PTR(&pin_GPIO4) },
    { MP_ROM_QSTR(MP_QSTR_IO5), MP_ROM_PTR(&pin_GPIO5) },
    { MP_ROM_QSTR(MP_QSTR_IO6), MP_ROM_PTR(&pin_GPIO6) },
    { MP_ROM_QSTR(MP_QSTR_IO7), MP_ROM_PTR(&pin_GPIO7) },
    { MP_ROM_QSTR(MP_QSTR_IO8), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_IO9), MP_ROM_PTR(&pin_GPIO9) },
    { MP_ROM_QSTR(MP_QSTR_IO10), MP_ROM_PTR(&pin_GPIO10) },
    { MP_ROM_QSTR(MP_QSTR_IO11), MP_ROM_PTR(&pin_GPIO11) },
    { MP_ROM_QSTR(MP_QSTR_IO12), MP_ROM_PTR(&pin_GPIO12) },
    { MP_ROM_QSTR(MP_QSTR_IO13), MP_ROM_PTR(&pin_GPIO13) },
    { MP_ROM_QSTR(MP_QSTR_IO14), MP_ROM_PTR(&pin_GPIO14) },
    { MP_ROM_QSTR(MP_QSTR_IO15), MP_ROM_PTR(&pin_GPIO15) },
    { MP_ROM_QSTR(MP_QSTR_IO16), MP_ROM_PTR(&pin_GPIO16) },
    { MP_ROM_QSTR(MP_QSTR_IO17), MP_ROM_PTR(&pin_GPIO17) },


    { MP_ROM_QSTR(MP_QSTR_IO18), MP_ROM_PTR(&pin_GPIO18) },
    { MP_ROM_QSTR(MP_QSTR_IO19), MP_ROM_PTR(&pin_GPIO19) },
    { MP_ROM_QSTR(MP_QSTR_IO20), MP_ROM_PTR(&pin_GPIO20) },
    { MP_ROM_QSTR(MP_QSTR_IO21), MP_ROM_PTR(&pin_GPIO21) },
    { MP_ROM_QSTR(MP_QSTR_IO26), MP_ROM_PTR(&pin_GPIO26) },
    { MP_ROM_QSTR(MP_QSTR_IO33), MP_ROM_PTR(&pin_GPIO33) },
    { MP_ROM_QSTR(MP_QSTR_IO34), MP_ROM_PTR(&pin_GPIO34) },
    { MP_ROM_QSTR(MP_QSTR_IO35), MP_ROM_PTR(&pin_GPIO35) },
    { MP_ROM_QSTR(MP_QSTR_IO36), MP_ROM_PTR(&pin_GPIO36) },
    { MP_ROM_QSTR(MP_QSTR_IO37), MP_ROM_PTR(&pin_GPIO37) },
    { MP_ROM_QSTR(MP_QSTR_IO38), MP_ROM_PTR(&pin_GPIO38) },
    { MP_ROM_QSTR(MP_QSTR_IO39), MP_ROM_PTR(&pin_GPIO39) },
    { MP_ROM_QSTR(MP_QSTR_IO40), MP_ROM_PTR(&pin_GPIO40) },
    { MP_ROM_QSTR(MP_QSTR_IO41), MP_ROM_PTR(&pin_GPIO41) },
    { MP_ROM_QSTR(MP_QSTR_IO42), MP_ROM_PTR(&pin_GPIO42) },
    { MP_ROM_QSTR(MP_QSTR_TX), MP_ROM_PTR(&pin_GPIO43) },
    { MP_ROM_QSTR(MP_QSTR_IO43), MP_ROM_PTR(&pin_GPIO43) },
    { MP_ROM_QSTR(MP_QSTR_RX), MP_ROM_PTR(&pin_GPIO44) },
    { MP_ROM_QSTR(MP_QSTR_IO44), MP_ROM_PTR(&pin_GPIO44) },
    { MP_ROM_QSTR(MP_QSTR_IO45), MP_ROM_PTR(&pin_GPIO45) },
    { MP_ROM_QSTR(MP_QSTR_IO46), MP_ROM_PTR(&pin_GPIO46) },


    { MP_ROM_QSTR(MP_QSTR_NEOPIXEL), MP_ROM_PTR(&pin_GPIO45) },

    { MP_ROM_QSTR(MP_QSTR_CAMERA_XCLK), MP_ROM_PTR(&pin_GPIO1) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_PCLK), MP_ROM_PTR(&pin_GPIO33) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_HREF), MP_ROM_PTR(&pin_GPIO3) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_VSYNC), MP_ROM_PTR(&pin_GPIO2) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_SIOD), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_SIOC), MP_ROM_PTR(&pin_GPIO7) },


    { MP_ROM_QSTR(MP_QSTR_CAMERA_DATA), MP_ROM_PTR(&camera_data_tuple) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D2), MP_ROM_PTR(&pin_GPIO46) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D3), MP_ROM_PTR(&pin_GPIO45) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D4), MP_ROM_PTR(&pin_GPIO41) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D5), MP_ROM_PTR(&pin_GPIO42) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D6), MP_ROM_PTR(&pin_GPIO39) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D7), MP_ROM_PTR(&pin_GPIO40) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D8), MP_ROM_PTR(&pin_GPIO21) },
    { MP_ROM_QSTR(MP_QSTR_CAMERA_D9), MP_ROM_PTR(&pin_GPIO38) },


    { MP_ROM_QSTR(MP_QSTR_TOUCH1), MP_ROM_PTR(&pin_GPIO1) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH2), MP_ROM_PTR(&pin_GPIO2) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH3), MP_ROM_PTR(&pin_GPIO3) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH4), MP_ROM_PTR(&pin_GPIO4) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH5), MP_ROM_PTR(&pin_GPIO5) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH6), MP_ROM_PTR(&pin_GPIO6) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH7), MP_ROM_PTR(&pin_GPIO7) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH8), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH9), MP_ROM_PTR(&pin_GPIO9) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH10), MP_ROM_PTR(&pin_GPIO10) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH11), MP_ROM_PTR(&pin_GPIO11) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH12), MP_ROM_PTR(&pin_GPIO12) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH13), MP_ROM_PTR(&pin_GPIO13) },
    { MP_ROM_QSTR(MP_QSTR_TOUCH14), MP_ROM_PTR(&pin_GPIO14) },

    // LED FPC
    { MP_ROM_QSTR(MP_QSTR_SCL), MP_ROM_PTR(&pin_GPIO7) },
    { MP_ROM_QSTR(MP_QSTR_SDA), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_BT_ARRAY_ADC), MP_ROM_PTR(&pin_GPIO6) },

    // 3.2 inch LCD FPC
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_MISO), MP_ROM_PTR(&pin_GPIO42) },
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_MOSI), MP_ROM_PTR(&pin_GPIO40) },
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_SCK), MP_ROM_PTR(&pin_GPIO38) },
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_CS), MP_ROM_PTR(&pin_GPIO33) },
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_IRQ), MP_ROM_PTR(&pin_GPIO2) },
    { MP_ROM_QSTR(MP_QSTR_LCD_TP_BUSY), MP_ROM_PTR(&pin_GPIO4) },
    { MP_ROM_QSTR(MP_QSTR_LCD_BL_CTR), MP_ROM_PTR(&pin_GPIO6) },
    { MP_ROM_QSTR(MP_QSTR_LCD_MISO), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_LCD_MOSI), MP_ROM_PTR(&pin_GPIO9) },
    { MP_ROM_QSTR(MP_QSTR_LCD_CS), MP_ROM_PTR(&pin_GPIO11) },
    { MP_ROM_QSTR(MP_QSTR_LCD_D_C), MP_ROM_PTR(&pin_GPIO13) },
    { MP_ROM_QSTR(MP_QSTR_LCD_CLK), MP_ROM_PTR(&pin_GPIO15) },
    { MP_ROM_QSTR(MP_QSTR_LCD_RST), MP_ROM_PTR(&pin_GPIO16) },

    // Audio
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SPI_MISO), MP_ROM_PTR(&pin_GPIO42) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SPI_MOSI), MP_ROM_PTR(&pin_GPIO40) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SPI_SCK), MP_ROM_PTR(&pin_GPIO38) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SPI_CS), MP_ROM_PTR(&pin_GPIO33) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_BT_ADC), MP_ROM_PTR(&pin_GPIO6) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SCL), MP_ROM_PTR(&pin_GPIO7) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_SDA), MP_ROM_PTR(&pin_GPIO8) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S0_MCLK), MP_ROM_PTR(&pin_GPIO41) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S0_BCLK), MP_ROM_PTR(&pin_GPIO39) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S0_LRCK), MP_ROM_PTR(&pin_GPIO21) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S0_SDI), MP_ROM_PTR(&pin_GPIO1) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S0_SDO), MP_ROM_PTR(&pin_GPIO3) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_RST), MP_ROM_PTR(&pin_GPIO5) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_WAKE_INT), MP_ROM_PTR(&pin_GPIO34) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S1_MCLK), MP_ROM_PTR(&pin_GPIO35) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_PA_CTRL), MP_ROM_PTR(&pin_GPIO10) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S1_SDI), MP_ROM_PTR(&pin_GPIO46) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S1_SDO), MP_ROM_PTR(&pin_GPIO12) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S1_LRCK_DAC1), MP_ROM_PTR(&pin_GPIO17) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_I2S1_BCLK_DAC2), MP_ROM_PTR(&pin_GPIO18) },

};
MP_DEFINE_CONST_DICT(board_module_globals, board_module_globals_table);
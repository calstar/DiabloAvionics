#pragma once

//define actuator board pins to be imported where needed
namespace actuator_board_pins {
    struct Layout {
        // ETH_MOSI, ETH_MISO, ETH_SCLK, ETH_CS, ETH_INT, ETH_RST
        int eth_mosi;
        int eth_miso;
        int eth_sclk;
        int eth_cs;
        int eth_int;
        int eth_rst;

        // ADC_MOSI, ADC_MISO, ADC_SCLK
        int adc_mosi;
        int adc_miso;
        int adc_sclk;

        // ADC_CS_1, ADC_RESET_1, ADC_START_1, ADC_DRDY_1
        int adc_cs_1;
        int adc_reset_1;
        int adc_start_1;
        int adc_drdy_1;

        // ADC_CS_2, ADC_RESET_2, ADC_START_2, ADC_DRDY_2
        int adc_cs_2;
        int adc_reset_2;
        int adc_start_2;
        int adc_drdy_2;

        // LED
        int led;
    };

    const Layout Actuator_Board = {
        // ETH_MOSI, ETH_MISO, ETH_SCLK, ETH_CS, ETH_INT, ETH_RST
        11, 12, 13, 10, 2, 9,

        // ADC_MOSI, ADC_MISO, ADC_SCLK
        7, 6, 5,

        // ADC_CS_1, ADC_RESET_1, ADC_START_1, ADC_DRDY_1
        4, 3, 8, A0,

        // ADC_CS_2, ADC_RESET_2, ADC_START_2, ADC_DRDY_2
        A1, A2, A3, A4,

        // LED
        LED_BUILTIN
    };
}

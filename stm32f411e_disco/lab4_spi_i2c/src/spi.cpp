/*================================================================================
 *
 *    Filename : spi.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm interfaces L3GD20 gyroscope sensor via SPI bus. Measu-
 *               rements from the device are printed to the serial debug port
 *               with a frequency controlled by FREQUENCY constant.
 *
 *               With the same frequency brightness of the LED3-LED6 diodes is
 *               updated so that duty cycle of the signal controlling a LED
 *               is proportional to the value of the angular rate toward
 *               direction pointing by the LED itself. If measurement is equal
 *               to the actaul range of the sensor, duty cycle is equal 100%,
 *               and when measurement is close to zero, duty cycle also is set
 *               to 0%. 
 *
 *===============================================================================*/

#include <config/conf.h>                // ISIX       : Base configuration
#include <isix.h>                       // ISIX       : System modules
#include <isix/types.h>                 // ISIX       : Infinite timeout macro
#include <foundation/sys/dbglog.h>      // FOUNDATION : Logging module
#include <foundation/sys/tiny_printf.h> // FOUNDATION : printf
#include <periph/gpio/gpio.hpp>         // PERIPH     : GPIO framework
#include <stm32_ll_bus.h>               // LL         : clock enable bit masks
#include <string.h>                     // STD        : C-type strings
#include <math.h>                       // STD        : abs()

#include "L3GD20.h"                     // L3GD20 : class
#include "L3GD20_reg.h"                 // L3GD20 : gyroscope's registers


namespace {

    // Main loop's frequency [Hz]
    constexpr uint32_t FREQUENCY = 100;

    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;


    /**
     * Updates duty cycle of the PWm driving LED3-LED6
     * that indicates direction and speed of the sensor's
     * angular movement
     *
     * @param x_measurement : measurement in the X axis
     * @param y_measurement : measurement in the Y axis
     * @param range : actual range of the sensor
     */
    void update_led(float x_measurement, float y_measurement, float range){

        uint16_t x_magnitude = uint16_t(abs(x_measurement) * 500 / range); 
        uint16_t y_magnitude = uint16_t(abs(y_measurement) * 500 / range);

        /**
         * LED3 -> CCR2
         * LED5 -> CCR3
         * LED6 -> CCR4
         * LED4 -> CCR1
         */
        if(x_measurement > 0){
            TIM4->CCR1 = x_magnitude;
            TIM4->CCR3 = 0;
        } else{
            TIM4->CCR1 = 0;
            TIM4->CCR3 = x_magnitude;
        }
        if(y_measurement > 0){
            TIM4->CCR4 = y_magnitude;
            TIM4->CCR2 = 0;
        } else{
            TIM4->CCR4 = 0;
            TIM4->CCR2 = y_magnitude;
        }

    }
}

/**
 * Thread executes periodic reads of L3GD20 MEMS
 * gyroscope via SPI bud and writes read values 
 * to the debug log (USART2).
 */
void gyro(void*){

#ifdef ISIX
    // Create and initialize SPI master
    L3GD20::SPI_InitType_ISIX init_struct{
        .speed    = uint32_t(10E6),
        .polarity = periph::option::polarity::low,
        .phase    = periph::option::phase::_1_edge,
        .dwidth   = 8,
        .bitorder = periph::option::bitorder::msb,
        .timeout  = ISIX_TIME_INFINITE
    };
    L3GD20 gyroscope(
        "spi1", &init_struct,
        LL_AHB1_GRP1_PERIPH_GPIOA |
        LL_AHB1_GRP1_PERIPH_GPIOE
    );
#else
    // LL-specific constructor
#endif

    // Check whether L3GD20 is present on the bus
    if(!gyroscope.isConected())
        while(1);

    // Configure gyroscope
    const uint8_t config_output[5] = {0x0F, 0x00, 0x08, 0x30, 0x00};
    if(gyroscope.writeControlRegisters(config_output, L3GD20_CTRL_REG1, 5) < 5)
        while(1);

    // Perform scanning
    while (true){

        // Get measurements
        float measurements[3] = {};
        if(gyroscope.readMeasurementRegisters(measurements, L3GD20_OUT_X_L, 3) < 3)
            while(1);

        // Print measurements
        char output[300] {};
        sprintf(output,
            "\r\n"
            "=============================================================================\r\n"
            "| Angular rate [x, y, z]: [%9.4f deg/s,%9.4f deg/s,%9.4f deg/s] |\r\n"
            "=============================================================================\r\n",
            measurements[0],
            measurements[1],
            measurements[2]
        );
        fnd::tiny_printf(output);

        // Update LED brightness
        update_led(measurements[0], measurements[1], gyroscope.getRange());

        // Wait until the next iteration
        isix::wait( FREQUENCY );
    }
}

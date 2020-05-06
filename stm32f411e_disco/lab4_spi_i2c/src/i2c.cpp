/*================================================================================
 *
 *    Filename : i2c.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm interfaces LSM303 acclerometer / magnetometer sensor
 *               via I2C bus. Measurements from the device are printed to the
 *               serial debug port with a frequency controlled by FREQUENCY 
 *               constant.
 *
 *               With the same frequency brightness of the LED3-LED6 diodes is
 *               updated so that duty cycle of the signal controlling a LED
 *               is proportional to the value of the acceleration toward
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
#include <periph/clock/clocks.hpp>      // PERIPH     : Clocks enabling
#include <stm32_ll_bus.h>               // LL         : clock enable bit masks
#include <string.h>                     // STD        : C-type strings
#include <math.h>                       // STD        : abs()

#include "LSM303.h"                     // LSM303 : class
#include "LSM303_reg.h"                 // LSM303 : sensor's registers


namespace {

    // Main loop's frequency [Hz]
    constexpr uint32_t FREQUENCY = 100;

    /**
     * Minimal difference between two subsequent measurements
     * [in g/Gs] that triggers serial monitor to print actual
     * measurement.
     */
    constexpr float ACC_THRESHOLD = 0.1;
    constexpr float MAG_THRESHOLD = 0.001;

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
            TIM4->CCR4 = x_magnitude;
            TIM4->CCR2 = 0;
        } else{
            TIM4->CCR4 = 0;
            TIM4->CCR2 = x_magnitude;
        }        
        if(y_measurement > 0){
            TIM4->CCR3 = y_magnitude;
            TIM4->CCR1 = 0;
        } else{
            TIM4->CCR3 = 0;
            TIM4->CCR1 = y_magnitude;
        }

    }
}




/**
 * Thread executes periodic reads of L3GD20 MEMS
 * gyroscope via SPI bud and writes read values 
 * to the debug log (USART2).
 */
void gyro(void*){

    int ret {};

#ifdef ISIX
    // Create and initialize SPI master
    LSM303::I2C_InitType_ISIX init_struct{
        .speed    = uint32_t(400'000),
        .timeout  = 1000
    };
    LSM303 sensor(
        "i2c1", &init_struct,
        LL_AHB1_GRP1_PERIPH_GPIOB
    );
#else
    // LL-specific constructor
#endif

    // Configure gyroscope
    uint8_t config = 0x27; 
    ret = sensor.writeControlRegisters(LSM303::ACC, &config, LSM303_CTRL_REG1_A, 1);
    if(ret < 0)
        while(1);

    config = 0x00;
    ret = sensor.writeControlRegisters(LSM303::MAG, &config, LSM303_MR_REG_M, 1);
    if(ret < 0)
        while(1);

    // Perform scanning
    while (true){

        static float acc_measurements_prev[3] = {};
        static float mag_measurements_prev[3] = {};
                float acc_measurements[3]      = {};
                float mag_measurements[3]      = {};

        // Get accelerometer's measurements
        ret = sensor.readMeasurementRegisters(LSM303::ACC, acc_measurements, LSM303_OUT_X_L_A, 3);
        if(ret < 0)
            while(1);
        
        // Get magnetometer's measurements
        ret = sensor.readMeasurementRegisters(LSM303::MAG, mag_measurements, LSM303_OUT_X_H_M, 3);
        if(ret < 0)
            while(1);

        // Check if measurements has changed
        bool acc_changed = 
            abs(acc_measurements[0] - acc_measurements_prev[0]) > ACC_THRESHOLD ||
            abs(acc_measurements[1] - acc_measurements_prev[1]) > ACC_THRESHOLD ||
            abs(acc_measurements[2] - acc_measurements_prev[2]) > ACC_THRESHOLD;

        bool mag_changed = 
            abs(mag_measurements[0] - mag_measurements_prev[0]) > MAG_THRESHOLD ||
            abs(mag_measurements[1] - mag_measurements_prev[1]) > MAG_THRESHOLD ||
            abs(mag_measurements[2] - mag_measurements_prev[2]) > MAG_THRESHOLD;

        // If acceleration measurement has changed, print measurement
        if(acc_changed)
        {
            // Print measurements
            char acc_output[200] {};
            sprintf(
                acc_output,
                "\r\n"
                "==================================================================\r\n"
                "| Accelerometer [x, y, z]: [%8.4f  g,%8.4f  g,%8.4f  g] |",
                acc_measurements[0],
                acc_measurements[1],
                acc_measurements[2]
            );
            fnd::tiny_printf(acc_output);
        }

        // If magetic field measurement has changed, print measurement
        if(mag_changed)
        {
            // Print measurements
            char mag_output[200] {};
            sprintf(
                mag_output,
                "\r\n"
                "==================================================================\r\n"
                "| Magnetometer  [x, y, z]: [%8.4f Gs,%8.4f Gs,%8.4f Gs] |",
                mag_measurements[0],
                mag_measurements[1],
                mag_measurements[2]
            );
            fnd::tiny_printf(mag_output);
        }

        if(acc_changed || mag_changed)
            fnd::tiny_printf("\r\n==================================================================\r\n");

        // Save actual measurements for the next iteration
        for(int i = 0; i < 3; ++i){
            mag_measurements_prev[i] = mag_measurements[i];
            acc_measurements_prev[i] = acc_measurements[i];
        }

        // Update LED brightness
        update_led(acc_measurements[0], acc_measurements[1], sensor.getAccRange());

        // Wait for the next iteration            
        isix::wait( FREQUENCY );
    }
}

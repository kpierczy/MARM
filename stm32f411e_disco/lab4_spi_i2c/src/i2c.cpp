#include <config/conf.h>                        // (ISIX) : Base configuration
#include <isix.h>                               //  ISIX  : System modules
#include <isix/types.h>                         //  ISIX  : Infinite timeout macro
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <foundation/sys/tiny_printf.h>         // FOUNDATION : printf
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <periph/clock/clocks.hpp>              // PERIPH : Clocks enabling
#include <string.h>                             // STD : C-type strings

#include "LSM303.h"                             // LSM303 : class
#include "LSM303_reg.h"                         // LSM303 : sensor's registers


namespace {
     
    /**
     * Thread executes periodic reads of L3GD20 MEMS
     * gyroscope via SPI bud and writes read values 
     * to the debug log (USART2).
     */
    void gyro(void*){

#ifdef ISIX
        // Create and initialize SPI master
        LSM303::I2C_InitType_ISIX init_struct{
            .speed    = uint32_t(100000),
            .timeout  = ISIX_TIME_INFINITE
        };
        LSM303 sensor(
            "spi1", &init_struct,
            LL_AHB1_GRP1_PERIPH_GPIOB
        );
#else
        // LL-specific constructor
#endif

        // Configure gyroscope
        uint8_t config = 0x27; 
        if(sensor.writeControlRegisters(LSM303::ACC, &config, LSM303_CTRL_REG1_A, 1) < 1)
            while(1);
                config = 0x00; 
        if(sensor.writeControlRegisters(LSM303::MAG, &config, LSM303_MR_REG_M, 1) < 1)
            while(1);

        // Perform scanning
        while (true){

            // Get accelerometer's measurements
            float measurements[3] = {};
            if(sensor.readMeasurementRegisters(LSM303::ACC, measurements, LSM303_OUT_X_L_A, 3) < 3)
                while(1);

            // Print measurements
            char acc_output[100] {};
            sprintf(acc_output, "[x, y, z]: [%9.4fg,%9.4fg,%9.4fg]\r\n", measurements[0], measurements[1], measurements[2]);
            fnd::tiny_printf(acc_output);
            isix::wait( 500 );

            // Get magnetometer's measurements
            if(sensor.readMeasurementRegisters(LSM303::MAG, measurements, LSM303_OUT_X_H_M, 3) < 3)
                while(1);

            // Print measurements
            char mag_output[100] {};
            sprintf(mag_output, "[x, y, z]: [%9.4fGs,%9.4fGs,%9.4fGs]\r\n\r\n", measurements[0], measurements[1], measurements[2]);
            fnd::tiny_printf(mag_output);
            isix::wait( 500 );
        }
    }
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Enable GPIOA for USART2 alternate function
    periph::clock::device_enable(
        periph::dt::clk_periph{
            .xbus = periph::dt::bus::ahb1,
            .bit  = RCC_AHB1ENR_GPIOAEN_Pos
        }
    );

    // Initialize debug logger
	dblog_init_locked(
		[](int ch, void*) {
			return periph::drivers::uart_early::putc(ch);
		},
		nullptr,
		[]() {
			m_ulock_sem.wait(ISIX_TIME_INFINITE);
		},
		[]() {
			m_ulock_sem.signal();
		},
		periph::drivers::uart_early::open,
		"serial0", 115200
	);

    // Create threads
	isix::task_create(gyro, nullptr, 1536, isix::get_min_priority() );

    // Send welcome message to the log (UART)
    dbprintf("");
    dbprintf("");
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
    dbprintf("");
    dbprintf("");

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}
#include <config/conf.h>                        // (ISIX) : Base configuration
#include <isix.h>                               //  ISIX  : System modules
#include <isix/types.h>                         //  ISIX  : Infinite timeout macro
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <foundation/sys/tiny_printf.h>         // FOUNDATION : printf
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <periph/clock/clocks.hpp>              // PERIPH : Clocks enabling
#include <string.h>                             // STD : C-type strings

#include "L3GD20.h"                             // L3GD20 : class
#include "L3GD20_reg.h"                         // L3GD20 : gyroscope's registers


namespace {
     
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
            char output[100] {};
            sprintf(output, "[x, y, z]: [%9.4f*/s,%9.4f*/s,%9.4f*/s]\r\n", measurements[0], measurements[1], measurements[2]);
            fnd::tiny_printf(output);
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
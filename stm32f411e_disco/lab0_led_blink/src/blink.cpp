#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>

#include <stm32_ll_bus.h>

namespace {
    constexpr auto LED0 = periph::gpio::num::PD13;
}

namespace app {

    void main_thread(void*) {
        for(int i=0;;++i) {

            // Blink LED (1Hz)
            isix::wait_ms(500);
            periph::gpio::set(LED0, i%2);

            // Report number of loops on debug log (UART)
            if(i%2==0) {
                dbprintf("Loop %i",i>>1);
            }
        }
    }

}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    /**
     * Enable GPIOA for USART2 alternate function
     * 
     * @note Undermentioned dblog initialization
     *       uses "serial0" (USART2) peripheral that
     *       is connected to the PA2 & PA3 pins 
     *       (alternate function AF07). Port A is
     *       required to be clocked to select pin
     *       mode (i.e. alternate)
     */
    LL_AHB1_GRP1_EnableClock(
        LL_AHB1_GRP1_PERIPH_GPIOA
    );

    // Configure logging module
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

    // Configure PD13 pin LED as an output
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    periph::gpio::setup( 
        LED0,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );

    // Create task
	isix::task_create( app::main_thread, nullptr, 1536, isix::get_min_priority() );

    // Send welcome message to the log (UART)
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");

    // Begin scheduling
    isix::start_scheduler();

	return 0;
}

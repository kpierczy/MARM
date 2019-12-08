/** Example shows howto use semaphores for interasks
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>



namespace {
	/* Initialize the debug USART */
	auto usart_protected_init() -> void {
		static isix::mutex m_mtx;
		dblog_init_locked(
				[](int ch, void*) {
					return periph::drivers::uart_early::putc(ch);
				},
				nullptr,
				[]() { m_mtx.lock();  },
				[]() { m_mtx.unlock(); },
				periph::drivers::uart_early::open, "serial0", 115200
		);
	}
}

// Start main function
auto main() -> int
{
	usart_protected_init();
	// Create the samaphore and val
	static isix::semaphore sem { 0, 1 };
	// Wait some time before startup
    isix::wait_ms( 500 );
    dbprintf("<<<< LAB5 semaphore");
	//TODO: LAB5 user code here
	isix::start_scheduler();
	return 0;
}

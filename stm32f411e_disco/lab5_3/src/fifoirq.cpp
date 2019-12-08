/** Example shows howto use fifo from irq context
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>

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


extern "C" {
	//! EXTI 0 vector
	void exti0_isr_vector() {
		//! EXTI ISR vector here
	}
}

// Start main function
auto main() -> int
{
	usart_protected_init();
	// Create the samaphore and val
	// Wait some time before startup
    isix::wait_ms(500);
    dbprintf("<<<< LAB5 fifo queue");
	//TODO: Lab code here
	// Create task for blinking
	isix::start_scheduler();
	return 0;
}



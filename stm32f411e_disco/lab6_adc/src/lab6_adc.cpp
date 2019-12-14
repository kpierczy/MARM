#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <periph/drivers/serial/uart_early.hpp>
#include <foundation/sys/dbglog.h>
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

auto main() -> int
{
	usart_protected_init();
	// Wait some time before startup
    isix::wait_ms( 500 );
    dbprintf("<<<< LAB6 ADC converter ");
	//TODO:  LAB5 user code here
	isix::start_scheduler();
	return 0;
}


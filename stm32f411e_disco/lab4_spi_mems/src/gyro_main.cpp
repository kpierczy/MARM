/** LAB4 mems SPI sensor template
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/drivers/spi/spi_master.hpp>
#include <isix.h>
#include <periph/dma/dma.hpp>
#include <cstring>

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

	


	//Task for the led blinking
	auto watch_sensor() -> void {
		namespace opt = periph::option;
		int ret {};
		periph::drivers::spi_master spi("spi1");
		do {
			if((ret=spi.set_option(opt::speed(10E6)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::polarity(opt::polarity::low)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::phase(opt::phase::_1_edge)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::dwidth(8)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::bitorder(opt::bitorder::msb)))<0) {
				break;
			}
			if((ret=spi.open(ISIX_TIME_INFINITE))<0) {
				break;
			}
			//TODO: LAB4 user code here
		} while(0);
		dbg_info("Task exit with code %i", ret);
	}

	}


// Start main function
auto main() -> int
{
	usart_protected_init();
	// Wait some time before startup
    isix::wait_ms(500);
	static constexpr auto tsk_stack = 1024;
	static constexpr auto tsk_prio = 6;
	static auto watch_task =
		isix::thread_create_and_run(tsk_stack,tsk_prio,0,watch_sensor);
    dbprintf("<<<< LAB4 SPI MEMS sensor demo >>>>");
	isix::start_scheduler();
}



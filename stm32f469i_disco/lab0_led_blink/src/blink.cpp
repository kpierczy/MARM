#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>

namespace {
    constexpr auto led_0 = periph::gpio::num::PD5;
}

namespace {
	static void
	sdram_memtest(void)
	{
		uint32_t *addr;
		int i;
		addr = (uint32_t *)0xc0000000;
		for (i = 0; i < (4 * 1024 * 1024); i++) {
			/* Test */
			*(volatile uint32_t *)(addr + i) = i;
		}
		for (i = 0; i < (4 * 1024 * 1024); i++) {
			if (*(volatile uint32_t *)(addr + i) != uintptr_t(i)) {
				dbprintf("sdram test failed %x",
			    *(volatile uint32_t *)(addr + i), addr +i );
				break;
			}
		}
		dbprintf("sdram test completed");
	}

}

namespace app {
    void test_thread(void*) {
		sdram_memtest();
        for(int i=0;;++i) {
            isix::wait_ms(500);
            if(i%2==0) {
                dbprintf("Loop %i",i>>1);
            }
            periph::gpio::set(led_0, i%2);
        }
    }
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );
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
    periph::gpio::setup( led_0,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );
	isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
	isix::start_scheduler();
	return 0;
}

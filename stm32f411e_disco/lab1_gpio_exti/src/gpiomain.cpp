#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>


namespace app {
    auto lab1_thread() -> void
    {
        //TODO: User lab code here
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
		periph::drivers::uart_early::open, "serial0", 115200
	);
    static constexpr auto stack_size = 2048;
    static auto thread1 = 
        isix::thread_create_and_run(stack_size,isix::get_min_priority(),0,app::lab1_thread);
    dbprintf("Lab 1 - GPIO usage");
	isix::start_scheduler();
	return 0;
}
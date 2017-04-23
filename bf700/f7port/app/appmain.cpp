/*
 * =====================================================================================
 *
 *       Filename:  appmain.cpp
 *
 *    Description:  Minimal application for porting purpose
 *
 *        Version:  1.0
 *        Created:  20.04.2017 23:13:11
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p@boff.pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */


#include <config/conf.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <stm32gpio.h>
#include <isix.h>


#ifdef PDEBUG
namespace {
namespace usart_debug {
	isix::semaphore m_ulock_sem { 1, 1 };
	void lock()
	{
		m_ulock_sem.wait( ISIX_TIME_INFINITE );
	}
	void unlock()
	{
		m_ulock_sem.signal();
	}
}}
#endif




static const auto LED_PORT = GPIOG;


void task_test_nanana( void* )
{
	for(;;) {
		isix::wait_ms(1000);
		dbprintf("Tick tick #1" );
	}
}


void pulse_test( void* )
{
	using namespace stm32;
	gpio_config_ext( LED_PORT, 0xf0, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_LOW );
	// Test the APP environment
	 for(int r=0;;r++) {
		gpio_set_clr_mask( LED_PORT, r, 0x20 );
		gpio_set_clr_mask( LED_PORT, ~gpio_get_mask(GPIOB,0xC0), 0xc0 );
		gpio_set_clr_mask( LED_PORT, gpio_get(GPIOE,0)?1<<4:0, 0x10 );
		isix::wait_ms( 10 );
	}
}


int main()
{
	isix::wait_ms( 500 );
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ
	);
	dbprintf("Hello from F7 board cpuid" );
	isix::task_create( task_test_nanana, nullptr, 512, isix::get_min_priority() );
	isix::task_create( pulse_test, nullptr, 512, isix::get_min_priority() );
	isix::start_scheduler();
}

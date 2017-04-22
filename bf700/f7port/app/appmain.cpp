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



int main()
{

	//isix::wait_ms( 500 );
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ 
	);
	dbprintf("Hello from F7 board");
}

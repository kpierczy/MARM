/**
 * =====================================================================================
 * 	File: platform_startup.cpp
 * 	Created Date: Tuesday, November 19th 2019, 10:01:58 pm
 * 	Author: Lucjan Bryndza
 * 	Copyright (c) 2019 BoFF
 * 
 * 	GPL v2/3
 * =====================================================================================
 */


/*
 * platform_setup.cpp
 *  Platform initializaton specific code
 *  Created on: 20 lis 2013
 *      Author: lucck
 */
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_gpio.h>
#include <config/conf.h>
#include <functional>
#include <isix/arch/irq.h>
#include <isix.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>

namespace drv {
namespace board {
namespace {


/* This function is called after C init code
   end just before main function call
*/
bool uc_periph_setup()
{

	constexpr auto retries=100000;

	isix_set_irq_vectors_base( &_exceptions_vectors );

    //! Deinitialize RCC
    LL_RCC_DeInit();
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	LL_FLASH_EnablePrefetch();
	//! Set MCU Prescallers
	LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
	LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );
	LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
	//! Enable HSE generator
	LL_RCC_HSE_Enable();
	for( int i=0; i<retries; ++i ) {
		if(LL_RCC_HSE_IsReady()) {
			break;
		}
	}
	if( !LL_RCC_HSE_IsReady() ) {
		return false;
	}

	//Enable clocks for GPIOS
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA|LL_AHB1_GRP1_PERIPH_GPIOB|
		LL_AHB1_GRP1_PERIPH_GPIOC|LL_AHB1_GRP1_PERIPH_GPIOD|LL_AHB1_GRP1_PERIPH_GPIOE );


	/** User should configure clocks to following values:
	 * HCLK = 100MHz (CPU frequency)
	 * PCLK1 = 50MHz (Peripheral bus 1)
	 * PCLK2 = 100MHz (Peripheral bus 2)
	 * Please assume that crystal connected to the OSC_IN OSC_OUT pins has
	 * 8MHz value.
	 * TODO: User code here 
	 */
	return true;
}



//! Application crash called from hard fault
void application_crash( crash_mode type, unsigned long* sp )
{
#ifdef PDEBUG
	cortex_cm3_print_core_regs( type, sp );
#else
	static_cast<void>(type);
	static_cast<void>(sp);
#endif
	for(;;) asm volatile("wfi\n");
}


}	//Unnamed NS

extern "C" {


//! This function is called just before call global constructors
void _external_startup(void)
{
	//SysTimer values
	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++) asm volatile("nop\n");

	//Initialize system perhipheral

	if( uc_periph_setup() ) {
		//1 bit for preemtion priority
		isix_set_irq_priority_group( isix_cortexm_group_pri7 );
		//Initialize isix
		isix::init(CONFIG_HCLK_HZ);
	} else {
		//TODO: Handle failure initialization
		//! Initialization failure
		for(;;);
	}
}


//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{
	_cm3_hard_hault_entry_fn( application_crash );
}

} /* extern C */

 
}}	//NS drv


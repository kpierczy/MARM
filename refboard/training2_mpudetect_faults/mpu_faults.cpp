/*
 * =====================================================================================
 *
 *       Filename:  mpu_faults.cpp
 *
 *    Description: MPU how to detect faults.
 *
 *        Version:  1.0
 *        Created:  20.10.2016 15:13:23
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB)
 *
 * =====================================================================================
 */

#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <stm32system.h>
#include <stm32gpio.h>
#include <stm32crashinfo.h>
#include <atomic>
#include <arm-v7m/mpu.h>


namespace {
	std::atomic<bool> got_key;
}

int main() {
	dblog_init( stm32::usartsimple_putc, nullptr, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	dbprintf("Faults example");

	//Vector table in flash 
	SCB->VTOR = NVIC_VectTab_FLASH;
	//1 bit for preemtion priority
	stm32::nvic_priority_group(NVIC_PriorityGroup_1);

	constexpr auto irq_prio = 1;
	constexpr auto irq_sub = 7;
	// Configure systick timer
	constexpr auto tick_freq = 10;
	stm32::systick_config( CONFIG_HCLK_HZ/8/tick_freq  );
	//Set timer priority
	stm32::nvic_set_priority( SysTick_IRQn, irq_prio, irq_sub );
	//TODO: On startup MPU is disabled
	if( 0 ) {
		mpu_set_region_size( 0, 0x0000'0000, 0x1'0000,
				MPU_RGN_PERM_PRV_NO_USR_NO|MPU_RGN_MEMORY| MPU_RGN_PERM_NX );
		mpu_enable_region( 0 );
		mpu_enable( MPU_CONFIG_PRIV_DEFAULT );
	}
	for(;;)
	if( got_key ) {
		constexpr unsigned long* ulong_at_nulladdr = nullptr;
		const auto val0 = *ulong_at_nulladdr;
		dbprintf("Read nullptr %08lx", val0 );
		got_key = false;
	}

}

namespace {
extern "C" {

	void __attribute__((interrupt,naked)) hard_fault_exception_vector() {
		cm3_hard_hault_regs_dump();
	}

	//! Systick ISR vector
	void __attribute__((interrupt)) systick_isr_vector() {
		const auto IOPORT = GPIOB;
		constexpr auto IOPIN = 2U;
		const auto val = !stm32::gpio_get(IOPORT,IOPIN);
		static bool pval;
		if(val && !pval ) {
			got_key = true;
		}
		pval = val;
	}
}}




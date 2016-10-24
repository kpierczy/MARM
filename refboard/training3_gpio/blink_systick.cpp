/*
 * =====================================================================================
 *
 *       Filename:  nvic_systick.cpp
 *
 *    Description: NVIC and systick on bare metal
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
#include <stm32rcc.h>
#include <stm32syscfg.h>
#include <stm32gpio.h>

namespace {


	constexpr auto PLL_M = 25;
	constexpr auto PLL_N = 336;
	constexpr auto PLL_P = 2;	//168MHZ master clock
	constexpr auto PLL_Q = 7;	//48MHz for USB clk
	//Configure port and PIN
	const auto LED_PORT = GPIOG;
	constexpr auto LED_PIN = 4;

	int core_freq;

	void periph_config()
	{
		using namespace stm32;
		//Configure common ADCS
		rcc_apb2_periph_clock_cmd( RCC_APB2Periph_ADC, true );
		//Setup DMA
		rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, true );

		//Configure used GPIOS
		rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
				RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|
				RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOH, true );


	}
	unsigned system_config()
	{
		using namespace stm32;

		rcc_flash_latency(CONFIG_HCLK_HZ);
		rcc_hse_config( RCC_HSE_ON );
		rcc_wait_for_hse_startup();
		rcc_pll_config( RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q );
		rcc_pll_cmd( true );
		for( auto r=0; r<100000; ++r ) {
			if( !rcc_get_flag_status( RCC_FLAG_PLLRDY ) )
				break;
		}

		rcc_pclk2_config( RCC_HCLK_Div2 );
		rcc_pclk1_config( RCC_HCLK_Div4 );
		rcc_hclk_config(  RCC_SYSCLK_Div1 );
		//Enable GPIO system compensation cell block for gpio > 50MHz
		rcc_ahb2_periph_clock_cmd( RCC_APB2Periph_SYSCFG, true );
		syscfg_compensation_cell_cmd( true );




		// Enable main PLL
		rcc_sysclk_config( RCC_SYSCLKSource_PLLCLK );
		constexpr auto PLL_SRC = 0x08;
		for( auto r=0; r<100000; ++r ) {
			if( rcc_get_sysclk_source() != PLL_SRC )
				break;
		}
		return CONFIG_HCLK_HZ;
	}

	extern "C" {

		//! This function is called just before call global constructors
		void _external_startup(void)
		{
			using namespace stm32;

			//Give a chance a JTAG to reset the CPU
			for(unsigned i=0; i<1000000; i++) stm32::nop();

			//Initialize system perhipheral
			core_freq = system_config();

			//Configure perhipherals
			periph_config();

		}
	}

}




int main() {
	dblog_init( stm32::usartsimple_putc, nullptr, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	dbprintf("Core frequency %i",core_freq);

#if CONFIG_WITH_SBL_BOOTLOADER_ENABLED
		SCB->VTOR = NVIC_VectTab_FLASH + 0x4000;
#else
		SCB->VTOR = NVIC_VectTab_FLASH;
#endif
	//1 bit for preemtion priority
	stm32::nvic_priority_group(NVIC_PriorityGroup_1);

	constexpr auto irq_prio = 1;
	constexpr auto irq_sub = 7;
	// Configure systick timer
	constexpr auto tick_freq = 10;
	stm32::systick_config( core_freq/8/tick_freq  );
	//Set timer priority
	stm32::nvic_set_priority( SysTick_IRQn, irq_prio, irq_sub );
	//Configure GPIO port
	stm32::gpio_clock_enable( LED_PORT, true );
	stm32::gpio_config( LED_PORT, LED_PIN, stm32::GPIO_MODE_OUTPUT, 
			stm32::GPIO_PUPD_NONE, stm32::GPIO_SPEED_25MHZ, stm32::GPIO_OTYPE_PP );
}

namespace {
extern "C" {
	void __attribute__((interrupt)) systick_isr_vector() {
		static int i;
		if(i++%2) stm32::gpio_set( LED_PORT, LED_PIN );
		else stm32::gpio_clr( LED_PORT, LED_PIN );
	}
}}




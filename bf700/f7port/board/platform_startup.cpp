#include <config/conf.h>
#include <stm32rcc.h>
#include <stm32pwr.h>
//#include <stm32adc.h>
#include <isix.h>
#include <stm32crashinfo.h>
#include <stm32syscfg.h>
//#include <stm32dma.h>
#include <irq_vectors_symbol.h>
#include <isix/arch/irq.h>

namespace drv {
namespace board {
namespace {


//Number of isix threads
constexpr unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
constexpr unsigned MHZ = 1000000;

constexpr auto PLL_M = 25;
constexpr auto PLL_N = 432;
constexpr auto PLL_P = stm32::RCC_PLLP_DIV2;	//168MHZ master clock
constexpr auto PLL_Q = 9;	//48MHz for USB clk
constexpr auto PLL_R = 2;


const auto MCO1_PORT =  GPIOA;
constexpr auto MCO1_PIN = 8;


/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
unsigned system_config()
{
	using namespace stm32;

    rcc_flash_latency(CONFIG_HCLK_HZ);
	rcc_hse_config( RCC_HSE_ON );
	rcc_wait_for_hse_startup();
	rcc_pll_config( RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q, PLL_R );
	rcc_pll_cmd( true );
	for( auto r=0; r<100000; ++r ) {
		if( !rcc_get_flag_status( RCC_FLAG_PLLRDY ) )
			break;
	}

    rcc_pclk2_config( RCC_HCLK_Div2 );
    rcc_pclk1_config( RCC_HCLK_Div4 );
	rcc_hclk_config(  RCC_SYSCLK_Div1 );
	//Enable GPIO system compensation cell block for gpio > 50MHz
    rcc_ahb2_periph_clock_cmd( RCC_APB2ENR_SYSCFGEN, true );
    syscfg_compensation_cell_cmd( true );


	isix_set_irq_vectors_base( &_exceptions_vectors );

    // Enable main PLL
	rcc_sysclk_config( RCC_SYSCLKSource_PLLCLK );
    constexpr auto PLL_SRC = 0x08;
    for( auto r=0; r<100000; ++r ) {
	if( rcc_get_sysclk_source() != PLL_SRC )
		break;
    }
    return CONFIG_HCLK_HZ;
}


void periph_config()
{
	using namespace stm32;
	//Configure common ADCS
    rcc_apb2_periph_clock_cmd( RCC_APB2ENR_ADC1EN, true );
	//Setup DMA
	rcc_ahb1_periph_clock_cmd( RCC_AHB1ENR_DMA1EN|RCC_AHB1ENR_DMA2EN, true );

	//Configure used GPIOS
	rcc_ahb1_periph_clock_cmd( RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|
		RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN|
		RCC_AHB1ENR_GPIOFEN|RCC_AHB1ENR_GPIOGEN|RCC_AHB1ENR_GPIOHEN, true );

	//Configure MCO1 as master clock output for external devs
	gpio_config( MCO1_PORT, MCO1_PIN, GPIO_MODE_ALTERNATE,
			GPIO_PUPD_PULLUP, GPIO_SPEED_HI );
	rcc_mco1_config( RCC_MCO1Source_HSE, RCC_MCO1Div_1 );


}


//NS end

}
}
}


extern "C" {

//! This function is called just before call global constructors
void _external_startup(void)
{

	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++)
		asm volatile("nop\n");

	//Initialize system perhipheral
	const auto freq = drv::board::system_config();
	(void)freq;

	//Configure perhipherals
	drv::board::periph_config();

	//1 bit for preemtion priority
	isix_set_irq_priority_group( isix_cortexm_group_pri7 );

	//Initialize isix
	//isix::init(freq);

}



//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{
	cm3_hard_hault_regs_dump();
}

} /* extern C */



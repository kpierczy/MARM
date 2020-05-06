/*================================================================================
 *
 *    Filename : dts.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : System's device tree
 *
 *===============================================================================*/


#include <cmath>
#include <periph/dt/dts.hpp>
#include <periph/gpio/gpio_numbers.hpp>
#include <periph/dt/dts_config.hpp>
#include <isix/arch/irq.h>
#include <stm32_ll_usart.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>



namespace periph::dt::_dts_config {

	namespace {
		constexpr clock clk[] {
			{	bus::ahb1, []() -> unsigned {
				LL_RCC_ClocksTypeDef clk;
				LL_RCC_GetSystemClocksFreq(&clk);
				return clk.HCLK_Frequency;
				}
			},
			{	bus::apb1, []() -> unsigned {
				LL_RCC_ClocksTypeDef clk;
				LL_RCC_GetSystemClocksFreq(&clk);
				return clk.PCLK1_Frequency;
				}
			},
			{	bus::apb2, []() -> unsigned {
				LL_RCC_ClocksTypeDef clk;
				LL_RCC_GetSystemClocksFreq(&clk);
				return clk.PCLK2_Frequency;
				}
			},
			{	bus::cpu, []() -> unsigned {
				LL_RCC_ClocksTypeDef clk;
				LL_RCC_GetSystemClocksFreq(&clk);
				return clk.SYSCLK_Frequency;
				}
			},
			{}
		};

		//Serial debug interface
		constexpr pin ser0_pins[] {
			{ pinfunc::txd, gpio::num::PA2 },
			{}
		};

		constexpr device devices[]
		{
			{
				"serial0", reinterpret_cast<uintptr_t>(USART2),
				bus::apb1, LL_GPIO_AF_7,
				unsigned(std::log2(LL_APB1_GRP1_PERIPH_USART2)),
				ser0_pins,
				nullptr
			},
			{}
		};
	}

	//! The machine config
	constexpr configuration the_machine_config {
		clk,
		devices
	};

}

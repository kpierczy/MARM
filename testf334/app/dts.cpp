#include <cmath>
#include <periph/dt/dts.hpp>
#include <periph/gpio/gpio_numbers.hpp>
#include <periph/dt/dts_config.hpp>
#include <stm32_ll_usart.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>



namespace periph {
namespace dt {
namespace _dts_config {

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

	constexpr pin ser0_pins[] {
		{ pinfunc::txd, gpio::num::PC4 },
		{}
	};


	constexpr device devices[]
	{
		{
			"serial0", reinterpret_cast<uintptr_t>(USART1),
			bus::apb2, LL_GPIO_AF_7,
			unsigned(std::log2(LL_APB2_GRP1_PERIPH_USART1)),
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


} } }

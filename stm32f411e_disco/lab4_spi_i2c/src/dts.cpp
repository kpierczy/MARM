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
#include <stm32_ll_spi.h>
#include <stm32_ll_i2c.h>
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

		// Serial debug interface
		constexpr pin ser0_pins[] {
			{ pinfunc::txd, gpio::num::PA2 },
			{}
		};

		// SPI controller
		constexpr pin spi1_pins[] {
			{ pinfunc::sck , gpio::num::PA5 },	// SCK config			
			{ pinfunc::miso, gpio::num::PA6 },	// MISO config
			{ pinfunc::mosi, gpio::num::PA7 },	// MOSI config
			{ pinfunc::cs0 , gpio::num::PE3 },	// CS config
			{}
		};

		constexpr device_conf spi1_conf {
			{},
			SPI1_IRQn,
			1,7,				// IRQ prio subprio
			device_conf::fl_dma	// Use DMA transfer
		};

		// I2C controller
		constexpr pin i2c1_pins[] {
			{ pinfunc::scl, gpio::num::PB6 }, //SCK pin
			{ pinfunc::sda, gpio::num::PB9 }, //SCL pin
			{}
		};

		constexpr device_conf i2c1_conf {
			{},
			I2C1_EV_IRQn,
			1,7, 		  // IRQ prio subprio
			0			  // Don't use DMA transfer
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

			{
				"spi1", reinterpret_cast<uintptr_t>(SPI1),
				bus::apb2, LL_GPIO_AF_5,
				unsigned(std::log2(LL_APB2_GRP1_PERIPH_SPI1)),
				spi1_pins,
				&spi1_conf
			},
			{
				"i2c1", reinterpret_cast<uintptr_t>(I2C1),
				bus::apb1, LL_GPIO_AF_4,
				unsigned(std::log2(LL_APB1_GRP1_PERIPH_I2C1)),
				i2c1_pins,
				&i2c1_conf
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

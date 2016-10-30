/* ------------------------------------------------------------------ */
/*
 * config.hpp
 *
 *  Created on: 2010-01-28
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef CONFIG_HPP_
#define CONFIG_HPP_
/* ------------------------------------------------------------------ */
namespace config
{
#if defined(STM32MCU_MAJOR_TYPE_F1)
	namespace
	{
			//HCLK system speed
			const unsigned XTAL_HZ =  25000000;
			const unsigned HCLK_HZ =  75000000;
			const unsigned PCLK1_HZ = HCLK_HZ/4;
			const unsigned PCLK2_HZ = HCLK_HZ/2;
	}
#elif defined(STM32MCU_MAJOR_TYPE_F4)
	namespace
	{
		//HCLK system speed
		const unsigned XTAL_HZ = 8000000;
		const unsigned HCLK_HZ = 168000000;
		const unsigned PCLK1_HZ = HCLK_HZ/4;
		const unsigned PCLK2_HZ = HCLK_HZ/2;
	}
#elif defined(STM32MCU_MAJOR_TYPE_F2)
	namespace
	{
		//HCLK system speed
		const unsigned XTAL_HZ = 8000000;
		const unsigned HCLK_HZ = 120000000;
		const unsigned PCLK1_HZ = HCLK_HZ/4;
		const unsigned PCLK2_HZ = HCLK_HZ/2;
	}
#endif
}
/* ------------------------------------------------------------------ */
#endif /* CONFIG_HPP_ */

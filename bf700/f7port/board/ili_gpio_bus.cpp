/*
 * ili_gpio_bus.cpp
 *
 *  Created on: 21 lis 2013
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <board/ili_gpio_bus.hpp>
#include <stm32gpio.h>
#include <stm32system.h>
#include <stm32rcc.h>
#include <isix.h>
/* ------------------------------------------------------------------ */
namespace drv {
/* ------------------------------------------------------------------ */
namespace {
	uint8_t data_get()
	{
		using namespace stm32;
		auto a1 = gpio_get_mask( GPIOD, 0xc003 );
		auto n1 = (( a1>>14 ) & 0b0011 ) | ((a1<<2) & 0b1100 );
		auto n2 =  gpio_get_mask( GPIOE, 0x780 ) >> 7;
		return (n2<<4) |  n1;
	}
	void data_set( uint16_t val )
	{
		using namespace stm32;
		gpio_set_clr_mask( GPIOD, ((val&0b0011)<<14)|((val&0b1100)>>2), 0xc003 );
		gpio_set_clr_mask( GPIOE, ((val&0xf0)>>4)<<7, 0x780 );
	}
}

/* ------------------------------------------------------------------ */
void ili_gpio_bus::bus_dir( ili_gpio_bus::dir_t dir )
{
	using namespace stm32;
	if( dir==dir_t::out && m_dir==dir_t::in )
	{
		gpio_abstract_config_ext( GPIOE, 0x780, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL );
		gpio_abstract_config_ext( GPIOD, 0xc003, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL );
	}
	else if( dir==dir_t::in && m_dir==dir_t::out  )
	{
		gpio_abstract_config_ext( GPIOE, 0x780, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
		gpio_abstract_config_ext( GPIOD, 0xc003, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
	}
	m_dir = dir;
}
/* ------------------------------------------------------------------ */
//Constructor
ili_gpio_bus::ili_gpio_bus()
{
	using namespace stm32;
	//Enable periph backlight

	gpio_abstract_config_ext( GPIOE, 0x780, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
	gpio_abstract_config_ext( GPIOD, 0xc003, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );


	gpio_abstract_config_ext( GPIOD,
		bv(7)|bv(5)|bv(4), AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL);
	gpio_abstract_config_ext( GPIOF, bv(2)|bv(3), AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL );
	gpio_set_mask( GPIOD, bv(7)|bv(5)|bv(4) );
	gpio_set_mask( GPIOF, bv(2)|bv(3) ); /* PWM Configuration */
	gpio_abstract_config( GPIOE, 6, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_LOW );
	gpio_set( GPIOE, 6 );
}

/* ------------------------------------------------------------------ */
// Lock bus and set address
void ili_gpio_bus::set_ctlbits( int bit, bool val )
{
	using namespace stm32;
	const auto io_fn = val?(&gpio_set):(&gpio_clr);
	switch( bit )
	{
	case CSL_BIT_CMD: io_fn( GPIOD, 7 ); break;
	case RS_BIT_CMD:  io_fn( GPIOF, 2 ); break;
	case RST_BIT_CMD: io_fn( GPIOF, 3 ); break;
	}
}
/* ------------------------------------------------------------------ */
/* Read transfer */
void ili_gpio_bus::read( void *buf, std::size_t len )
{
	using namespace stm32;
	bus_dir( dir_t::in );
	for( size_t l=0; l<len; ++l )
	{
		gpio_clr( GPIOD, 4 );
		nop(); nop(); nop(); nop();
		*(reinterpret_cast<uint8_t*>(buf)+l) = data_get();
		gpio_set( GPIOD, 4 );
	}
}
/* ------------------------------------------------------------------ */
/* Write transfer */
void ili_gpio_bus::write( const void *buf, size_t len )
{
	using namespace stm32;
	bus_dir( dir_t::out );
	for( size_t l=0; l<len; ++l )
	{
		//One write
		data_set( *(reinterpret_cast<const uint8_t*>(buf)+l) );
		gpio_clr( GPIOD, 5 );	//WR down
		gpio_set( GPIOD, 5 );	//WR up
	}
}
/* ------------------------------------------------------------------ */
/* Fill pattern */
void ili_gpio_bus::fill( unsigned value, size_t nelms )
{
	using namespace stm32;
	bus_dir( dir_t::out );
	for( size_t l=0; l<nelms; ++l )
	{
		//One write
		data_set( value );
		gpio_clr( GPIOD, 5 );	//WR down
		gpio_set( GPIOD, 5 );	//WR up
		data_set( value>>8 );
		gpio_clr( GPIOD, 5 );	//WR down
		gpio_set( GPIOD, 5 );	//WR up
	}
}
/* ------------------------------------------------------------------ */
/* Set PWM  */
void ili_gpio_bus::set_pwm( int percent )
{
#if 0
	static constexpr auto c_scale1 = 1000;
	static constexpr auto c_scale2 = 550;
	if( percent > 100 ) percent = 100;
	else if( percent < 0 ) percent = 0;
	percent = (percent*c_scale2)/c_scale1;
	stm32::tim_set_ccr( TIM3, stm32::tim_cc_chn3, percent );
#else
	static_cast<void>(percent);
#endif
}
/* ------------------------------------------------------------------ */
/* Wait ms long delay */
void ili_gpio_bus::delay( unsigned tout )
{
	isix_wait_ms( tout );
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */

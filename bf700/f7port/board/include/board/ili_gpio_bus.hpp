/*
 * ili_gpio_bus.hpp
 *
 *  Created on: 21 lis 2013
 *      Author: lucck
 */

#ifndef DRV_ILI_GPIO_BUS_HPP_
#define DRV_ILI_GPIO_BUS_HPP_
/* ------------------------------------------------------------------ */
#include <gfx/drivers/disp/ili9341.hpp>
/* ------------------------------------------------------------------ */
namespace drv {
/* ------------------------------------------------------------------ */
class ili_gpio_bus : public gfx::drv::disp_bus
{
	static constexpr auto CSL_BIT_CMD = 0;
	static constexpr auto RS_BIT_CMD = 1;
	static constexpr auto RST_BIT_CMD = 2;
	static constexpr auto DEFAULT_BACKLIGHT = 1;
	static constexpr unsigned bv( unsigned pin )
	{
		return 1<<pin;
	}
	enum class dir_t : bool
	{
		in,
		out
	};
	void bus_dir( dir_t dir );
public:
	//Constructor
	ili_gpio_bus();
	// Destructor
	virtual ~ili_gpio_bus() {}
	// Lock bus and set address
	virtual void set_ctlbits( int bit, bool val );
	/* Read transfer */
	virtual void read( void *buf, std::size_t len );
	/* Write transfer */
	virtual void write( const void *buf, size_t len );
	/* Fill pattern */
	virtual void fill( unsigned value, size_t nelms );
	/* Wait ms long delay */
	virtual void delay( unsigned tout );
	/* Set PWM  */
	virtual void set_pwm( int percent );
private:
	dir_t m_dir { dir_t::in };
};

/* ------------------------------------------------------------------ */
}

/* ------------------------------------------------------------------ */


#endif /* ILI_GPIO_BUS_HPP_ */

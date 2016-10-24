/*
 * ili_gpio_bus.hpp
 *
 *  Created on: 21 lis 2013
 *      Author: lucck
 */

#include <gfx/drivers/disp/ili9341.hpp>

namespace drv {

class ili_fsmc_bus : public gfx::drv::disp_bus
{
	enum ctlbits {
		CSL_BIT_CMD, RS_BIT_CMD, RST_BIT_CMD
	};
	static constexpr auto DEFAULT_BACKLIGHT = 1;
	static constexpr unsigned bv( unsigned pin )
	{
		return 1<<pin;
	}
public:
	//Constructor
	ili_fsmc_bus();
	// Destructor
	virtual ~ili_fsmc_bus() {}
	// Lock bus and set address
	void set_ctlbits( int bit, bool val ) override;
	/* Read transfer */
	void read( void *buf, std::size_t len ) override;
	/* Write transfer */
	void write( const void *buf, size_t len ) override;
	/* Fill pattern */
	void fill( unsigned value, size_t nelms ) override;
	/* Wait ms long delay */
	void delay( unsigned tout ) override;
	/* Set PWM  */
	void set_pwm( int percent ) override;
private:
	void fsmc_gpio_setup();
	void fsmc_setup();
private:
	volatile uint8_t* m_bus_addr {};
};

}



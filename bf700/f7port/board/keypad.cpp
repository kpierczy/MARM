/*
 * =====================================================================================
 *
 *       Filename:  keypad.cpp
 *
 *    Description:  Keypad driver for graphics library
 *
 *        Version:  1.0
 *        Created:  27.08.2016 10:51:47
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <board/keypad.hpp>
#include <stm32gpio.h>


namespace dev {

// Port configuration
namespace {
	static const auto key_port = GPIOF;
	static const auto key_portok = GPIOB;
	static constexpr auto key_left = 12;
	static constexpr auto key_right = 13;
	static constexpr auto key_up = 14;
	static constexpr auto key_down = 15;
	static constexpr auto key_ok = 2;

	bool io( unsigned pin ) {
		return stm32::gpio_get( key_port, pin );
	}
	bool io() {
		return stm32::gpio_get( key_portok, key_ok );
	}

}

// Constructor
keypad::keypad( gfx::gui::frame& frm )
	: m_thr( isix::thread_create_and_run( c_thr_stk, isix::get_min_priority(), 0,
		std::bind( &keypad::thread, this ) ) )
	, m_frm( frm )
{
	stm32::gpio_config_ext( key_port,
		_bv(key_left)|_bv(key_right)|_bv(key_up)|_bv(key_down),
		stm32::GPIO_MODE_INPUT, stm32::GPIO_PUPD_PULLUP );
	stm32::gpio_config( key_portok, key_ok, stm32::GPIO_MODE_INPUT, stm32::GPIO_PUPD_PULLUP );

}



//Report key to gui engine
void keypad::report_key( char key , ks type )
{
	const gfx::input::event_info ei {
		isix::get_jiffies(),
		gfx::input::event_info::evtype::EV_KEY,
		nullptr, {
			type, key, 0, 0
		}
	};
	m_frm.report_event( ei );
}


// Thread configuration data
void keypad::thread() noexcept
{
		using keys = gfx::input::kbdcodes;
		bool pok{}, pl{}, pr{}, pu{}, pd{};
		for(;;)
		{
			//ok
			const bool ok = io();
			if( !ok && pok ) report_key( keys::enter, ks::DOWN );
			else if( ok && !pok ) report_key( keys::enter, ks::UP );
			pok = ok;
			//left
			const bool l = io(key_left);
			if( !l && pl ) report_key( keys::os_arrow_left, ks::DOWN );
			if( l && !pl ) report_key( keys::os_arrow_left, ks::UP );
			pl = l;
			//right
			const bool r = io(key_right);
			if( !r && pr ) report_key( keys::os_arrow_right, ks::DOWN );
			if( r && !pr ) report_key( keys::os_arrow_right, ks::UP );
			pr = r;
			//right
			const bool u = io(key_up);
			if( !u && pu ) report_key( keys::os_arrow_up, ks::DOWN );
			if( u && !pu ) report_key( keys::os_arrow_up, ks::UP );
			pu = u;
			//down
			const bool d = io(key_down);
			if( !d && pd ) report_key( keys::os_arrow_down, ks::DOWN );
			if( d && !pd ) report_key( keys::os_arrow_down, ks::UP );
			pd = d;
			isix::wait_ms(c_keypad_delay);
		}
}


}


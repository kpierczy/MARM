/*
 * =====================================================================================
 *
 *       Filename:  keypad.hpp
 *
 *    Description:  Keypad driver for graphics library
 *
 *        Version:  1.0
 *        Created:  27.08.2016 10:02:05
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include <gfx/gui/frame.hpp>
#include <isix.h>

#pragma once
namespace dev {

	class keypad {
		static constexpr auto c_thr_stk = 384;
		static constexpr auto c_keypad_delay = 20;
	public:
		explicit keypad( gfx::gui::frame& );
		keypad( keypad& ) = delete;
		keypad& operator=( keypad& ) = delete;
	private:
		using ks = gfx::input::detail::keyboard_tag::status;
		void thread() noexcept;
		static constexpr auto _bv( unsigned x ) {
			return 1<<x;
		}
		void report_key( char key , ks type );
	private:
		isix::thread m_thr;
		gfx::gui::frame& m_frm;
	};

}

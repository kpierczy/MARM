/*
 * =====================================================================================
 *
 *       Filename:  tftdemo.hpp
 *
 *    Description:  TFT demo
 *
 *        Version:  1.0
 *        Created:  28.08.2016 20:16:40
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#pragma once

#include <isix.h>
#include <board/keypad.hpp>
#include <gfx/types.hpp>
#include <gfx/disp/gdi.hpp>
#include <gfx/gui/gui.hpp>
#include <gfx/drivers/disp/ili9341.hpp>
//#include <board/ili_fsmc_bus.hpp>
#include <board/ili_gpio_bus.hpp>

namespace app {


class tft_tester: public isix::task_base
{
private:
	//A window calllback for select item
	bool window_callback( const gfx::gui::event &ev );
	//Buttons callback
	bool on_click( const gfx::gui::event &ev );
	//On select item
	bool on_select_item( const gfx::gui::event &ev );
	bool on_seek_change( const gfx::gui::event &ev );
public:
	//Constructor
	tft_tester();
	gfx::gui::frame& get_frame() {
		return frame;
	}

protected:
	virtual void main();
private:
	//Base buttons and windows demo
	void windows_test();
	void gdi_test();
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	//ili_bus ibus;
	//drv::ili_fsmc_bus gbus;
	drv::ili_gpio_bus gbus;
	gfx::drv::ili9341 gdisp;
	gfx::gui::frame frame;
	gfx::gui::editbox* m_edit {};
	bool m_edit_mode = false;
};


}


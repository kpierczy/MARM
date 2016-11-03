/*
 * =====================================================================================
 *
 *       Filename:  tftdemo.cpp
 *
 *    Description:  TFT demo
 *
 *        Version:  1.0
 *        Created:  28.08.2016 20:15:18
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include "tftdemo.hpp"
#include <foundation/dbglog.h>

namespace app {

	//A window calllback for select item
	bool tft_tester::window_callback( const gfx::gui::event &ev )
	{
		using namespace gfx::input;
		bool ret {};
		if( ev.keyb.stat == detail::keyboard_tag::status::DOWN )
		{
			auto win = static_cast<gfx::gui::window*>(ev.sender);
			if( ev.keyb.key == kbdcodes::os_arrow_left && !m_edit_mode)
			{
				win->select_prev();
				ret = true;
			}
			else if( ev.keyb.key == kbdcodes::os_arrow_right && !m_edit_mode)
			{
				win->select_next();
				ret = true;
			}
			if( ev.keyb.key == kbdcodes::enter && m_edit == win->current_widget())
			{
				m_edit_mode = !m_edit_mode;
				m_edit->readonly( !m_edit_mode );
				dbprintf("Toggle edit mode %i", m_edit_mode );
			}
		}
		return ret;//	Need redraw
	}
	//Buttons callback
	bool tft_tester::on_click( const gfx::gui::event &ev )
	{
		auto b = static_cast<gfx::gui::button*>(ev.sender);
		dbprintf("Button %p clicked with desc %s", ev.sender, b->caption().c_str());
		return false;
	}
	//On select item
	bool tft_tester::on_select_item( const gfx::gui::event &ev )
	{
		auto b = static_cast<gfx::gui::choice_menu*>(ev.sender);
		dbprintf("Sel %p clicked with item %i", ev.sender, b->selection());
		return false;
	}
	bool tft_tester::on_seek_change( const gfx::gui::event &ev )
	{
		auto b = static_cast<gfx::gui::seekbar*>(ev.sender);
		dbprintf("Seek event pos %i", b->value());
		return false;
	}
	tft_tester::tft_tester()
		: gdisp( gbus ), frame(gdisp)
	{
		start_thread( STACK_SIZE, TASK_PRIO );
	}
	void tft_tester::main()
	{
		dbprintf("GDI test go");
#if 0
		//gdi_test();
		gdisp.power_ctl( gfx::drv::power_ctl_t::on );
		gdisp.clear( 0xAAA5 );
		for(;;) {
		for( int x=0;x<240;++x )
		for( int y=0;y<320;++y )
		{
			uint16_t pv;
			pv = gdisp.get_pixel(x,y);
			if( pv != 0xAAA5 ) {
				dbprintf("PIXEL VAL %04x", pv );
			}
		}
			dbprintf("OK PIXEL VAL %04x", gdisp.get_pixel(0,0) );
		}
#else
		windows_test();
#endif
	}
	//Base buttons and windows demo
	void tft_tester::windows_test()
	{
		using namespace gfx::gui;
		using namespace gfx::drv;
		//Menu for choice
		static constexpr choice_menu::item menu1[] = {
				{ 1, "linia 1" },
				{ 2, "linia 2" },
				{ 3, "linia 3" },
				{ 4, "linia 4" },
				{ 5, "linia 5" },
				{ 6, "linia 6" },
				{ 7, "linia 7" },
				{ 8, "linia 8" },
				{ 9, "linia 9" },
				{ 9, "linia 10" },
				{ 9, "linia 11" },
				choice_menu::end	//Termination
		};

		gdisp.power_ctl( power_ctl_t::on );
		window win( rectangle( 10, 10, 200, 300), frame, window::flags::border |window::flags::selectborder );
		button btn( rectangle(20, 20, 100 , 40), layout(), win );
		choice_menu choice1( rectangle(20, 147, 178, 120), layout(), win, choice_menu::style::normal );
		seekbar seek( rectangle(20, 65, 160 , 20), layout(), win );
		editbox edit( rectangle(20, 110, 160 , 30), layout(), win );
		edit.readonly(true);
		m_edit = &edit;
		seek.value( 50 );
		btn.caption("Button");
		edit.value("Text edit value");
		choice1.items( menu1 );
		//Connect windows callback to the main window
		win.connect(std::bind(&tft_tester::window_callback,this,std::placeholders::_1), event::evtype::EV_KEY);
		btn.connect(std::bind(&tft_tester::on_click,this,std::placeholders::_1),event::evtype::EV_CLICK);
		btn.pushkey( gfx::input::kbdcodes::enter);
		choice1.connect(std::bind(&tft_tester::on_select_item,this,std::placeholders::_1),event::evtype::EV_CLICK);
		seek.connect(std::bind(&tft_tester::on_seek_change,this,std::placeholders::_1),event::evtype::EV_CLICK);
		frame.set_focus( &win );
		frame.execute();
	}
	void tft_tester::gdi_test()
	{
		static uint16_t img[16*16];
		for( int x=0;x<16;x++ )
		for( int y=0;y<16;y++ )
		{
			if( y == 8 || y==0 || x==0 ||y==15||x==15)
				img[x + 16*y ] = gfx::rgb( 255 ,0, 255 );
			else
				img[x + 16*y ] = gfx::rgb( 255,255,0 );
		}
		gdisp.power_ctl( gfx::drv::power_ctl_t::on );
		gfx::disp::gdi gdi(gdisp);
		gdi.clear();
		ostick_t tbeg = isix::get_jiffies();
		dbprintf("TIME=%u", isix::get_jiffies() - tbeg );
		gdisp.fill(50,150,30,30, gfx::rgb(127,0,255));
		gdisp.blit( 20, 40, 16, 16, 0, img );
		tbeg = isix::get_jiffies();
		dbprintf("BLIT=%u", isix::get_jiffies() - tbeg );
		dbprintf( "PIX1=%04X %04x", gdisp.get_pixel( 1, 1 ) ,  gfx::rgb(0,255,0)  );
		dbprintf( "PIX2=%04X %04x", gdisp.get_pixel(140,140 ), gfx::rgb(255,0,0) );
		gdi.set_fg_color( gfx::rgb(64,255,45) );
		constexpr char txt[] = "Przykladowy napis";
		tbeg = isix::get_jiffies();
		gdi.draw_text(1,2, txt);
		dbprintf("string_time=%u", isix::get_jiffies() - tbeg );
		tbeg = isix::get_jiffies();
		gdi.draw_line( 1, 100, 100, 100 );
		dbprintf("line_time=%u", isix::get_jiffies() - tbeg );
		tbeg = isix::get_jiffies();
		gdi.draw_circle( 150, 150, 30 );
		dbprintf("circle_time=%u", isix::get_jiffies() - tbeg );
		tbeg = isix::get_jiffies();
		gdi.draw_ellipse( 50, 150, 30, 40 );
		dbprintf("ellipse_time=%u", isix::get_jiffies() - tbeg );
		dbprintf("text_width=%u", gdi.get_text_width(txt));
		tbeg = isix::get_jiffies();
		gdi.set_fg_color( gfx::rgb(255,0,0) );
		gdi.set_bg_color( gfx::rgb(0,0,0) );
		dbprintf("drawimage_time=%u", isix::get_jiffies() - tbeg );
	}

}

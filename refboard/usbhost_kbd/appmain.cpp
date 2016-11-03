/*
 * =====================================================================================
 *
 *       Filename:  appmain.cpp
 *
 *    Description:  Application startup 
 *
 *        Version:  1.0
 *        Created:  25.08.2016 21:49:00
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <config.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <stm32gpio.h>
#include <isix.h>
#include <usb/host/internal.h>
#include <usb/host/controller.h>
#include <usb/drivers/hostdev/hid_keyboard.h>
#include <usb/drivers/hostdev/hid_joystick.h>
#include <algorithm>

static const auto LED_PORT = GPIOG;

#ifdef PDEBUG
namespace {
namespace usart_debug {
	isix::semaphore m_ulock_sem { 1, 1 };
	void lock() 
	{
		m_ulock_sem.wait( ISIX_TIME_INFINITE );
	}
	void unlock() 
	{
		m_ulock_sem.signal();
	}
}}
#endif





void pulse_test(void* ) {
	using namespace stm32;
	gpio_config_ext( LED_PORT, 0xf0, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_2MHZ );
	// Test the APP environment
	 for(int r=0;;r++) {
		gpio_set_clr_mask( LED_PORT, r, 0x20 );
		gpio_set_clr_mask( LED_PORT, ~gpio_get_mask(GPIOB,0xC0), 0xc0 );
		gpio_set_clr_mask( LED_PORT, gpio_get(GPIOE,0)?1<<4:0, 0x10 );
		isix::wait_ms( 10 );
	}
}


/* ------------------------------------------------------------------ */
//Anonymous namespace for special keyboard handler
namespace {
	void  kbd_connected( const usbh_keyb_hid_context_t* id ) {
		dbprintf("Keyb ID %p connected", id );
	}
	void  kbd_disconnected( const usbh_keyb_hid_context_t* id ) {
		dbprintf("Keyb ID %p disconnected", id );
	}
	void kbd_report( const usbh_keyb_hid_context_t* , const usbh_keyb_hid_event_t* evt ) {
		if( evt->key ) {
			fnd::tiny_printf("%c", evt->key );
		} else {
			//dbprintf("S %i %02x", evt->scan_code, evt->scan_code );
		}
	}

	void kbd_desc( const usbh_keyb_hid_context_t* id, usbh_driver_desc_type desc, const char *str ) {
		dbprintf("ID %p Desc %i str: %s", id, desc, str );
	}
	constexpr usbh_hid_kbd_ops kbd_fops = {
		kbd_connected,
		kbd_disconnected,
		kbd_report,
		kbd_desc
	};
}
/* ------------------------------------------------------------------ */ 
namespace {
	void joy_connected( const usbh_hid_joy_context_t* id ) {
		dbprintf("Joy ID %p, connected, " , id);
	}
	void joy_disconnected( const usbh_hid_joy_context_t* id ) {
		dbprintf("Joy ID %p, disconnected, " , id );
	}
	void joy_desc( const usbh_hid_joy_context_t* id, usbh_driver_desc_type desc, const char *str ) {
		dbprintf("Joy ID %p Desc %i str: %s", id, desc, str );
	}
	void joy_report( const usbh_hid_joy_context_t *id, const usbh_joy_hid_event_t* evt ) {
		dbprintf("Joy ID %p events", id );
		if( evt->has.X ) {
			dbprintf("X_pos=%u", evt->X );
		}
		if( evt->has.Y ) {
			dbprintf("Y_pos=%u", evt->Y );
		}
		if( evt->has.Z ) {
			dbprintf("Z_pos=%u", evt->Z );
		}
		if( evt->has.rX ) {
			dbprintf("rX_pos=%u", evt->rX );
		}
		if( evt->has.rY) {
			dbprintf("rY_pos=%u", evt->rY );
		}
		if( evt->has.rZ ) {
			dbprintf("rZ_pos=%u", evt->rZ );
		}
		if( evt->has.hat ) {
			dbprintf("hat_pos=%u", evt->hat );
		}
		if( evt->has.slider ) {
			dbprintf("slider_pos=%u", evt->slider );
		}
		if( evt->n_buttons > 0 ) {
			dbprintf("got %u buttons val %x", evt->n_buttons, evt->buttons );
		}
	}
	constexpr usbh_hid_joystick_ops joy_ops = {
		joy_connected,
		joy_disconnected,
		joy_report,
		joy_desc,
	};
}
/* ------------------------------------------------------------------ */

int main() {
	isix::wait_ms( 500 );
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	// test
	dbprintf("USBHost app started OK");
	isix::task_create( pulse_test, nullptr, 256, isix::get_min_priority(), 0 );
	usbh_controller_init(USB_PHY_A, 2);
	usbh_controller_attach_driver( usbh_hid_keyboard_init(&kbd_fops) );
	usbh_controller_attach_driver( usbh_hid_joystick_init(&joy_ops) );
	dbprintf("Joystick and KBD initialized and completed");
	isix::start_scheduler();
	return 0;
}



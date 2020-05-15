/*================================================================================
 *
 *    Filename : thread.cpp
 *        Date : Wed May 14 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm creates two threads using C++ ISIX API. One of thread
 *               blinks LD3 with frequency 2Hz, when the latter suspends and
 *               resumes first thread in raction to USER button press.
 *
 *===============================================================================*/


#include <config/conf.h>                        // (ISIX) : base configuration
#include <isix.h>                               //  ISIX  : system modules
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <stm32_ll_bus.h>                       // LL     : clocks enabling


namespace {

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 100;

    // I/O pins aliases
    constexpr auto LD3    = periph::gpio::num::PD13;
    constexpr auto BUTTON = periph::gpio::num::PA0;

}

namespace app {

    void blink() {
        while(true) {

            // Blink LED (1Hz)
            isix::wait_ms(500);
            periph::gpio::toggle(LD3);

        }
    }

    void supervisor() {

        // State of the button (true = pressed)
        static bool pressed = false;

        // Create tasks
        isix::thread blink_h =
            isix::thread_create_and_run(
                1536, 
                isix::get_min_priority(),
                isix_task_flag_suspended,
                app::blink
            );

        // Display counter on LEDs        
        while(true){

            // Check if debounce is active
            if(periph::gpio::get(BUTTON)){

                if(!pressed){

                    // Wait for input to stabilize
                    isix::wait_ms(DEBOUNCE_MS);

                    // Check button state
                    if(periph::gpio::get(BUTTON)){
                        if(!pressed){
                            
                            pressed = true;
                            auto state = blink_h.get_state();

                            // If thread is suspended -> resume
                            if(state == OSTHR_STATE_SUSPEND){
                                blink_h.resume();
                                dbprintf("Thread resumed");
                            }
                            // If thread is running -> suspend
                            else if(state == OSTHR_STATE_RUNNING || state == OSTHR_STATE_SLEEPING){
                                dbprintf("Thread suspended");
                                blink_h.suspend();
                            }
                            // If threas is in a wrong state -> reboot
                            else{
                                dbprintf("'blink' thread crashed (state : %i). Rebooting...", state);
                                isix::wait_ms(500);
                                isix_reboot();
                            }

                        }        
                    }
                    else
                        pressed = false;
                }
            }
            else
                pressed = false;
        }
    }

}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Initialize debug logger
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	dblog_init_locked(
		[](int ch, void*) {
			return periph::drivers::uart_early::putc(ch);
		},
		nullptr,
		[]() {
			m_ulock_sem.wait(ISIX_TIME_INFINITE);
		},
		[]() {
			m_ulock_sem.signal();
		},
		periph::drivers::uart_early::open,
		"serial0", 115200
	);

    // Configure LD3 pin as an output
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    periph::gpio::setup( 
        LD3,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );

    // Configure BUTTON pin as an input
    periph::gpio::setup( 
        BUTTON,
        periph::gpio::mode::in{
            periph::gpio::pulltype::floating
        }
    );

    // Create supervisor task
    static auto supervisor_h = 
        isix::thread_create_and_run(
            1536,
            isix::get_min_priority(),
            0,
            app::supervisor
        );

    // Send welcome message to the log (UART)
    dbprintf("");
    dbprintf("");
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
    dbprintf("");
    dbprintf("");
    fnd::tiny_printf("\r\n");

    // Begin scheduling
    isix::start_scheduler();

	return 0;
}

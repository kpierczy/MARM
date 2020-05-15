/*================================================================================
 *
 *    Filename : semaphore.cpp
 *        Date : Wed May 14 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm creates two threads using C++ ISIX API. One of thread
 *               reads state of the USER button (software debouncing), when the
 *               seconds blocks on the semafore waiting for button state change 
 *               notification.
 *               When the button state changes, the first thread signals
 *               semaphore and the firs thread can read state of the button
 *               (saved in the global variable).
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

    // Variable to switch
    bool pressed = false;
    // Semaphore connected to the variable
    isix::semaphore semaphore{1, 1};

}

namespace app {

    void reader() {
        while(true) {
            
            // Wait for data to arrive
            semaphore.wait(ISIX_TIME_INFINITE);
            
            // Update LD3 state
            periph::gpio::set(LD3, pressed);
            dbprintf("LED3 switched");

        }
    }

    void writer() {


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
                            // Change data and signal to the reader                            
                            pressed = true;
                            semaphore.signal();
                        }        
                    }
                    else{
                        // Change data and signal to the reader
                        pressed = false;
                        semaphore.signal();
                    }
                }
            }
            else if(pressed){
                // Change data and signal to the reader
                pressed = false;
                semaphore.signal();
            }
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

    // Create writing task
    static auto writer_h = 
        isix::thread_create_and_run(
            1536,
            isix::get_min_priority(),
            0,
            app::writer
        );

    // Create reading task
    static isix::thread reader_h =
        isix::thread_create_and_run(
            1536, 
            isix::get_min_priority(),
            0,
            app::reader
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

/*================================================================================
 *
 *    Filename : fifo.cpp
 *        Date : Wed May 15 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Program creates 4 reading and 5 writing threads. Writing 
 *               threads monitors buttons on pins C0-C3 (active-low) and
 *               USER button (active-high). When change of a state of the
 *               button is ntoted, threads set their coresponding flags
 *               in the bit-event object.
 * 
 *               Reading threads wait for setting particular bits in the
 *               bit-event object. When the thread gets a signal about
 *               bit / bit combination setting it toggle the LED that it
 *               drives. Programmed combinations are:
 *          
 *                 (1)      PC0     <-> LD3
 *                 (2)      PC1     <-> LD4
 *                 (3)      PC2     <-> LD5
 *                 (4) PC3 & BUTTON <-> LD6
 * 
 *===============================================================================*/


#include <config/conf.h>                        // (ISIX) : base configuration
#include <isix.h>                               //  ISIX  : system modules
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <periph/clock/clocks.hpp>              // PERIPH : clocks enabling
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <stm32_ll_bus.h>                       // LL     : clocks enabling
#include <stm32_ll_exti.h>                      // LL     : EXTI controller
#include <stm32_ll_system.h>                    // LL     : EXTI macros


namespace {

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 100;

    // LEDs' aliases
    constexpr unsigned char LD[] = {
        periph::gpio::num::PD13,
        periph::gpio::num::PD12,
        periph::gpio::num::PD14,
        periph::gpio::num::PD15
    };
    
    // Buttons' aliases
    constexpr unsigned char K[] = {
        periph::gpio::num::PC0,
        periph::gpio::num::PC1,
        periph::gpio::num::PC2,
        periph::gpio::num::PC3,
        periph::gpio::num::PA0
    };

    // BitEvent object used as communication channel between threads
    isix::event event;

    // GPIO ports configuration
    void GPIO_config(){

        // Configure LEDs
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        periph::gpio::setup( 
            {LD[0], LD[1], LD[2], LD[3]},
            periph::gpio::mode::out{
                periph::gpio::outtype::pushpull,
                periph::gpio::speed::low
            }
        );

        // Configure buttons
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        periph::gpio::setup( 
            {K[0], K[1], K[2], K[3]},
            periph::gpio::mode::in{
                periph::gpio::pulltype::up
            }
        );
        periph::gpio::setup( 
            K[4],
            periph::gpio::mode::in{
                periph::gpio::pulltype::floating
            }
        );
    }

}

namespace app {

    void reader(const unsigned char LD_num){

        while(true){

            // Wait for flag set
            switch(LD_num){
                case 0:
                    event.wait(0x1, true, true);
                    break;
                case 1:
                    event.wait(0x2, true, true);
                    break;
                case 2:
                    event.wait(0x4, true, true);
                    break;
                case 3:
                    event.wait(0x18, true, true);  
                    break;
            }

            // Toggle LED
            periph::gpio::toggle(LD[LD_num]);
            dbprintf("LD%i set to %i", LD_num + 3, periph::gpio::get(LD[LD_num]));

        }
    }

    void writer(const unsigned char K_num) {

        volatile bool pressed = false;

        // Display counter on LEDs        
        while(true){

            // Check if debounce is active
            if(!periph::gpio::get(K[K_num])){

                if(!pressed){

                    // Wait for input to stabilize
                    isix::wait_ms(DEBOUNCE_MS);

                    // Check button state
                    if(!periph::gpio::get(K[K_num])){
                        if(!pressed){
                            
                            // Note button as pressed & send notification
                            pressed = true;
                            switch(K_num){
                                case 0:
                                    event.set(0x1);
                                    break;
                                case 1:
                                    event.set(0x2);
                                    break;
                                case 2:
                                    event.set(0x4);
                                    break;
                                case 3:
                                    event.set(0x8);
                                    break;
                                case 4:
                                    event.set(0x10);
                                    break;
                            }
                            dbprintf("K%i pressed", K_num);

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

    // Configure GPIOS
    GPIO_config();

    // Create writing theads
    static isix::thread writer1_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::writer, 0);
    static isix::thread writer2_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::writer, 1);
    static isix::thread writer3_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::writer, 2);
    static isix::thread writer4_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::writer, 3);
    static isix::thread writer5_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::writer, 4);

    // Create reading theads
    static isix::thread reader1_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::reader, 0);
    static isix::thread reader2_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::reader, 1);
    static isix::thread reader3_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::reader, 2);
    static isix::thread reader4_h =
        isix::thread_create_and_run(1536, isix::get_min_priority(), 0, app::reader, 3);

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

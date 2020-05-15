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
 * Description : Program creates a thread that reads 1-element FIFO queue. The
 *               queue is sourced with ISR that handles EXTI interrupts 
 *               triggered by the change of a state of the USER button. The
 *               thread updates LD3 state accoring to got message.
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

    // I/O pins aliases
    constexpr auto LD3    = periph::gpio::num::PD13;
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // FIFO used as communication channel between thread and ISR
    isix::fifo<bool> fifo{1};


    // EXTI interrupts config
    void EXTI_config(){

        /**
         * EXTI module configuration
         * 
         * 1) Active CLK for SYSFG module
         * 2) Configure PORTA.0 to source EXTI line 0
         * 3) Configure EXTI module
        */
        periph::clock::device_enable(
            periph::dt::clk_periph{
                .xbus = periph::dt::bus::apb2,
                .bit = RCC_APB2ENR_SYSCFGEN_Pos
            }
        );
        LL_SYSCFG_SetEXTISource(
            LL_SYSCFG_EXTI_PORTA,
            LL_SYSCFG_EXTI_LINE0
        );
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
        LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
        LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

        // Enabling EXTI interrupts in NVIC controller
        isix::set_irq_priority(EXTI0_IRQn , {1, 7});
        isix::request_irq(EXTI0_IRQn);
    }

}

namespace app {

    void reader() {

        static bool pressed = false;

        while(true) {
            
            // Wait for data to arrive
            fifo.pop(pressed, ISIX_TIME_INFINITE);
            
            // Update LD3 state
            periph::gpio::set(LD3, pressed);
            dbprintf("LED3 switched");

        }
    }

}

extern "C" {

    void exti0_isr_vector() {

        // Clear interrupt flag
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

        // Set debounce indicator
        fifo.push_isr(periph::gpio::get(BUTTON));

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

    // Configure EXTI
    EXTI_config();

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

#ifdef TIMER_INTERRUPT

#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>

#include <stm32_ll_bus.h>
#include <stm32_ll_tim.h>

namespace {

    // Counter that indicates number of the actually turned-on LED
    volatile int led_counter = 0;
    
    // Display LEDs
    constexpr auto led_3 = periph::gpio::num::PD13;
    constexpr auto led_4 = periph::gpio::num::PD12;
    constexpr auto led_5 = periph::gpio::num::PD14;
    constexpr auto led_6 = periph::gpio::num::PD15;

    // Timer1 configuration for periodic interrupts
    void TIM1_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

        // Set counter mode (not obligatory - UP mode set by default)
        LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);

        /* Configure timer period (Prescaler + AutoReloadRegister [ARR])
         *
         *  - TIM1 CLK : 1kHz
         *  - TIM2 OVF : 5Hz
        */
        LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(100000000 , 1000));
        LL_TIM_SetAutoReload(TIM1, __LL_TIM_CALC_ARR(100000000 , LL_TIM_GetPrescaler(TIM1), 5));

        // Configure OVF (update) interrupts
        LL_TIM_EnableIT_UPDATE(TIM1);
        isix_irq_prio_t prior{
            .prio = 0,
            .subp = 0
        };
        isix_set_irq_priority(TIM1_UP_TIM10_IRQn, prior);
        isix::request_irq(TIM1_UP_TIM10_IRQn);
        
        // Enable TIM1
        LL_TIM_EnableCounter(TIM1);

        /* Trigger the first update event by hand
         *
         * @note Update event doesn't trigger interrup / DMA transfer
         *       every time (only if RepetitionCounter is set to 0).
         *       It triggers loading data from timer's shadow registers
         *       though. To start timer properly Update event should be
         *       generated to clean rubbish from the registers and load
         *       them with desired values
        */
        LL_TIM_GenerateEvent_UPDATE(TIM1);
    }

    // Thread functions
    void main_thread(void*){

        constexpr char const* colour[4] = {
            "orange",
            "red",
            "blue",
            "green"
        };

        while (true){
            isix::wait_ms( 200 );
            dbprintf("Actual LED: %s", colour[led_counter]);
        }
    }
}

extern "C" {

    void tim1_up_tim10_isr_vector(void) {

        switch (led_counter){
            // LD3 (orange)
            case 0:
                periph::gpio::set(led_4, false);
                periph::gpio::set(led_3, true);
                break;
            // LD5 (red)
            case 1:
                periph::gpio::set(led_3, false);
                periph::gpio::set(led_5, true);
                break;
            // LD6 (blue)
            case 2:
                periph::gpio::set(led_5, false);
                periph::gpio::set(led_6, true);
                break;
            // LD4 (green)
            case 3:
                periph::gpio::set(led_6, false);
                periph::gpio::set(led_4, true);
                break;
        }

        // Increment counter (modulo 4)
        ++led_counter %= 4;

        // Clear interrupt flag
        isix_clear_irq_pending(TIM1_UP_TIM10_IRQn);
    }
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Initialize debug logger
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

    // Configure LEDs
    periph::gpio::setup( 
        {led_3, led_4, led_5, led_6},
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );

    // Configure TIM1 timebase
    TIM1_config();

    // Create threads
	isix::task_create(main_thread, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

// TIMER_INTERRUPT
#endif
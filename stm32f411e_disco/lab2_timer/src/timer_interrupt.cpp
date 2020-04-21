/**
 * Program's model version
 *  - ISIX
 *  - LL
 */
#define ISIX

/**
 * Frequency in range [16; 10 000] HZ
 */
#define LED_FREQ 10

#include <config/conf.h> // ISIX base configuration
#include <foundation/sys/dbglog.h> // Logging module
#include <periph/drivers/serial/uart_early.hpp> // UART module used by logging module

#include <isix.h> // ISIX system modules
#include <isix/arch/irq.h> // ISR symbols
#include <periph/clock/clocks.hpp> // Peripherals enabling
#include <periph/gpio/gpio.hpp> // GPIO framewoek

#include <stm32_ll_bus.h> // Peripherals clocking
#include <stm32_ll_rcc.h> // CLK & Reset module
#include <stm32_ll_system.h> // Flash latency
#include <stm32_ll_tim.h> // TIMx modules

namespace {

    // Counter that indicates number of the actually turned-on LED
    volatile int led_counter = 0;
    
    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;

     // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOD
        periph::clock::device_enable(
            periph::dt::clk_periph{
                .xbus = periph::dt::bus::ahb1,
                .bit = RCC_AHB1ENR_GPIODEN_Pos
            }
        );

        // Configure LEDs
        periph::gpio::setup( 
            {LED3, LED4, LED5, LED6},
            periph::gpio::mode::out{
                periph::gpio::outtype::pushpull,
                periph::gpio::speed::low
            }
        );
    }

    // Timer1 configuration for periodic interrupts
    void TIM1_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

        /**
         * TIM1 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + TIM1 DK_CNT : 10kHz
         *      + TIM2 OVF    : 5Hz
         */
        LL_TIM_InitTypeDef TIM1_struct{
            .Prescaler         = __LL_TIM_CALC_PSC(100000000 , 10000),
            .CounterMode       = LL_TIM_COUNTERMODE_UP,
            .Autoreload        = __LL_TIM_CALC_ARR(100000000 , __LL_TIM_CALC_PSC(100000000 , 10000), LED_FREQ),
            .RepetitionCounter = 0
        };
        LL_TIM_Init(TIM1, &TIM1_struct);

        // Enable OVF (update) interrupts
        LL_TIM_EnableIT_UPDATE(TIM1);

        // Active TIM1 interrupt in NVIC module
        isix::set_irq_priority(
            TIM1_UP_TIM10_IRQn,
            isix_irq_prio_t{
                .prio = 1,
                .subp = 7
            }
        );
        isix::request_irq(TIM1_UP_TIM10_IRQn);
        
        // Enable TIM1
        LL_TIM_EnableCounter(TIM1);

        /* Trigger the first update event by hand
         *
         * @note Update event triggers every 'RepetitionCounter' times.
         *       It also triggers loading data from timer's shadow registers.
         *       To start timer properly Update event should be
         *       generated to clean rubbish from the registers and load
         *       them with desired values
        */
        LL_TIM_GenerateEvent_UPDATE(TIM1);
    }

    // Thread functions
    void main_thread(void*){

        char const* colour[4] = {
            "orange",
            "red",
            "blue",
            "green"
        };

        while (true){
            isix::wait_us( 1000000 / LED_FREQ );
            dbprintf("Actual LED: %s", colour[led_counter]);
        }
    }
}

extern "C" {

    /**
     * TIM1 "Update" Interrupt Service Routine
     * 
     * @note TIM1 module's update flag is NOT automatically
     *       cleared after entering ISR. If this flag is
     *       active it activates a corresponding flag in the
     *       NVIC controller
     * 
     *         --------------------------------------
     *         |  Flags in the NVIC controller ARE  |
     *         |  cleared automaitcally during ISR  |
     *         |  servicing                         |
     *         --------------------------------------
     * 
     *      NVIC's flag activation is performed every bus cycle
     *      when peripheral's flag is active. It is important
     *      to clear peripheral's flag as soon as possible
     *      after entering ISR.
     * 
     *          ----------------------------------------
     *         |  If one clears the peripheral's flag  |
     *         |  to late, it is possible that there   |
     *         |  will be no a new bus cycle before    |
     *         |  leaving ISR, and NVIC flag will be   |
     *         |  activated one more time with the old |
     *         |  value of the peripheral's flag.      |
     *         -----------------------------------------
     *          
     */
    void tim1_up_tim10_isr_vector(void) {

        /**
         *  Clear interrupt flag
         * 
         *  @note Despite vector beeing shared between
         *        two interrupt sources, the TIM1U flag
         *        is not checked before clearing, as
         *        the TIM10 module is not used
         */
        LL_TIM_ClearFlag_UPDATE(TIM1);

        // Increment counter (modulo 4)
        ++led_counter %= 4;

        switch (led_counter){
            // LD3 (orange)
            case 0:
                periph::gpio::set(LED4, false);
                periph::gpio::set(LED3, true);
                break;
            // LD5 (red)
            case 1:
                periph::gpio::set(LED3, false);
                periph::gpio::set(LED5, true);
                break;
            // LD6 (blue)
            case 2:
                periph::gpio::set(LED5, false);
                periph::gpio::set(LED6, true);
                break;
            // LD4 (green)
            case 3:
                periph::gpio::set(LED6, false);
                periph::gpio::set(LED4, true);
                break;
        }
    }
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Enable GPIOA for USART2 alternate function
    LL_AHB1_GRP1_EnableClock(
        LL_AHB1_GRP1_PERIPH_GPIOA
    );

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
    GPIO_config();

    // Configure TIM1 timebase
    TIM1_config();

    // Create threads
	isix::task_create(main_thread, nullptr, 1536, isix::get_min_priority() );

    // Send welcome message to the log (UART)
    dbprintf("");
    dbprintf("");
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
    dbprintf("");
    dbprintf("");

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

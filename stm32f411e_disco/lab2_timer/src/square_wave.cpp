/**
 * Program's model version
 *  - ISIX
 *  - LL
 */
#define LL

#include <config/conf.h> // ISIX base configuration
#include <foundation/sys/dbglog.h> // Logging module
#include <periph/drivers/serial/uart_early.hpp> // UART module used by logging module

#include <isix.h> // ISIX system modules
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
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        
        /*
         * Configure LEDs
         * 
         * @note : alternate function 
         *                 =
         *             TIM4_CHx     
         */
        periph::gpio::setup( 
            {LED3, LED4, LED5, LED6},
            periph::gpio::mode::alt{
                periph::gpio::outtype::pushpull,
                2,
                periph::gpio::speed::low
            }
        );
    }

    /**
     *  Timer2 configuration for PWM (50% duty cycle)
     *  driving LEDs.
     */
    void TIM4_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM4);

        /**
         * TIM4 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + TIM1 DK_CNT : 10kHz
         *      + TIM4 OVF    : 2Hz
         */
        LL_TIM_InitTypeDef TIM4_struct{
            .Prescaler         = __LL_TIM_CALC_PSC(100000000 , 10000),
            .Autoreload        = __LL_TIM_CALC_ARR(100000000 , __LL_TIM_CALC_PSC(100000000 , 10000), 2)
        };
        LL_TIM_Init(TIM4, &TIM4_struct);

        /**
         * TIM4 compare/capture configuration structures
         *     - Frequency : 1Hz
         *     - Subsequent LEDs have got a phase
         *       shifted by the 90 degrees :
         *          + LED3 : 0   degree
         *          + LED5 : 90  degree
         *          + LED6 : 180 degree
         *          + LED4 : 270 degree
         *     - CCRx buffered
         */
        LL_TIM_OC_InitTypeDef TIM4_CC_struct{
            .OCMode       = LL_TIM_OCMODE_TOGGLE,
            .OCState      = LL_TIM_OCSTATE_ENABLE
        };
        // Channel_2 = LED3
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_LOW,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM4_CC_struct);
        // Channel_2 = LED5
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = (uint32_t)(TIM4_struct.Autoreload / 2);
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM4_CC_struct);
        // Channel_2 = LED6
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM4_CC_struct);
        // Channel_2 = LED4
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_LOW,
        TIM4_CC_struct.CompareValue = (uint32_t)(TIM4_struct.Autoreload / 2);
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM4_CC_struct);
        
        // Enable TIM4
        LL_TIM_EnableCounter(TIM4);

        /* Trigger the first update event by hand
         *
         * @note Update event triggers every 'RepetitionCounter' times.
         *       It also triggers loading data from timer's shadow registers.
         *       To start timer properly Update event should be
         *       generated to clean rubbish from the registers and load
         *       them with desired values
        */
        LL_TIM_GenerateEvent_UPDATE(TIM4);
    }

    // Thread functions
    void main_thread(void*){
        while (true){
            isix::wait_ms( 500 );
            dbprintf("Main loop's alive!");
        }
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
    GPIO_config();

    // Configure TIM1 timebase
    TIM4_config();

    // Create threads
	isix::task_create(main_thread, nullptr, 1536, isix::get_min_priority() );

    // Send welcome message to the log (UART)
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

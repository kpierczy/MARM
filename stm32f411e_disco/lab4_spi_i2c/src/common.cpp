/*================================================================================
 *
 *    Filename : common.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Functions common to both versions of the programm
 *
 *===============================================================================*/

#include <config/conf.h>                        // ISIX       : Base configuration
#include <isix.h>                               // ISIX       : System modules
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <foundation/sys/tiny_printf.h>         // FOUNDATION : printf
#include <periph/drivers/serial/uart_early.hpp> // PERIPH     : UART controller used by logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH     : GPIO framework
#include <periph/clock/clocks.hpp>              // PERIPH     : Clocks enabling
#include <stm32_ll_bus.h>                       // LL         : clock enable bit masks
#include <stm32_ll_tim.h>                       // LL         : timers API

void gyro(void*);

namespace{
    
    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;


    /**
     *  Timer4 configuration for PWM (variable duty cycle)
     *  driving LEDs.
     */
    void TIM4_config(void){

        // Enable clocks for GPIOs (A & D)
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        
        // Configure LEDs to alternate function (TIM4_CHx)
        periph::gpio::setup( 
            {LED3, LED4, LED5, LED6},
            periph::gpio::mode::alt{
                periph::gpio::outtype::pushpull,
                2,
                periph::gpio::speed::low
            }
        );

        // Enable clock from APB1 for the TIM4 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM4);

        /**
         * TIM4 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + CK_CNT : 100 kHz
         *      + OVF    : 200 Hz
         */
        LL_TIM_InitTypeDef TIM4_struct{
            .Prescaler         = __LL_TIM_CALC_PSC(100000000 , 100000),
            .Autoreload        = 499
        };
        LL_TIM_Init(TIM4, &TIM4_struct);

        /**
         * TIM4 compare/capture configuration structures
         *     - Subsequent LEDs have got a duty
         *        cycle changed :
         *          + LED3 : 20 %
         *          + LED5 : 40 %
         *          + LED6 : 60 %
         *          + LED4 : 0 - 100 % 
         *     - Duty cycle of the LED4 changes by 10% every
         *       single USER button press (UP and DOWN)
         *     - CCRx buffered
         */
        LL_TIM_OC_InitTypeDef TIM4_CC_struct{
            .OCMode       = LL_TIM_OCMODE_PWM1,
            .OCState      = LL_TIM_OCSTATE_ENABLE
        };
        // Channel_2 = LED3
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM4_CC_struct);
        // Channel_2 = LED5
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM4_CC_struct);
        // Channel_2 = LED6
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM4_CC_struct);
        // Channel_2 = LED4
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM4_CC_struct);
        
        // Enable TIM4
        LL_TIM_EnableCounter(TIM4);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM4);
    }
}




auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Enable GPIOA for USART2 alternate function
    periph::clock::device_enable(
        periph::dt::clk_periph{
            .xbus = periph::dt::bus::ahb1,
            .bit  = RCC_AHB1ENR_GPIOAEN_Pos
        }
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

    // Configure TIM4 PWM for LEDs driving
    TIM4_config();

    // Create threads
	isix::task_create(gyro, nullptr, 1536, isix::get_min_priority() );

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
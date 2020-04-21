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
#include <isix/arch/irq.h> // ISIX interrupts
#include <periph/gpio/gpio.hpp> // GPIO framewoek


#include <stm32_ll_bus.h> // Peripherals clocking
#include <stm32_ll_exti.h> // EXTI module
#include <stm32_ll_rcc.h> // CLK & Reset module
#include <stm32_ll_system.h> // Flash latency
#include <stm32_ll_tim.h> // TIMx modules

namespace {

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 200;

    // Debouncing indicator
    volatile bool debounce_active = false;

    // Counter that indicates number of the actually turned-on LED
    volatile int led_counter = 0;
    
    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

     // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOs (A & D)
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
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

        // Configure button
        periph::gpio::setup( 
            BUTTON,
            periph::gpio::mode::in{
                periph::gpio::pulltype::floating
            }
        );
    }

    // EXTI interrupts config
    void EXTI_config(){

        /**
         * EXTI module configuration
         * 
         * 1) Active CLK for SYSFG module
         * 2) Configure PORTA.0 to source EXTI line 0
         * 3) Configure EXTI module
        */
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
        LL_SYSCFG_SetEXTISource(
            LL_SYSCFG_EXTI_PORTA,
            LL_SYSCFG_EXTI_LINE0
        );
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
        LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

        // Enabling EXTI interrupts in NVIC controller
        isix::set_irq_priority(EXTI0_IRQn , {1, 7});
        isix::request_irq(EXTI0_IRQn);
    }

/* 
     * TIM1 configuration for periodic interrupts (1Hz)
     */
    void TIM1_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

        /**
         * TIM1 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + TIM1 DK_CNT : 10kHz
         *      + TIM2 OVF    : 1Hz
         */
        LL_TIM_InitTypeDef TIM1_struct{
            .Prescaler         = __LL_TIM_CALC_PSC(100000000 , 10000),
            .Autoreload        = __LL_TIM_CALC_ARR(100000000 , __LL_TIM_CALC_PSC(100000000 , 10000), 1)
        };
        LL_TIM_Init(TIM1, &TIM1_struct);

        // Enable OVF (update) interrupts
        LL_TIM_EnableIT_UPDATE(TIM1);
        // Active TIM1 interrupt in NVIC module
        isix::set_irq_priority(TIM1_UP_TIM10_IRQn, {1, 7});
        isix::request_irq(TIM1_UP_TIM10_IRQn);
        
        // Enable TIM1
        LL_TIM_EnableCounter(TIM1);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM1);
    }

    /**
     *  Timer4 configuration for PWM (variable duty cycle)
     *  driving LEDs.
     */
    void TIM4_config(void){

        // Enable clock from APB1 for the TIM4 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM4);

        /**
         * TIM4 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + DK_CNT : 20 kHz
         *      + OVF    : 199.599 Hz
         */
        LL_TIM_InitTypeDef TIM4_struct{
            .Prescaler         = __LL_TIM_CALC_PSC(100000000 , 100000),
            .Autoreload        = 500
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
        TIM4_CC_struct.CompareValue = 200;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM4_CC_struct);
        // Channel_2 = LED5
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 300;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM4_CC_struct);
        // Channel_2 = LED6
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 0;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM4_CC_struct);
        // Channel_2 = LED4
        TIM4_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM4_CC_struct.CompareValue = 100;
        LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM4_CC_struct);
        
        // Enable TIM4
        LL_TIM_EnableCounter(TIM4);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM4);
    }

    // Thread functions
    void main_thread(void*){

        // State of the button (true = pressed)
        static bool button_pressed = false;

        /* Direction of the LED6's brightness changes
         * triggered by the button press
         *
         *     - 0 : increases
         *     - 1 : decreases
         */
        static bool direction = 0;
                
        while (true){

            // Check if debounce is active
            if(debounce_active){

                // Wait for input to stabilize
                isix::wait_ms(DEBOUNCE_MS);

                // Check button state
                if(periph::gpio::get(BUTTON)){
                    if(!button_pressed){
                        // Succesfull debouncing
                        button_pressed = true;
                        // Increment LED4's duty cycle by 10%
                        if(direction == 0){
                            TIM4->CCR4 += 50;
                            if(TIM4->CCR4 == 500)
                                direction = 1;
                        }
                        else{
                            TIM4->CCR4 -= 50;
                            if(TIM4->CCR4 == 0)
                                direction = 0;
                        }
                    }        
                }
                else
                    button_pressed = false;
            }
        }
    }
}

extern "C" {

    void exti0_isr_vector() {

        // Clear interrupt flag
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

        // Set debounce indicator
        debounce_active = true;
    }

    /**
     * Periodical interrupt (1Hz) printing actually
     * values of TIM4 C/C 1 registers
     */
    void tim1_up_tim10_isr_vector(void) {

        // Clear interrupt flag
        LL_TIM_ClearFlag_UPDATE(TIM1);

        // Print info about TIM4 C/C1 registers
        dbprintf(
            "LED4: %i LED3: %i LED5: %i LED6: %3i ARR: %i PSC: %i",
            TIM4->CCR1, TIM4->CCR2,
            TIM4->CCR3, TIM4->CCR4,
            TIM4->ARR, TIM4->PSC
        );
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

    // Button interrupt config
    EXTI_config();

    // Configure TIM4 (PWM)
    TIM4_config();

    // Configure TIM1 timebase (periodic interrupt)
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

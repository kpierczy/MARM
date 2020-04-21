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
#include <isix/arch/irq.h> // ISR symbols
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

    // GPIOs declarations
    constexpr auto INPUT = periph::gpio::num::PA6;
    constexpr auto GENERATOR = periph::gpio::num::PA1;

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // Generator's frequency
    int frequencies_size = 3;
    uint32_t frequencies[] = {
        10000,
        100000,
        1000000
    };
    // Index of thee frequency of the generator
    volatile uint32_t freq_index = 0;

    // TIM3->CCR1 last taken copy
    volatile float period = 0;

    // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOD
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        
        // Configure LEDs to alternate function (TIM3_CH1 / TIM5_CH1)
        periph::gpio::setup( 
            {INPUT, GENERATOR},
            periph::gpio::mode::alt{
                .out   = periph::gpio::outtype::pushpull,
                .altno = 2,
                .spd = periph::gpio::speed::high
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
        isix::set_irq_priority(EXTI0_IRQn , {1, 0});
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
     * Configure TIM3 to measure period of the signal
     */
    void TIM3_config(){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM3);

        /**
         * TIM3 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + CK_CNT   : 100 MHz
         *      + OVF freq : 1.53 kHz
         */
        LL_TIM_InitTypeDef TIM3_struct{
            .Prescaler  = 0,
            .Autoreload = 0xffff
        };
        LL_TIM_Init(TIM3, &TIM3_struct);

        /**
         * TIM3 compare/capture configuration structures
         * 
         *     - Mode : input PWM
         *     - Sampling frequency : 100 MHz 
         *     - Detected edge : rising
         *     - Filter : none
         *     - Edges to detect (ICPSC) : 1
         *     - Observed input : TI1 (PA6)
         * 
         * TIM3 is set to the (reset) slave mode with respect
         * to TI1FP1 (TI1) signal.
         */
        LL_TIM_IC_InitTypeDef TIM3_IC_struct{
            .ICPolarity    = LL_TIM_IC_POLARITY_RISING,
            .ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
            .ICPrescaler   = LL_TIM_ICPSC_DIV1,
            .ICFilter      = LL_TIM_IC_FILTER_FDIV1
        };
        LL_TIM_IC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM3_IC_struct);
        LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);
        LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);

        // Enable C/C interrupts
        LL_TIM_EnableIT_CC1(TIM3);
        // Active TIM1 interrupt in NVIC module
        isix::set_irq_priority(TIM3_IRQn, {0, 0});
        isix::request_irq(TIM3_IRQn);

        // Enable channel 1
        LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
        
        // Enable TIM3
        LL_TIM_EnableCounter(TIM3);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM3);
    }


    /**
     *  TIM5 configuration for PWM generator
     */
    void TIM5_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM5);

        /**
         * TIM5 configuration structure
         *  - Mode : up
         *  - Frequencies:
         *      + DK_CNT   : 10 MHz
         *      + OVF freq : 10 kHz
         */
        LL_TIM_InitTypeDef TIM5_struct{
            .Prescaler  = __LL_TIM_CALC_PSC(100000000 , 10000000),
            .Autoreload = __LL_TIM_CALC_ARR(100000000 , __LL_TIM_CALC_PSC(100000000 , 10000000), 10000)
        };
        LL_TIM_Init(TIM5, &TIM5_struct);

        /**
         * TIM4 compare/capture configuration structures
         *     - Mode       : PWM1
         *     - Frequency  : 10 kHz
         *     - Duty Cycle : 50%
         *     - CCR1 buffered
         */
        LL_TIM_OC_InitTypeDef TIM5_CC_struct{
            .OCMode       = LL_TIM_OCMODE_PWM1,
            .OCState      = LL_TIM_OCSTATE_ENABLE
        };
        // Channel_2 (PA1)
        TIM5_CC_struct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH,
        TIM5_CC_struct.CompareValue = (TIM5->ARR >> 1);
        LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH2, &TIM5_CC_struct);
        
        // Enable TIM5
        LL_TIM_EnableCounter(TIM5);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM5);
    }

    // Thread functions
    void main_thread(void*){

        // State of the button (true = pressed)
        static bool button_pressed = false;
                
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

                        // Increment freuency index
                        ++freq_index %= frequencies_size;

                        // Introduce new frequency
                        TIM5->ARR =
                            __LL_TIM_CALC_ARR(100000000 , LL_TIM_GetPrescaler(TIM5), frequencies[freq_index]);
                        // Set duty cycle
                        TIM5->CCR2 =
                            TIM5->ARR >> 1;
                    }        
                }
                else
                    button_pressed = false;
            }
        }
    }
}

extern "C" {

    /**
     * Button debouncing interrupt
     */
    void exti0_isr_vector() {

        // Clear interrupt flag
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

        // Set debounce indicator
        debounce_active = true;
    }

    /**
     * Periodical interrupt (1Hz) printing actually
     * measured frequency to the dblog (UART)
     */
    void tim1_up_tim10_isr_vector(void) {

        // Clear interrupt flag
        LL_TIM_ClearFlag_UPDATE(TIM1);

        // Print measured frequency
        if(period != 0)
            dbprintf(
                "Measured period : %i Hz",
                (uint32_t)(100000000.0f / period)
            );
        else
            dbprintf(
                "Measured frequency: 0 Hz"
            );
        // Print measured frequency
        dbprintf(
            "Actual frequency: %i Hz",
            frequencies[freq_index]
        );
        dbprintf("");
    }

    void tim3_isr_vector(void)
    {
        // Clear interrupt flag
        LL_TIM_ClearFlag_CC1(TIM3);
        
        // Save period value
        period = (float)TIM3->CCR1;
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

    // EXTI configuration
    EXTI_config();

    // Configure TIM5 (Generator)
    TIM5_config();

    // Configure TIM3 (frequency measure)
    TIM3_config();

    // Configure TIM1 (measurements printing)
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

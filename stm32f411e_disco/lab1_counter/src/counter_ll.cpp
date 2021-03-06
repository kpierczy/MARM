/*================================================================================
 *
 *    Filename : counter_ll.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm counts USER button pushes using EXTI peripheral module.
 *               Four least significant bits of the counter are displayed using
 *               LED3-LED6 diodes. Programm debouncing procedure has been 
 *               implemented. Task was accomplished using basic LL functions.
 *
 *===============================================================================*/


#include <config/conf.h>                        // (ISIX) : base configuration
#include <isix.h>                               //  ISIX  : system modules
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <periph/clock/clocks.hpp>              // PERIPH : clocks enabling
#include <stm32_ll_exti.h>                      // LL : EXTI controller
#include <stm32_ll_rcc.h>                       // LL : Reset & Clock control
#include <stm32_ll_system.h>                    // LL : Flash latency
#include <stm32_ll_bus.h>                       // LL : bus API
#include <stm32_ll_gpio.h>                      // LL : gpios configuration

// Common 
namespace{

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 200;

    // Debouncing indicator
    volatile bool debounce_active = false;

    // Display LEDs
    constexpr auto LED3 = LL_GPIO_PIN_13;
    constexpr auto LED4 = LL_GPIO_PIN_12;
    constexpr auto LED5 = LL_GPIO_PIN_14;
    constexpr auto LED6 = LL_GPIO_PIN_15;

    // User button
    constexpr auto BUTTON = LL_GPIO_PIN_0;

    /**
     * Configures CLK sources as follows:
     *  - SYSCLK Source : PLLCLK
     *  - PLL Source : HSE
     *  - HSI : Disabled
     *  - HCLK : 100MHz
     *  - APB1 : 50MHz
     *  - APB2 : 100MHz
     */
    bool CLK_config()
    {
        // Number of retries while waiting for 
        // CLK modules to turn on/off
        constexpr auto RETRIES = 100000;
        
        // Deinitialize RCC
        LL_RCC_DeInit();

        // Configure ART module
        LL_FLASH_SetLatency( LL_FLASH_LATENCY_3 );
        LL_FLASH_EnablePrefetch();

        // Set MCU Prescallers
        LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
        LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
        LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );

        // Enable HSE generator
        LL_RCC_HSE_Enable();
        for( int r = 0; r < RETRIES; ++r ) {
            if(LL_RCC_HSE_IsReady()) {
                break;
            }
        }
        if( !LL_RCC_HSE_IsReady() ) {
            return false;
        }

        // Configure PLL
        LL_RCC_PLL_ConfigDomain_SYS(
            LL_RCC_PLLSOURCE_HSE,
            LL_RCC_PLLM_DIV_4,
            100,
            LL_RCC_PLLP_DIV_2
        );
        LL_RCC_PLL_Enable();
        for( auto r = 0; r < RETRIES; ++r ) {
            if( LL_RCC_PLL_IsReady() ) {
                break;
            }
        }
        if( !LL_RCC_PLL_IsReady() ) {
            return false;
        }

        // Set PLL as SYSCLK source
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        for( auto r = 0; r < RETRIES; ++r ) {
            if( LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {
                break;
            }
        }
        if(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){
            return false;
        }

        // Disable HSI generator
        LL_RCC_HSI_Disable();
        for( int r = 0; r < RETRIES; ++r ) {
            if(!LL_RCC_HSI_IsReady()) {
                break;
            }
        }

        return !LL_RCC_HSE_IsReady();
    }

    // GPIO ports config
    void GPIO_config(){

        // Enable clocks for GPIOS
        LL_AHB1_GRP1_EnableClock(
            LL_AHB1_GRP1_PERIPH_GPIOA |
            LL_AHB1_GRP1_PERIPH_GPIOD
        );

        // Configure LEDs
        LL_GPIO_InitTypeDef led_init_struct{
            .Mode = LL_GPIO_MODE_OUTPUT,
            .Speed = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL
        };
        led_init_struct.Pin = LED3;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LED4;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LED5;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LED6;
        LL_GPIO_Init(GPIOD, &led_init_struct);

        // Configure button
        LL_GPIO_InitTypeDef button_init_struct{
            .Pin = BUTTON,
            .Mode = LL_GPIO_MODE_INPUT,
            .Pull = LL_GPIO_PULL_NO
        };
        LL_GPIO_Init(GPIOA, &button_init_struct);
    }

    // EXTI interrupts config
    void EXTI_config(){

        // Interrupt configuration
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
        LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA , LL_SYSCFG_EXTI_LINE0);
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
        LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

        // Enabling EXTI irq in NVIC controller
        NVIC_SetPriority(EXTI0_IRQn , 0);
        NVIC_EnableIRQ(EXTI0_IRQn);

    }

    void button_counter(void*) {
        
        // State of the button (true = pressed)
        static bool button_pressed = false;

        // Button counter
        unsigned int counter = 0;


        // Display counter on LEDs        
        while(true){
            if(counter & (1U << 3))
                LL_GPIO_SetOutputPin(GPIOD, LED4);
            else
                LL_GPIO_ResetOutputPin(GPIOD, LED4);
            if(counter & (1U << 2))
                LL_GPIO_SetOutputPin(GPIOD, LED3);
            else
                LL_GPIO_ResetOutputPin(GPIOD, LED3);
            if(counter & (1U << 1))
                LL_GPIO_SetOutputPin(GPIOD, LED5);
            else
                LL_GPIO_ResetOutputPin(GPIOD, LED5);
            if(counter & (1U << 0))
                LL_GPIO_SetOutputPin(GPIOD, LED6);  
            else
                LL_GPIO_ResetOutputPin(GPIOD, LED6);


            // Check if debounce is active
            if(debounce_active){

                // Wait for input to stabilize
                isix::wait_ms(DEBOUNCE_MS);

                // Check button state
                if(LL_GPIO_IsInputPinSet(GPIOA, BUTTON)){
                    if(!button_pressed){
                        button_pressed = true;
                        ++counter;
                        dbprintf("Button pressed");
                        if(counter > 15)
                            counter = 0;
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

}





auto main() -> int
{
    // Initialize system semapher
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

    // Configure CLK distribution
    CLK_config();

    // GPIO configuration
    GPIO_config();

    // EXTI interrupts config
    EXTI_config();

    // Create programm task
	isix::task_create( button_counter, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

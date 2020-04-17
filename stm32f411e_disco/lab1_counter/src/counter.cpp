#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>

#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_exti.h>

/**
 * Program version
 *  (1) - isix
 *  (2) - LL
 *  (3) - LL with interrupt
 */
#define VERSION 1


namespace {

    // Debouncing time [ms]
    constexpr int debounce_ms = 200;

    // Display LEDs
    constexpr auto led_3 = periph::gpio::num::PD13;
    constexpr auto led_4 = periph::gpio::num::PD12;
    constexpr auto led_5 = periph::gpio::num::PD14;
    constexpr auto led_6 = periph::gpio::num::PD15;

    // User button
    constexpr auto button = periph::gpio::num::PA0;

#if VERSION == 3
    // Debouncing indicator
    volatile bool debounce_active = false;
#endif

    /**
     * Configures CLK sources as follows:
     *  - SYSCLK Source : PLLCLK
     *  - PLL Source : HSE
     *  - HSI : Disabled
     *  - HCLK : 100MHz
     *  - APB1 : 50MHz
     *  - APB2 : 100MHz
     */
    bool uc_periph_setup()
    {
        // Number of retries while waiting for 
        // CLK modules to turn on/off
        constexpr auto retries=100000;
        
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
        for( int i=0; i<retries; ++i ) {
            if(LL_RCC_HSE_IsReady()) {
                break;
            }
        }
        if( !LL_RCC_HSE_IsReady() ) {
            return false;
        }

        // Enable clocks for GPIOS
        LL_AHB1_GRP1_EnableClock(
            LL_AHB1_GRP1_PERIPH_GPIOA|LL_AHB1_GRP1_PERIPH_GPIOB|
            LL_AHB1_GRP1_PERIPH_GPIOC|LL_AHB1_GRP1_PERIPH_GPIOD|
            LL_AHB1_GRP1_PERIPH_GPIOE 
        );

        // Configure PLL
        LL_RCC_PLL_ConfigDomain_SYS(
            LL_RCC_PLLSOURCE_HSE,
            LL_RCC_PLLM_DIV_4,
            100,
            LL_RCC_PLLP_DIV_2
        );
        LL_RCC_PLL_Enable();
        for( auto r=0; r<retries; ++r ) {
            if( LL_RCC_PLL_IsReady() ) {
                break;
            }
        }
        if( !LL_RCC_PLL_IsReady() ) {
            return false;
        }

        // Set PLL as SYSCLK source
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        for( auto r=0; r<retries; ++r ) {
            if( LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {
                break;
            }
        }
        if(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){
            return false;
        }

        // Disable HSI generator
        LL_RCC_HSI_Disable();
        for( int i=0; i<retries; ++i ) {
            if(!LL_RCC_HSI_IsReady()) {
                break;
            }
        }

        return !LL_RCC_HSE_IsReady();
    }
}

namespace app {

#if VERSION == 1

    void button_counter(void*) {
        
        // Configure LEDs
        periph::gpio::setup( 
            {led_3, led_4, led_5, led_6},
            periph::gpio::mode::out{
                periph::gpio::outtype::pushpull,
                periph::gpio::speed::low
            }
        );

        // Configure button
        periph::gpio::setup( 
            button,
            periph::gpio::mode::in{
                periph::gpio::pulltype::floating
            }
        );

        // State of the button (true = pressed)
        bool button_pressed = false;
                
        // Counter (mod(16)) displayed on the LEDs
        int counter = 0;

        // Debouncing utilities
        ostick_t debounce_begin = 0;
        
        while(true){
            
            dbprintf("Counter: %i", counter);

            // Display counter on LEDs
            periph::gpio::set(led_4, counter & (1U << 3));
            periph::gpio::set(led_3, counter & (1U << 2));
            periph::gpio::set(led_5, counter & (1U << 1));
            periph::gpio::set(led_6, counter & (1U << 0));

            // Debouncing
            if(!periph::gpio::get(button)){
                debounce_begin = isix::get_jiffies();
                button_pressed = false;
            }

            if(!button_pressed && isix::timer_elapsed(debounce_begin ,debounce_ms)){
                button_pressed = true;
                ++counter;
                if(counter > 15)
                    counter = 0;
            }           
            
        }
    }

#elif VERSION == 2

    void button_counter(void*) {

        // Configure LEDs
        LL_GPIO_InitTypeDef led_init_struct{
            .Mode = LL_GPIO_MODE_OUTPUT,
            .Speed = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL
        };
        led_init_struct.Pin = LL_GPIO_PIN_12;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_13;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_14;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_15;
        LL_GPIO_Init(GPIOD, &led_init_struct);

        // Configure button
        LL_GPIO_InitTypeDef button_init_struct{
            .Pin = LL_GPIO_PIN_0,
            .Mode = LL_GPIO_MODE_INPUT,
            .Pull = LL_GPIO_PULL_NO
        };
        LL_GPIO_Init(GPIOA, &button_init_struct);

        // State of the button (true = pressed)
        bool button_pressed = false;
        
        // Counter (mod(16)) displayed on the LEDs
        int counter = 0;

        // Debouncing utilities
        ostick_t debounce_begin = 0;

        while(true){
            
            // Display counter on LEDs
            if(counter & (1U << 3))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 12));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 12));
            if(counter & (1U << 2))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 13));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 13));
            if(counter & (1U << 1))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 14));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 14));
            if(counter & (1U << 0))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 15));  
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 15));

            // Debouncing
            if(!LL_GPIO_IsInputPinSet(GPIOA, (1U << 0))){
                debounce_begin = isix::get_jiffies();
                button_pressed = false;
            }

            if(!button_pressed && isix::timer_elapsed(debounce_begin ,debounce_ms)){
                button_pressed = true;
                ++counter;
                if(counter > 15)
                    counter = 0;
            }           
            
        }
    }

#elif VERSION == 3

    void button_counter(void*) {
        
        // Configure LEDs
        LL_GPIO_InitTypeDef led_init_struct{
            .Mode = LL_GPIO_MODE_OUTPUT,
            .Speed = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL
        };
        led_init_struct.Pin = LL_GPIO_PIN_12;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_13;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_14;
        LL_GPIO_Init(GPIOD, &led_init_struct);
        led_init_struct.Pin = LL_GPIO_PIN_15;
        LL_GPIO_Init(GPIOD, &led_init_struct);

        // Configure button
        LL_GPIO_InitTypeDef button_init_struct{
            .Pin = LL_GPIO_PIN_0,
            .Mode = LL_GPIO_MODE_INPUT,
            .Pull = LL_GPIO_PULL_NO
        };
        LL_GPIO_Init(GPIOA, &button_init_struct);
            
        // Interrupt configuration
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
        LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA , LL_SYSCFG_EXTI_LINE0);
        LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
        LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

        // Enabling EXTI irq in NVIC controller
        isix::set_irq_priority(EXTI0_IRQn , {1, 7});
        isix::request_irq(EXTI0_IRQn);

        // State of the button (true = pressed)
        static bool button_pressed = false;

        // Button counter
        unsigned int counter = 0;


        // Display counter on LEDs        
        while(true){
            if(counter & (1U << 3))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 12));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 12));
            if(counter & (1U << 2))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 13));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 13));
            if(counter & (1U << 1))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 14));
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 14));
            if(counter & (1U << 0))
                LL_GPIO_SetOutputPin(GPIOD, (1U << 15));  
            else
                LL_GPIO_ResetOutputPin(GPIOD, (1U << 15));


            // Check if debounce is active
            if(debounce_active){

                // Wait for input to stabilize
                isix::wait_ms(debounce_ms);

                // Check button state
                if(LL_GPIO_IsInputPinSet(GPIOA, (1U << 0))){
                    if(!button_pressed){
                        button_pressed = true;
                        ++counter;
                        if(counter > 15)
                            counter = 0;
                    }        
                }
                else
                    button_pressed = false;
            }
        }

    }

#endif

}

#if VERSION == 3

extern "C" {

    void exti0_isr_vector() {

        // Set debounce indicator
        debounce_active = true;

        // Clear interrupt flag
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    }

}

#endif

auto main() -> int
{
    // Initialize system semapher
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

    // Configure CLK distribution
    uc_periph_setup();

    // Create programm task
	isix::task_create( app::button_counter, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

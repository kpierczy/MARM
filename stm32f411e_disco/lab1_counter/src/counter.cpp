/**
 * Program's model version
 *  - ISIX
 *  - LL
 */
#define ISIX

#include <config/conf.h> // ISIX base configuration
#include <foundation/sys/dbglog.h> // Logging module
#include <periph/drivers/serial/uart_early.hpp> // UART module used by logging module
#include <isix.h> // ISIX system modules

#include <stm32_ll_exti.h> // EXTI module
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h> // SYSCFG (EXTI source) config

// ISIX specific includes
#ifdef ISIX
#include <isix/arch/irq.h> // ISIX interrupts
#include <periph/clock/clocks.hpp> // Peripherals enabling
#include <periph/gpio/gpio.hpp> // GPIOs module
#endif

// LL specific includes
#ifdef LL
#include <stm32_ll_bus.h> // Bus control (peripherals enabling)
#include <stm32_ll_gpio.h> // General Purpose Input Output
#endif




// Common 
namespace{

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 200;

    // Debouncing indicator
    volatile bool debounce_active = false;

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
}





#ifdef ISIX

namespace {
    
    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOS (A & D)
        periph::clock::device_enable(
            periph::dt::clk_periph{
                .xbus = periph::dt::bus::ahb1,
                .bit = RCC_AHB1ENR_GPIOAEN_Pos
            }
        );
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

        // Configure button
        periph::gpio::setup( 
            BUTTON,
            periph::gpio::mode::in{
                periph::gpio::pulltype::floating
            }
        );
    }

    // EXTI interrupts config
    void interrupt_config(){

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
        LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

        // Enabling EXTI interrupts in NVIC controller
        isix::set_irq_priority(EXTI0_IRQn , {1, 7});
        isix::request_irq(EXTI0_IRQn);
    }

    // Main thread
    void button_counter(void*) {
        
        // State of the button (true = pressed)
        static bool button_pressed = false;

        // Button counter
        unsigned int counter = 0;

        // Display counter on LEDs        
        while(true){

            // Display counter on LEDs
            periph::gpio::set(LED4, counter & (1U << 3));
            periph::gpio::set(LED3, counter & (1U << 2));
            periph::gpio::set(LED5, counter & (1U << 1));
            periph::gpio::set(LED6, counter & (1U << 0));

            // Check if debounce is active
            if(debounce_active){

                // Wait for input to stabilize
                isix::wait_ms(DEBOUNCE_MS);

                // Check button state
                if(periph::gpio::get(BUTTON)){
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
}

extern "C" {

    void exti0_isr_vector() {

        // Clear interrupt flag
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

        // Set debounce indicator
        debounce_active = true;
    }

}

#endif





#ifdef LL

namespace {

    // Display LEDs
    constexpr auto LED3 = LL_GPIO_PIN_13;
    constexpr auto LED4 = LL_GPIO_PIN_12;
    constexpr auto LED5 = LL_GPIO_PIN_14;
    constexpr auto LED6 = LL_GPIO_PIN_15;

    // User button
    constexpr auto BUTTON = LL_GPIO_PIN_0;

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
    void interrupt_config(){

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
    CLK_config();

    // GPIO configuration
    GPIO_config();

    // EXTI interrupts config
    interrupt_config();

    // Create programm task
	isix::task_create( button_counter, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
	isix::start_scheduler();

	return 0;
}

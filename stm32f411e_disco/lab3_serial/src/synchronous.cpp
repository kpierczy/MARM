#include <config/conf.h>                        // ISIX base configuration
#include <foundation/sys/dbglog.h>              // Logging module
#include <periph/drivers/serial/uart_early.hpp> // UART module used by logging module
#include <isix.h>                               // ISIX system modules
#include <periph/gpio/gpio.hpp>                 // GPIOs module
#include <stm32_ll_bus.h>                       // Bus control (peripherals enabling)
#include <stm32_ll_usart.h>                     // USART control
#include <string.h>

namespace {

    // Maximum read buffer size
    constexpr unsigned int BUFFER_SIZE = 100;

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;

    // USART1 pinout
    constexpr auto TX1 = periph::gpio::num::PA15;
    constexpr auto RX1 = periph::gpio::num::PB7;


     // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOs (A & D)
        LL_AHB1_GRP1_EnableClock(
            LL_AHB1_GRP1_PERIPH_GPIOA |
            LL_AHB1_GRP1_PERIPH_GPIOB |
            LL_AHB1_GRP1_PERIPH_GPIOD 
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

        // Configure USART1 pins
        periph::gpio::setup( 
            {TX1, RX1},
            periph::gpio::mode::alt{
                periph::gpio::outtype::pushpull,
                7,
                periph::gpio::speed::high
            }
        );
    }

#ifndef ISIX

    /**
     * USART1 configuration :
     * 
     *    1) Baud rate : 115200
     *    2) Data bits : 8
     *    3: Stop bits : 1
     *    4) Parity control : None 
     * 
     */
    void USART1_config(){

        // NebaleEnable APB2 clock for USART1 peripheral
        LL_APB2_GRP1_EnableClock(
            LL_APB2_GRP1_PERIPH_USART1
        );

        // Fill USART's init structure
        LL_USART_InitTypeDef usart_struct{
            .BaudRate = 115200,
            .DataWidth = LL_USART_DATAWIDTH_8B,
            .StopBits = LL_USART_STOPBITS_1,
            .Parity = LL_USART_PARITY_NONE,
            .TransferDirection = LL_USART_DIRECTION_TX_RX,
            .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
            .OverSampling = LL_USART_OVERSAMPLING_16
        };

        // // Initialize USART1
        LL_USART_Init(USART1, &usart_struct);

        // Enable USART1
        LL_USART_Enable(USART1);

    }

    /**
     *  Put a single character to transfer
     */
    void putc( int ch ){
        while( !LL_USART_IsActiveFlag_TXE(USART1) );
        LL_USART_TransmitData8(USART1, ch);
    }

    /**
     *  Get a single character from buffer
     */
    int getc(){
        while( !LL_USART_IsActiveFlag_RXNE(USART1) );
        return LL_USART_ReceiveData8(USART1);
    }

    /**
     * 
     * Get a string from a buffer (chain of characters close with NULL).
     * 
     *  @param arr : pointer to the array to fill
     *  @returns : number of characters received - 1 ('\n' included).
     *             If '\n' character is not received, function returns (-1)
     */
    int getline(char * arr){
        int num;
        for(num = 0; num < BUFFER_SIZE; ++num){
            arr[num] = getc();
            if(arr[num] == '\n')
                break;
        }
        if(num == BUFFER_SIZE)
            return -1;
        else
            return num;
    }

    /**
     * Sends C-type string via USART1
     * 
     * @param str : C-type string to send (end with NULL)
     */
    void puts( const char * str ){
        while( *str ) {
            putc( *str++ );
        }
        while( !LL_USART_IsActiveFlag_TC(USART1) );
    }

#endif

    // Main thread
    void shell(void*) {

        char buffer[BUFFER_SIZE] {};
        periph::gpio::set(LED3, false);

        while(true){
            int ret = getline(buffer);
            if(ret > 0){
                buffer[ret] = NULL;
                
                // LED status change request
                if(strncmp(buffer, "led ", 4) == NULL){

                    // Initialize led to switch
                    auto led = preiph::gpio::PA0;

                    // Check led number
                    switch(buffer[4]){
                        case 3:
                            led = LED3;
                            break;
                        case 4:
                            led = LED4;
                            break;
                        case 5:
                            led = LED5;
                            break;
                        case 6:
                            led = LED6;
                            break;
                    };

                    // Check requested state
                    if(strcmp(buffer + 6, "on")){
                        periph::gpio::set(led, true);    
                    } 
                    if(strcmp(buffer + 6, "off")){
                        periph::gpio::set(led, false);
                    }
                } 
                // Button state request
                else if(strcmp(buffer, "button")){
                    char button_state[12] = "Button:  \n\r";
                    if(periph::gpio::get(BUTTON) == 1){
                        button_state[8] = '1';
                    }
                    else{
                        button_state[8] = '0';
                    }
                    puts(button_state)
                }
                // Free stack space request
                else if(strcmp(buffer, "heap")){
                    char heap_state[] = "Heap:     \n\r";
                    memory_stat stat;
                    heap_stats(memory_stat);
                    heap_state()
                }
                // CPU use request
                else if(strcmp(buffer, "cpu")){
                    char cpu_state[] = "CPU: \n\r"
                    puts()
                }
                    
            }
        }
    }

}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );

    // Configure GPIO
    GPIO_config();

    // Configure USART1
    USART1_config();

    // Create task
	isix::task_create( shell, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
    isix::start_scheduler();

	return 0;
}

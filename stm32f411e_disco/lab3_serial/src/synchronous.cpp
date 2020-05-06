/*================================================================================
 *
 *    Filename : synchronous.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm implements a simple serial port shell that features
 *               controlling a few basic elements of the system including:
 *                  
 *                 (1) LEDs controlling
 *                 (2) Button state checking
 *                 (3) CPU load measuring
 *                 (4) heap statistics checking
 * 
 *               The shell is shared via USART1 peripheral module and can be
 *               used through PA15(TX) and PB7(RX) pins. Description of the usage
 *               can be found in @doc.
 * 
 *               This version of the shell is implemented using synchronous UART
 *               communication.
 *
 *===============================================================================*/

#include <config/conf.h>                        // (ISIX) : base configuration
#include <isix.h>                               //  ISIX  : system modules
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <memory.h>                             //  ISIX  : Heap informations
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <periph/clock/clocks.hpp>              // PERIPH : clocks enabling
#include <periph/drivers/spi/spi_master.hpp>    // PERIPH : SPI controller
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <stm32_ll_bus.h>                       // LL : Bus control (peripherals enabling)
#include <stm32_ll_usart.h>                     // LL : USART control
#include <string.h>                             // STD : C-strings formatting

bool shellCommand(char * str, void (*puts)(const char *));

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

        // Initialize USART1
        LL_USART_Init(USART1, &usart_struct);

        // Enable USART1
        LL_USART_Enable(USART1);

    }

    /**
     *  Waits for transfer buffer to be empty and puta
     *  a single character to it
     * 
     *  @param ch : character to transfer
     */
    void putc( int ch ){
        while( !LL_USART_IsActiveFlag_TXE(USART1) );
        LL_USART_TransmitData8(USART1, ch);
    }

    /**
     * Sends C-type string via USART1. After putting the
     * last character into buffer function waits for
     * TC (Transfer Complete) flag to be set by hardware
     * 
     * @param str : C-type string to send (ends with NULL)
     */
    void puts( const char * str ){
        while( *str ) {
            putc( *str++ );
        }
        while( !LL_USART_IsActiveFlag_TC(USART1) );
    }

    /**
     * Waits for receive buffer to receive a new data.
     * Then returns it's content.     * 
     */
    int getc(){
        while( !LL_USART_IsActiveFlag_RXNE(USART1) );
        return LL_USART_ReceiveData8(USART1);
    }

    /**
     * Get a line from the USART1 channel. Line is defined
     * as a chain of characters ended up with \n or \r.
     * 
     *  @param arr [out] : incoming string buffer
     *  @returns : number of characters received (line-ending
     *             character excluded). If \n or \r character
     *             is not received before buffer overflow,
     *             function returns (-1)
     */
    int getline(char * arr){
        int num;
        for(num = 0; num < BUFFER_SIZE; ++num){
            arr[num] = getc();
            if(arr[num] == '\n' || arr[num] == '\r'){
                arr[num] = NULL;
                break;
            }
        }
        if(num == BUFFER_SIZE)
            return -1;
        else
            return num;
    }


    /**
     * Main thread function implementing a simple interactive
     * shell application. User can communicate with the programm
     * via USART1 channel.
     */
    void shell(void*) {

        // Read buffer
        char buffer[BUFFER_SIZE] {};

        while(true){

            // Ge a new line (blocking read)
            int ret = getline(buffer);

            /**
             * If some characters were received check the line content.
             * There are several commands that can be server by the programm.
             * The if-else ladder analyses the string and decides what
             * action should be taken.
             */
            if(ret > 0){

                // Analyse incoming line
                shellCommand(buffer, &puts);

                // Clear str (debug helpful)
                for(int i = 0; i < BUFFER_SIZE; ++i)
                    buffer[i] = NULL;
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

    // USART1 configuration
    USART1_config();

    // Create task
	isix::task_create( shell, nullptr, 1536, isix::get_min_priority() );

    // Begin scheduling
    isix::start_scheduler();

	return 0;
}

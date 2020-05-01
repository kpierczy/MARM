#include <config/conf.h>                        // ISIX base configuration
#include <foundation/sys/dbglog.h>              // Logging module
#include <periph/drivers/serial/uart_early.hpp> // UART module used by logging module
#include <isix.h>                               // ISIX system modules
#include <isix/arch/irq.h>                      // ISR symbols
#include <memory.h>                             // Heap informations
#include <periph/gpio/gpio.hpp>                 // GPIOs module
#include <stm32_ll_bus.h>                       // Bus control (peripherals enabling)
#include <stm32_ll_usart.h>                     // USART control
#include <string.h>                             // C-strings formatting

namespace {

    // Maximum read buffer size
    constexpr unsigned int BUFFER_SIZE = 100;

    // Asynchronous USART operations flags
    volatile bool read_data_ready = false;
    volatile bool read_buffer_corrupted = false;
    volatile bool read_buffer_overflow = false;
    
    volatile bool write_data_ready = false;
    volatile bool write_buffer_corrupted = false;

    // Buffer structure
    typedef struct buffer_struct_t{
        char buffer[BUFFER_SIZE];
        int index;
    } buffer_struct; 

    // True = buffer 1, False = buffer 2
    volatile bool input_buffer_choice = true;

    // Read-buffer banks
    volatile buffer_struct input_buffers[2] = {          
        {.index = 0},
        {.index = 0}
    };

    // True = buffer 1, False = buffer 2
    volatile bool output_buffer_choice = true;

    // Read-buffer banks
    volatile buffer_struct output_buffers[2] = {          
        {.index = 0},
        {.index = 0}
    };
    
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

        // Enable receive-data interrupts
        isix_irq_prio_t priority{
            .prio = 0,
            .subp = 7
        };
        isix_set_irq_priority(USART1_IRQn, priority);
        isix::request_irq(USART1_IRQn);
        LL_USART_EnableIT_RXNE(USART1);

        // Enable USART1
        LL_USART_Enable(USART1);

    }

    /**
     * Sends C-type string via USART1. After putting the
     * last character into buffer function waits for
     * TC (Transfer Complete) flag to be set by hardware
     * 
     * @param str : C-type string to send (ends with NULL)
     */
    void puts( const char * str ){
        
        // Coppy data to global write buffer & reset internal index
        strcpy(const_cast<char *>(output_buffers[output_buffer_choice].buffer), str);
        output_buffers[output_buffer_choice].index = 0;

        // Wait to transfer to finish
        while(write_data_ready);

        // Change active buffer bank
        output_buffer_choice = !output_buffer_choice;

        // Active global data-ready flag
        write_data_ready = true;

        // Activate transfer-data interrupts
        LL_USART_EnableIT_TXE(USART1);
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

        // If new data is read acquire it and return number of
        // acquired characters
        if(read_data_ready){
            for(int i = 0; i < input_buffers[!input_buffer_choice].index + 1; ++i){
                arr[i] = input_buffers[!input_buffer_choice].buffer[i];
            }
            read_data_ready = false;
            return input_buffers[!input_buffer_choice].index;
        }
        // Else return negative value
        else{
            return -1;
        }
    }

    /**
     * Function analyses an input string. If a valid
     * shell command is recognized, an adequate action
     * is taken.
     * 
     * @param str [in] : input string
     * @returns : true if an input string was recognised
     *            as a valid command; false otherwise
     * 
     * @note : content of the input value is discarded
     */
    bool shellCommand(char * str){
        
        /**
         * (1) LED status change request
         */
        if(strncmp(str, "led ", 4) == NULL){

            // Initialize led to switch
            auto led = periph::gpio::num::PA0;

            // Check led number
            switch(str[4]){
                case '3':
                    led = LED3;
                    break;
                case '4':
                    led = LED4;
                    break;
                case '5':
                    led = LED5;
                    break;
                case '6':
                    led = LED6;
                    break;
                default:
                    return false;
                    
            };

            // If led number is valid, try to execute command
            if(strcmp(str + 6, "on") == NULL){
                periph::gpio::set(led, true);    
            } 
            if(strcmp(str + 6, "off") == NULL){
                periph::gpio::set(led, false);
            }

            return true;

        } 
        /**
         *  (2) Request for the USER button state
         */
        else if(strcmp(str, "button") == NULL){

            // Read button state and write an appropriate message to terminal
            char button_state[30];
            if( sprintf(button_state, "Button:  %i\n\r", periph::gpio::get(BUTTON)) < 0 )
                puts("Error! Cannot display button state!\r\n");
            else
                puts(button_state);                    

            return true;
                
        }
        /**
         * (3) Request for the heap status
         */
        else if(strcmp(str, "heap") == NULL){

            // Get informations about heap
            isix_memory_stat heap;
            isix_heap_stats(&heap);
            
            // Print informations
            char heap_free[30] {};
            char heap_used[30] {};
            char heap_fragments[30] {};

            if( sprintf(heap_free     , "Free     :  %i\n\r", heap.free     ) < 0 ||
                sprintf(heap_used     , "Used     :  %i\n\r", heap.used     ) < 0 ||
                sprintf(heap_fragments, "Fragments:  %i\n\r", heap.fragments) < 0 )
                puts("Error! Cannot display heap informations!\r\n");
            else{
                puts(heap_free);
                puts(heap_used);
                puts(heap_fragments);
            }

            return true;

        }
        /**
         * (4) Request for CPU state
         */
        else if(strcmp(str, "cpu") == NULL){

            // Read CPU load and write an appropriate message to terminal
            char cpu_state[30] {};
            if(sprintf(cpu_state, "CPU: %i%% \r\n", isix::cpuload() / 10 ) < 0)
                puts("Error! Cannot display CPU informations!");
            else
                puts(cpu_state);

            return true;

        }
        /**
         * (5) Print available commands
         */
        else if(strcmp(str, "help") == NULL){

            puts("\r\n");
            puts("<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>\r\n");
            puts("|   1) led x on/off [x in {3, 4, 5, 6}]   |\r\n");
            puts("|   2) button                             |\r\n");
            puts("|   3) heap                               |\r\n");
            puts("|   4) cpu                                |\r\n");
            puts("<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>\r\n");
            puts("\r\n");

            return true;

        }
        /**
         * (6) If user typed a longer string that was
         *     not recognized as a valid command print
         *     help information.
         */
        else if(strlen(str) > 10){

            puts("\r\n");
            puts("<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>\r\n");
            puts("| Incorrect command. Available commands are: |\r\n");
            puts("|   1) led x on/off [x in {3, 4, 5, 6}]      |\r\n");
            puts("|   2) button                                |\r\n");
            puts("|   3) heap                                  |\r\n");
            puts("|   4) cpu                                   |\r\n");
            puts("<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>\r\n");
            puts("\r\n");

        }

        return false;

    }


    /**
     * Main thread function implementing a simple interactive
     * shell application. User can communicate with the programm
     * via USART1 channel.
     */
    void shell(void*) {

        // USART buffers
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
                shellCommand(buffer);

                // Clear str (debug helpful)
                for(int i = 0; i < BUFFER_SIZE; ++i)
                    buffer[i] = NULL;
            }
                
        }
    }

}

extern "C"{

    void usart1_isr_vector(){

        /**
         * Input data gathering
         */
        if(LL_USART_IsActiveFlag_RXNE(USART1)){
            
            // Clear interrupt flag as fast as possible
            LL_USART_ClearFlag_RXNE(USART1);

            // Read data from the USART1 buffer
            input_buffers[input_buffer_choice].buffer[
                input_buffers[input_buffer_choice].index
            ] = LL_USART_ReceiveData8(USART1);

            // Check if received character is a line-end character
            if(input_buffers[input_buffer_choice].buffer[input_buffers[input_buffer_choice].index] == '\n' ||
               input_buffers[input_buffer_choice].buffer[input_buffers[input_buffer_choice].index] == '\r'   ){

                // Substitute line-end character with string-end character (NULL)
                input_buffers[input_buffer_choice].buffer[input_buffers[input_buffer_choice].index] = NULL;

                // Change active buffer bank
                input_buffer_choice = !input_buffer_choice;

                // If previous data was gathered, active global data-ready flag
                if(!read_data_ready)                        
                    read_data_ready = true;

                // Otherwise active global overflow flag
                else
                    read_buffer_corrupted = true;

                // Reset buffer bank index
                input_buffers[input_buffer_choice].index = 0;
            }
            else{

                // Increment buffer index; if buffer overlown raise a global flag
                if(++(input_buffers[input_buffer_choice].index) == BUFFER_SIZE){
                    read_buffer_overflow = true;
                    input_buffers[input_buffer_choice].index = 0;
                }

            }
        }
        /**
         * Output data transferring
         */
        else if(LL_USART_IsActiveFlag_TXE(USART1)){

            // If the next character in the buffer is not NULL (string end)
            if(output_buffers[!output_buffer_choice].buffer[
                output_buffers[!output_buffer_choice].index] != NULL){

                // Send next data byte
                LL_USART_TransmitData8(USART1, output_buffers[!output_buffer_choice].buffer[
                                                   output_buffers[!output_buffer_choice].index]);

                // Increment buffor index
                ++(output_buffers[!output_buffer_choice].index);

            }
            // Otherwise wait for transfer to finish and desactivate 
            else{

                // Clean global flag
                write_data_ready = false;

                // Wait for transfer to complete
                while( !LL_USART_IsActiveFlag_TC(USART1) );

                // DIsable USART transfer interrupts
                LL_USART_DisableIT_TXE(USART1);

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

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
 * Description : File contains functions that are common for both synchronous
 *               and asynchronous serial shell.
 *
 *===============================================================================*/

#include <isix.h>                               //  ISIX  : system modules
#include <memory.h>                             //  ISIX  : Heap informations
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <string.h>                             // STD : C-strings formatting

namespace{

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // Display LEDs
    constexpr auto LED3 = periph::gpio::num::PD13;
    constexpr auto LED4 = periph::gpio::num::PD12;
    constexpr auto LED5 = periph::gpio::num::PD14;
    constexpr auto LED6 = periph::gpio::num::PD15;
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
bool shellCommand(char * str, void (*puts)(const char *)){
    
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
        if(sprintf(cpu_state, "CPU: %d.1%% \r\n", ((float)isix::cpuload()) / 10.0f ) < 0)
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
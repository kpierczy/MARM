/*
 *============================================================
 *
 *   File name   : platform_setup.cpp
 *   Created on  : 20 lis 2013
 *   Author      : lucck
 *   Description : Function crucial for the system startup
 * 
 *============================================================
 */
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_gpio.h>
#include <config/conf.h>
#include <functional>
#include <isix/arch/irq.h>
#include <isix.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>

namespace drv {
namespace board {

namespace {

	/**
     * Configures CLK sources as follows:
     * 
     *  - SYSCLK Source : PLLCLK
     *  - PLL Source    : HSE
     *  - HSI           : Disabled
     *  - HCLK          : 100MHz
     *  - APB1          : 50MHz
     *  - APB2          : 100MHz
     * 
     * @returns : true if initialization succeded
     */
    bool uc_periph_setup()
    {
        // Number of retries while waiting for 
        // CLK modules to turn on/off
        constexpr auto RETRIES = 100000;
        
        // Set interrupt vector tabke base address
		isix_set_irq_vectors_base( &_exceptions_vectors );

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
            LL_RCC_PLLM_DIV_8,
            400,
            LL_RCC_PLLP_DIV_4
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

        return true;
    }

    /* 
     * Application crash service routine
     * 
     * @note : This function is called from the
     *         HardFault routine.
     */
    void application_crash( crash_mode type, unsigned long* sp )
    {
        #ifdef PDEBUG
        cortex_cm3_print_core_regs( type, sp );
        #else
        static_cast<void>(type);
        static_cast<void>(sp);
        #endif
        for(;;) asm volatile("wfi\n");
    }

}

extern "C" {

    /**
     * Function is called just before global constructors call
     */
    void _external_startup(void)
    {
        // Wait to give a chance a JTAG to reset the CPU
        for(unsigned i = 0; i < 1000000; i++)
            asm volatile("nop\n");

        //Initialize system perhipheral
        if(uc_periph_setup()){

            // Set 1 bit for preemtion priority
            isix_set_irq_priority_group( isix_cortexm_group_pri7 );

            //Initialize isix
            isix::init(CONFIG_HCLK_HZ);

        }
        // @todo : Handle failure initialization
        else {
            for(;;);
        }
    }

    
    /**
     * Hard fault interrupt handler
     */
    void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
    {
        _cm3_hard_hault_entry_fn( application_crash );
    }
    
} 

} //board namespace
} // drv namespace


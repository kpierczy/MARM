/*================================================================================
 *
 *    Filename : adc.cpp
 *        Date : Fri May 22 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Programm measures RMS voltage of the signal present on the
 *               INPUT (PA6) pin. It uses AD convertion with DMA transfers
 *               to provide deterministic sampling rate.
 *              
 *               Data is saved to the circular buffer at frequency of 48kHz.
 *               Main thread reads this data and computes rms with full speed.
 *               RMS level is vizualized with a simple ruler build with LEDs.
 * 
 *               TIM5 is configured to produce 1kHz PWM with a duty cycle from
 *               six possibles. Duty cycle is changed tith USER button.
 * 
 *               Actual measurement and theoretical RMS value are printed to
 *               the serial port with a frequency configured with programm's
 *               configuration section.
 *
 *===============================================================================*/

#include <config/conf.h>                        // (ISIX) : base configuration
#include <isix.h>                               //  ISIX  : system modules
#include <isix/arch/irq.h>                      //  ISIX  : ISR symbols
#include <foundation/sys/dbglog.h>              // FOUNDATION : Logging module
#include <periph/gpio/gpio.hpp>                 // PERIPH : GPIO framework
#include <periph/drivers/serial/uart_early.hpp> // PERIPH : UART controller used by logging module
#include <stm32_ll_bus.h>                       // LL : Bus control (peripherals enabling)
#include <stm32_ll_exti.h>                      // LL : EXTI controller
#include <stm32_ll_rcc.h>                       // LL : Reset & Clock control
#include <stm32_ll_system.h>                    // LL : EXTI macros
#include <stm32_ll_tim.h>                       // LL : Timers API
#include <stm32_ll_adc.h>                       // LL : ADC API
#include <stm32_ll_dma.h>                       // LL : DMA API
#include <math.h>                               // Square root calculation

/*-----------------------------------------------------------------------*/
/*------------------------ Configuration data ---------------------------*/
/*-----------------------------------------------------------------------*/

namespace {

    // Time between subsequent logs [ms]
    constexpr auto LOGGING_PERIOD = 1000;

    // Debouncing time [ms]
    constexpr int DEBOUNCE_MS = 200;

    // Display LEDs
    constexpr auto LD3 = periph::gpio::num::PD13;
    constexpr auto LD4 = periph::gpio::num::PD12;
    constexpr auto LD5 = periph::gpio::num::PD14;
    constexpr auto LD6 = periph::gpio::num::PD15;

    // GPIOs declarations
    constexpr auto INPUT = periph::gpio::num::PA6;
    constexpr auto GENERATOR = periph::gpio::num::PA1;

    // User button
    constexpr auto BUTTON = periph::gpio::num::PA0;

    // Duty cycle increment at each button press [%]
    float incrementStep = 0.25f;

    // Filter data
    volatile float rms {};
    constexpr unsigned int WINDOW_SIZE = 100;
    volatile uint16_t samples[WINDOW_SIZE] {};
}


/*-----------------------------------------------------------------------*/
/*---------------------- Configuration function -------------------------*/
/*-----------------------------------------------------------------------*/

namespace{

    // GPIO ports configuration
    void GPIO_config(){

        // Enable clocks for GPIOA & GPIOD
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOD);

        // Configure LEDs
        periph::gpio::setup( 
            {LD3, LD4, LD5, LD6},
            periph::gpio::mode::out{
                periph::gpio::outtype::pushpull,
                periph::gpio::speed::low
            }
        );
        
        // Configure GENERATOR pin to alternate function (TIMx output)
        periph::gpio::setup( 
            {GENERATOR},
            periph::gpio::mode::alt{
                .out   = periph::gpio::outtype::pushpull,
                .altno = 2,
                .spd = periph::gpio::speed::high
            }
        );

        // Configure BUTTON pin as input
        periph::gpio::setup( 
            BUTTON,
            periph::gpio::mode::in{
                periph::gpio::pulltype::floating
            }
        );

        // Configure INPUT pin to analog function (ADC1 input)
        periph::gpio::setup( 
            {INPUT},
            periph::gpio::mode::an{
                periph::gpio::pulltype::floating
            }
        );
    }


    /*------------------------------- Timers --------------------------------*/

    /* 
     * TIM2 configuration for periodic overflow (48kHz)
     */
    void TIM2_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM2);

        // TIM2 configuration structure (CLK = 10MHz, 103 = 1 -> ~48kHz)
        LL_TIM_InitTypeDef TIM2_struct{
            // .Prescaler         = __LL_TIM_CALC_PSC(100'000'000, 10'000'000),
            // .Autoreload        = __LL_TIM_CALC_ARR(100'000'000, __LL_TIM_CALC_PSC(100'000'000 , 10'000'000), 48'000)
            .Prescaler         = __LL_TIM_CALC_PSC(100'000'000, 10'000'000),
            .Autoreload        = __LL_TIM_CALC_ARR(100'000'000, __LL_TIM_CALC_PSC(100'000'000 , 10'000'000), 48'000)
        }; LL_TIM_Init(TIM2, &TIM2_struct);
        
        // Set TRGO event to be triggered at timer's overflow
        LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);

        // Enable TIM2
        LL_TIM_EnableCounter(TIM2);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM2);

    }


    /**
     *  TIM5 configuration for PWM generator
     */
    void TIM5_config(void){

        // Enable clock from APB1 for the TIM1 periph
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

        // Enable ARR preloading
        LL_TIM_EnableARRPreload(TIM5);

        // TIM5 configuration structure (CLK = 10MHz, ARR = 999 -> 1kHz)
        LL_TIM_InitTypeDef TIM5_struct{
            .Prescaler  = __LL_TIM_CALC_PSC(100'000'000 , 1'000'000),
            .Autoreload = __LL_TIM_CALC_ARR(100'000'000 , __LL_TIM_CALC_PSC(100'000'000 , 1'000'000), 1'000)
        }; LL_TIM_Init(TIM5, &TIM5_struct);

        /**
         * TIM5 compare/capture configuration structures
         *     - Mode       : PWM1
         *     - Frequency  : 10 kHz
         *     - Duty Cycle : 50%
         *     - CCR1 buffered
         */
        LL_TIM_OC_InitTypeDef TIM5_CC_struct{
            .OCMode       = LL_TIM_OCMODE_PWM1,
            .OCState      = LL_TIM_OCSTATE_ENABLE,
            .CompareValue = 0,
            .OCPolarity   = LL_TIM_OCPOLARITY_HIGH
        };
        LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH2, &TIM5_CC_struct);
        
        // Enable TIM5
        LL_TIM_EnableCounter(TIM5);

        // Initialize shadow registers
        LL_TIM_GenerateEvent_UPDATE(TIM5);
    }


    /*------------------------ ADC configuration ----------------------------*/

    /**
     * ADC configuration - perform channel 6 conversion triggered
     * by the TIM1 overflow event
     */
    void ADC_config(){
        
        // Enable ADC source clock
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

        // Set ADC clockv value (25MHz)
        LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV8);

        /**
         * Initialize ADC to produce 12-bit, 
         * right-alligned samples.
         */
        LL_ADC_InitTypeDef adc_struct{
            .Resolution         = LL_ADC_RESOLUTION_12B,
            .DataAlignment      = LL_ADC_DATA_ALIGN_RIGHT,
            .SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE
        }; LL_ADC_Init(ADC1, &adc_struct);

        // Set number of cycles that SAH module gathers a sample
        LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4 , LL_ADC_SAMPLINGTIME_56CYCLES);

        /**
         * Initialize ADC group regular conversions:
         *   - convert channel 6
         *   - triggered byt TIM2 TRGO event
         *   - unlimited DMA requests (DMA writes to circular buffer) 
         */
        LL_ADC_REG_InitTypeDef adc_reg_struct{
            .TriggerSource    = LL_ADC_REG_TRIG_EXT_TIM2_TRGO,
            .SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE,
            .SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE,
            .ContinuousMode   = LL_ADC_REG_CONV_SINGLE,
            .DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED
        }; LL_ADC_REG_Init(ADC1, &adc_reg_struct);
        LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1 , LL_ADC_CHANNEL_6);
        
        // Enable ADC
        LL_ADC_Enable(ADC1);

        // Start convertion
        LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    

    }

    /**
     * DMA2 (stream 0, channel 0) configuration - single  
     * channel is used to gather samples produced by the ADC. 
     * These samples feed circuler buffer named 'samples'.
     */
    void DMA_config(){

        // configure priority of the DMA interrupt
        NVIC_SetPriority(DMA2_Stream0_IRQn , 1);
        NVIC_EnableIRQ(DMA2_Stream0_IRQn);

        // Enable DMA clock
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

        
        /**
         * Initialize single DMA channel
         */
        LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0 , LL_DMA_CHANNEL_0);
        LL_DMA_InitTypeDef dma_struct{
            .PeriphOrM2MSrcAddress  = LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
            .MemoryOrM2MDstAddress  = (uint32_t)samples,
            .Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
            .Mode                   = LL_DMA_MODE_CIRCULAR,
            .PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT,
            .MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
            .NbData                 = WINDOW_SIZE,
            .Channel                = LL_DMA_CHANNEL_0,
            .Priority               = LL_DMA_PRIORITY_HIGH
        }; LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &dma_struct);

        // Enable DMA2 interrupt at buffer overflow
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
        
        // Enable DMA
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
    }

}


/*-----------------------------------------------------------------------*/
/*-------------------------- Application code ---------------------------*/
/*-----------------------------------------------------------------------*/

namespace{

    // Updates indicator with given values
    void updateIndicator(bool one, bool two, bool three, bool four){
        periph::gpio::set(LD4, one);
        periph::gpio::set(LD3, two);
        periph::gpio::set(LD5, three);
        periph::gpio::set(LD6, four);
    }
}



namespace app{


    /**
     * Thread responsible for watching USER button state
     * and changing PWM duty cycle if it's pressed
     */
    void button_watch(void*){
        
        // State of the button (true = pressed)
        static bool pressed = false;

        // Display counter on LEDs        
        while(true){

            // Check if debounce is active
            if(periph::gpio::get(BUTTON)){

                if(!pressed){

                    // Wait for input to stabilize
                    isix::wait_ms(DEBOUNCE_MS);

                    // Check button state
                    if(periph::gpio::get(BUTTON)){
                        if(!pressed){
                            
                            pressed = true;
                            
                            // Set next duty cycle
                            if(TIM5->CCR2 + float(incrementStep) / 100.0f * TIM5->ARR > (1<<16 - 1)){
                                TIM5->CCR2 = 0;
                            }else{
                                TIM5->CCR2 = TIM5->CCR2 + float(incrementStep) / 100.0f * TIM5->ARR;
                            }

                        }        
                    }
                    else
                        pressed = false;
                }
            }
            else
                pressed = false;
        }
    }


    /**
     * Thread responsible RMS value calculation
     */
    void rms_filter(void*){

        while(true){

            // Compute rms
            int sum = 0;
            for(unsigned i = 0; i < WINDOW_SIZE; ++i)
                sum += samples[i] * samples[i];

            unsigned int rms_raw = sqrt(sum / WINDOW_SIZE);
            rms = 3300.0f * rms_raw / (1<<12);

            // Indicate RMS < 60%
            if(rms < 0.3 * 3300.0f){
                updateIndicator(false, false, false, false);
            }
            // Indicate 60% < RMS < 70%
            else if(rms < 0.5 * 3300.0f){
                updateIndicator(true, false, false, false);
            }
            // Indicate 70% < RMS < 80%
            else if(rms < 0.7 * 3300.0f){
                updateIndicator(true, true, false, false);
            }
            // Indicate 80% < RMS < 90%
            else if(rms < 0.9 * 3300.0f){
                updateIndicator(true, true, true, false);
            }
            // Indicate 90% < RMS < 100%
            else{
                updateIndicator(true, true, true, true);
            }

        }
    }

    /**
     * Thread responsible for writting measured and theoretical
     * rms values to the serial port.
     */
    void logger(void*){

        while(true){

            char buffer[200] {};
            sprintf(buffer,
                "Actual: %4.2fmV | Measured: %04.2fmV\r\n",
                3300.0f * sqrt(float(TIM5->CCR2) / float(TIM5->ARR)),
                rms
            ); fnd::tiny_printf(buffer);
            
            isix::wait_ms(LOGGING_PERIOD);
        }

    }
}


/*-----------------------------------------------------------------------*/
/*---------------------------- Interrupts -------------------------------*/
/*-----------------------------------------------------------------------*/

extern "C" {

    void dma2_stream0_isr_vector()
    {
        if(LL_DMA_IsActiveFlag_TE0(DMA2) == 1)
        {
            LL_DMA_ClearFlag_TE0(DMA2);
            dbprintf("Error occured");
        } 
    }

}


/*-----------------------------------------------------------------------*/
/*--------------------------- Program entry -----------------------------*/
/*-----------------------------------------------------------------------*/

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
    
    // Configure TIM1 (ADC beat signal)
    TIM2_config();

    // Configure TIM5 (Generator)
    TIM5_config();

    // Configure ADC
    ADC_config();

    // Configure DMA
    DMA_config();

    // Create threads
    isix::task_create(app::button_watch, nullptr, 1536, isix::get_min_priority());
	isix::task_create(app::rms_filter  , nullptr, 1536, isix::get_min_priority());
    isix::task_create(app::logger      , nullptr, 1536, isix::get_min_priority());


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

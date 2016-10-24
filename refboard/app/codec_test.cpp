/*
 * =====================================================================================
 *
 *       Filename:  codec_test.cpp
 *
 *    Description:  STM32 codec test
 *
 *        Version:  1.0
 *        Created:  02.09.2016 19:29:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB)
 *
 * =====================================================================================
 */
#include <isixdrv/i2c_bus.hpp>
#include <cstdint>
#include <foundation/dbglog.h>
#include <stm32rcc.h>
#include <stm32spi.h>
#include <stm32dma.h>
#include <stm32system.h>
#include <cmath>


// FIXME: Conflict on DMA stream 0 and 5 with I2C1_RX
//   Update STM32 I2C driver without DMA and IRQ only.

namespace app {

namespace {

const  auto I2S3_DR_Address			=			&SPI3->DR;
const  auto I2S3ext_DR_Address		=			&I2S3ext->DR;
constexpr auto DMA1_S3_PRIORITY			= 	0;
constexpr auto DMA1_S3_SUBPRIORITY	    =			0;

const  auto I2S3_SDTI_PORT			=			GPIOB;
constexpr auto I2S3_SDTI_PIN		    =			4;

const  auto I2S3_SDTO_PORT		    =			GPIOB;
constexpr auto I2S3_SDTO_PIN		    =			5;

const  auto I2S3_BICK_PORT		    =			GPIOB;
constexpr auto I2S3_BICK_PIN			=			3;

const  auto I2S3_LRCK_PORT			=			GPIOA;
constexpr auto I2S3_LRCK_PIN			=			15;

const  auto I2S3_MCLK_PORT			=			GPIOC;
constexpr auto I2S3_MCLK_PIN			=			7;

constexpr auto CHANNEL_BUFFER_SIZE		= 128;
constexpr auto BUFFER_SIZE				=			2*CHANNEL_BUFFER_SIZE;


	static signed long buf1[BUFFER_SIZE];
	static signed long buf2[BUFFER_SIZE];

	constexpr auto wm8731_addr_0 = 0x1A;
	constexpr auto codec_addr = wm8731_addr_0<<1;
	constexpr uint16_t w8731_init_data[] = 
	{
		0b000011111,			// Reg 00: Left Line In (0dB, mute off)
		0b000011111,			// Reg 01: Right Line In (0dB, mute off)
		0x079,			// Reg 02: Left Headphone out (0dB)
		0x079,			// Reg 03: Right Headphone out (0dB)
		0b000011101,			// Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
		0x000,			// Reg 05: Digital Audio Path Control
		0b000000000,			// Reg 06: Power Down Control (Clkout, Osc, Mic Off)
		//	0x00E,			// Reg 07: Digital Audio Interface Format (i2s, 32-bit, slave)
		0b000001000,			// Reg 07: Digital Audio Interface Format (i2s, 16-bit, slave)
		0x000,			// Reg 08: Sampling Control (Normal, 256x, 48k ADC/DAC)
		0x001			// Reg 09: Active Control
	};

	int codec_write_register( fnd::bus::ibus& bus, uint8_t addr, uint16_t val )
	{
		/* Assemble 2-byte data in WM8731 format */
		const uint8_t arr[2] {
			uint8_t(((addr<<1)&0xFE) | ((val>>8)&0x01)),
				uint8_t(val&0xFF)
		};
		return bus.write( codec_addr, arr, sizeof arr, nullptr, 0 );
	}
}


int codec_reset(  fnd::bus::ibus& bus  )
{
	auto ret = codec_write_register( bus, 0x0f, 0 );
	if( ret ) {
		dbg_err( "CODEC unable to write codec data %i", ret );
		return ret;
	}
	//Get some time for device reset
	isix::wait_ms(100);
	for( size_t reg = 0; reg<sizeof(w8731_init_data)/sizeof(w8731_init_data[0]); ++reg ) {
		if ( (ret=codec_write_register(bus,reg,w8731_init_data[reg])) ) {
			dbg_err("CODEC wr reg %lu err %i", reg, ret );
		}
	}
	return ret;
}
 

void i2s_init()
{

	using namespace stm32;
	//I2S PLL Config
	rcc_i2s_clk_config(RCC_I2S2CLKSource_PLLI2S);
	rcc_pll_i2s_config(192, 2);
	rcc_pll_i2s_cmd(ENABLE);
	while(!rcc_get_flag_status(RCC_FLAG_PLLI2SRDY));

	//DMA1 Clock Enable
	rcc_ahb1_periph_clock_cmd(RCC_AHB1Periph_DMA1, ENABLE);

	//SPI2 Clock Enable
	rcc_apb1_periph_clock_cmd(RCC_APB1Periph_SPI3, ENABLE);

	dbg_info("PLL is ok\n");

	//AF Config
	gpio_pin_AF_config(I2S3_SDTI_PORT, I2S3_SDTI_PIN, GPIO_AF_I2S3ext);		//SDTI1
	gpio_pin_AF_config(I2S3_SDTO_PORT, I2S3_SDTO_PIN, GPIO_AF_SPI3);		//SDTO1
	gpio_pin_AF_config(I2S3_BICK_PORT, I2S3_BICK_PIN, GPIO_AF_SPI3);		//BICK1
	gpio_pin_AF_config(I2S3_LRCK_PORT, I2S3_LRCK_PIN, GPIO_AF_SPI3);		//LRCK1
	gpio_pin_AF_config(I2S3_MCLK_PORT, I2S3_MCLK_PIN, GPIO_AF_SPI3);		//LRCK1

	//I2S2 Port Config
	gpio_config(I2S3_SDTI_PORT, I2S3_SDTI_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP);		//SDTI1
	gpio_config(I2S3_SDTO_PORT, I2S3_SDTO_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP);		//SDTO1
	gpio_config(I2S3_BICK_PORT, I2S3_BICK_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP);		//BICK1
	gpio_config(I2S3_LRCK_PORT, I2S3_LRCK_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP);		//LRCK1
	gpio_config(I2S3_MCLK_PORT, I2S3_MCLK_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP);		//MCLK1
	//NVIC Config
	nvic_set_priority(DMA1_Stream0_IRQn, DMA1_S3_PRIORITY, DMA1_S3_SUBPRIORITY);
	nvic_irq_enable(DMA1_Stream0_IRQn, ENABLE);

	//DMA1 Config (Circural Mode, Double Buffering, Packing)
	dma_init(DMA1_Stream0, DMA_Channel_3 | DMA_DIR_PeripheralToMemory | DMA_MemoryInc_Enable | DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_Word | DMA_Mode_Circular | DMA_Priority_VeryHigh, DMA_FIFOMode_Disable, 2*BUFFER_SIZE, I2S3ext_DR_Address, &buf1);		//RX
	dma_init(DMA1_Stream5, DMA_Channel_0 | DMA_DIR_MemoryToPeripheral | DMA_MemoryInc_Enable | DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_Word | DMA_Mode_Circular | DMA_Priority_VeryHigh, DMA_FIFOMode_Disable, 2*BUFFER_SIZE, I2S3_DR_Address, &buf1);		//TX

	dma_periph_inc_offset_size_config(DMA1_Stream0, DMA_PINCOS_Psize);
	dma_periph_inc_offset_size_config(DMA1_Stream5, DMA_PINCOS_Psize);

	dma_double_buffer_mode_config(DMA1_Stream0, &buf2, DMA_Memory_0);
	dma_double_buffer_mode_config(DMA1_Stream5, &buf2, DMA_Memory_0);

	dma_double_buffer_mode_cmd(DMA1_Stream0, ENABLE);
	dma_double_buffer_mode_cmd(DMA1_Stream5, ENABLE);

	dma_it_config(DMA1_Stream0, DMA_IT_TC, ENABLE);

	dma_cmd(DMA1_Stream0, ENABLE);
	dma_cmd(DMA1_Stream5, ENABLE);

	//I2S2 Config
	stm32::i2s_init(SPI3, I2S_Mode_MasterTx, I2S_Standard_Phillips, I2S_DataFormat_24b, I2S_MCLKOutput_Enable, I2S_AudioFreq_48k, I2S_CPOL_Low);
	i2s_full_duplex_config(I2S3ext, I2S_Mode_MasterTx, I2S_Standard_Phillips, I2S_DataFormat_24b, I2S_CPOL_Low);

	spi_i2s_dma_cmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	spi_i2s_dma_cmd(I2S3ext, SPI_I2S_DMAReq_Rx, ENABLE);

	i2s_cmd(I2S3ext, ENABLE);
	i2s_cmd(SPI3, ENABLE);
}


namespace {
	//Global buffer for float ops
	float fBuf[BUFFER_SIZE];
	float phase_acc = 0;
	constexpr auto sin_size = 64U;
	float sintable[ sin_size ];
	constexpr float FREQ = 1000;
	constexpr float SAMPLE_RATE = 48000;
	constexpr float PHI =  FREQ / SAMPLE_RATE * sin_size;
	constexpr float AMPL = 1<<16;

}






#if 1
extern "C"
void __attribute__ ((optimize(3), optimize("unroll-loops"))) dma1_stream0_isr_vector(void)
{
	using namespace stm32;
	if(dma_get_flag_status(DMA1_Stream0, DMA_FLAG_TCIF0))
	{
		signed long *slBuf1, *slBuf2;

		slBuf1 = slBuf2 = (dma_get_current_memory_target(DMA1_Stream0)) ? buf1 : buf2;

		for(unsigned long i = 0; i < BUFFER_SIZE; i++)
		{
			signed long tmp = *slBuf1++;

			asm volatile
			(
				"ROR %[result], %[data], #16 \n"
				: [result]"=&r"(tmp)
				: [data]"r"(tmp)
			);

			fBuf[i] = tmp;
		}

		for( int i=0; i< BUFFER_SIZE; i++ ) {
				fBuf[i] =  fBuf[i] * 3 + AMPL * sintable[ int(phase_acc) ];
				if( 1 ) {
					phase_acc += PHI;
					if( phase_acc >= sin_size ) {
						phase_acc -= sin_size;
					}
				}
		}

		//F32 to S32 Convert
		for(unsigned long i = 0; i < BUFFER_SIZE; i++)
		{
			signed long tmp = fBuf[i];

			asm volatile
			(
				"ROR %[result], %[data], #16 \n"
				: [result]"=&r"(tmp)
				: [data]"r"(tmp)
			);

			*slBuf2++ = tmp;
		}
		dma_clear_flag(DMA1_Stream0, DMA_FLAG_TCIF0);
	} else 	if(dma_get_flag_status(DMA1_Stream0, DMA_FLAG_TEIF0)) {
		dbg_err("DMA underflow error");
		abort();

	} else {
		dbg_err("DMA invalid request %08x %08x", DMA1->HISR, DMA1->LISR );
		abort();
	}
}
#endif


void codec_task( fnd::bus::ibus& bus )
{
	//Generate sine table
	for( size_t i=0; i<sin_size; ++i) {
		sintable[i] = std::sin( 2.0f * M_PI * float(i) / float(sin_size) ) + 1.0f;
	}
	//Test codec reprogram 100 times
	//for( int i=0; i<100; ++i ) {
	{
		if( codec_reset( bus ) ) {
			dbg_err("Codec Reset failed I2S config skipped");
			isix_wait_ms(100);
			return;
		}
		isix_wait_ms(10);
	}
	i2s_init();
}


}

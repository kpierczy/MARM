#include<stm32_ll_bus.h>
#include "L3GD20.h"
#include "L3GD20_reg.h"




bool L3GD20::isConected(){

    // Read WHO_AM_I register
    uint8_t who_am_i {};
    if(readRegisters(&who_am_i, L3GD20_WHO_AM_I, 1) < 0)
        return false;

    // If read value is not one of the expected, return false 
    if(who_am_i != 0xD3 && who_am_i != 0xD4 && who_am_i != 0xD7)
        return false;
    else
        return true;
}




int L3GD20::readControlRegisters(uint8_t* dst, uint8_t start_address, uint8_t num){
    
    // Check if start address is valid
    if(start_address < L3GD20_CTRL_REG1 || start_address > L3GD20_CTRL_REG5)
        return -1;

    // Cut number of registers to read to the range of the registers
    while(start_address + num - 1 > L3GD20_CTRL_REG5)
        --num;

    // Read registers
    if(readRegisters(dst, start_address, num) < 0)
        return -1;

    return num;
}




int L3GD20::writeControlRegisters(const uint8_t* src, uint8_t start_address, uint8_t num){
    
    // Check if start address is valid
    if(start_address < L3GD20_CTRL_REG1 || start_address > L3GD20_CTRL_REG5)
        return -1;

    // Cut number of registers to read to the range of the registers
    while(start_address + num - 1 > L3GD20_CTRL_REG5)
        --num;

    // Write registers
    if(writeRegisters(src, start_address, num) < 0)
        return -1;

    // Check settings for DPS
    for(int i = 0; i < num; ++i){
        if(start_address + i == L3GD20_CTRL_REG4){
            switch((src[i] & 0x30) >> 4){
                case 0b00:
                    resolution = 250;
                    break;
                case 0b01:
                    resolution = 500;
                    break;
                default:
                    resolution = 2000;
                    break;
            }
            break;
        }
    }

    return num;
}




int L3GD20::readMeasurementRegisters(float* dst, uint8_t start_address, uint8_t num){
    
    // Check if start address is valid
    if(start_address < L3GD20_OUT_X_L || start_address > L3GD20_OUT_Z_L)
        return -1;

    // Cut number of registers to read to the range of the registers
    while(start_address + num * 2 - 1 > L3GD20_OUT_Z_H)
        --num;

    // Read registers
    uint8_t measurement_read_input[6] {};
    if(readRegisters(measurement_read_input, start_address, num * 2) < 0)
        return -1;

    // Copy read registers to the 'dst' array
    for(int i = 0; i < num; ++i){
        int16_t tmp = (int16_t(measurement_read_input[i*2 + 1]) << 8) | int16_t(measurement_read_input[i*2]);
        dst[i] =  float(tmp) * resolution / INT16_MAX;
    }

    return num;    
}




#ifdef ISIX

L3GD20::L3GD20(const char* name, SPI_InitType_ISIX * init, uint32_t used_ports)
    : valid(true),
      spi(name),
      resolution(250)
{

    // Enable GPIOs clocks
    LL_AHB1_GRP1_EnableClock( used_ports );

    // Set SPI parameters
    if(spi.set_option(periph::option::speed(init->speed)) < 0){
        valid = false;
        return;
    }
    if(spi.set_option(periph::option::polarity(init->polarity)) < 0){
        valid = false;
        return;
    }
    if(spi.set_option(periph::option::phase(init->phase)) < 0){
        valid = false;
        return;
    }
    if(spi.set_option(periph::option::dwidth(init->dwidth)) < 0){
        valid = false;
        return;
    }
    if(spi.set_option(periph::option::bitorder(init->bitorder)) < 0){
        valid = false;
        return;
    }
    if(spi.open(init->timeout) < 0){
        valid = false;
        return;
    }
}




int L3GD20::readRegisters(uint8_t* dst, uint8_t start_address, uint8_t num){
        
    /**
     * Prepare structure for the mixed transaction. Extended
     * write transaction is required to maintain SCL signal
     * and make L3GD20 able to write registers' content to
     * the bus. These writes are empty.
     */      
    uint8_t output[50] = {};
    if(num == 1)
        output[0] =  start_address | L3GD20_READ | L3GD20_NO_AUTOINC;
    else
        output[0] =  start_address | L3GD20_READ | L3GD20_AUTOINC;

    uint8_t input[50] = {};
    static_assert(sizeof(input) == sizeof(output));
    periph::blk::trx_transfer tran(output, input, num + 1);

    // Perform transaction
    int ret = spi.transaction(0, tran);
    if(ret < 0)
        return -1;

    // Copy read registers to the 'dst' array
    for(int i = 0; i < num; ++i)
        dst[i] = input[i + 1];

    return num;
}




int L3GD20::writeRegisters(const uint8_t* src, uint8_t start_address, uint8_t num){
        
    // Prepare write structure
    uint8_t output[50] {};
    if(num == 1)
        output[0] =  start_address | L3GD20_WRITE | L3GD20_NO_AUTOINC;
    else
        output[0] =  start_address | L3GD20_WRITE | L3GD20_AUTOINC;
    for(int i = 0; i < num; ++i)
        output[i + 1] = src[i];
    periph::blk::tx_transfer tran(output, num + 1);

    // Perform transaction
    int ret = spi.transaction(0, tran);
    if(ret < 0)
        return -1;

    return num;
}

#else // ISIX endif


#endif // LL endif
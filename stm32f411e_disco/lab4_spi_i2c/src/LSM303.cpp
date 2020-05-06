/*================================================================================
 *
 *    Filename : LSM303.cpp
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Implementation of the LSM303 interface class. @see LSM303.h
 *
 *===============================================================================*/

#include<stm32_ll_bus.h>
#include "LSM303.h"
#include "LSM303_reg.h"




int LSM303::readControlRegisters(sensor sen, uint8_t* dst, uint8_t start_address, uint8_t num){
    
    int ret {};

    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_CTRL_REG1_A || start_address > LSM303_CTRL_REG6_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_CTRL_REG6_A)
            --num;
         
        // Perform read
        ret = readRegisters(dst, LSM303_ADDR_A, start_address, num);
        if(ret < 0)
            return ret;  
    }
    // Magnetometer configuration
    else if (sen == MAG){

        // Check start address range
        if(start_address < LSM303_CRA_REG_M || start_address > LSM303_MR_REG_M)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_MR_REG_M)
            --num;

        // Perform read
        ret = readRegisters(dst, LSM303_ADDR_M, start_address, num);
        if(ret < 0)
            return ret;          

    }
    // Invalid start address
    else
        return -1;

    return ret; 
}




int LSM303::writeControlRegisters(sensor sen, const uint8_t* src, uint8_t start_address, uint8_t num){
    
    int ret {};

    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_CTRL_REG1_A || start_address > LSM303_CTRL_REG6_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_CTRL_REG6_A)
            --num;

        // Perform transaction
        ret = writeRegisters(src, LSM303_ADDR_A, start_address, num);
        if(ret < 0)
            return ret;

        // Check settings for DPS
        for(int i = 0; i < num; ++i){
            if(start_address + i == LSM303_CTRL_REG4_A){
                switch((src[i] & 0x30) >> 4){
                    case 0b00:
                        range_a = 2;
                        break;
                    case 0b01:
                        range_a = 4;
                        break;
                    case 0b10:
                        range_a = 8;
                        break;
                    case 0b11:
                        range_a = 16;
                        break;
                }
                break;
            }
        }

    }
    // Magnetometer configuration
    else if (sen == MAG){

        // Check start address range
        if(start_address < LSM303_CRA_REG_M || start_address > LSM303_MR_REG_M)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_MR_REG_M)
            --num;

        // Perform transaction
        ret = writeRegisters(src, LSM303_ADDR_M, start_address, num);
        if(ret < 0)
            return ret;

        // Check settings for DPS
        for(int i = 0; i < num; ++i){
            if(start_address + i == LSM303_CRB_REG_M){
                switch((src[i] & 0xE0) >> 5){
                    case 0b001:
                        range_m = 1.3;
                        break;
                    case 0b010:
                        range_m = 1.9;
                        break;
                    case 0b011:
                        range_m = 2.5;
                        break;
                    case 0b100:
                        range_m = 4.0;
                        break;
                    case 0b101:
                        range_m = 4.7;
                        break;
                    case 0b110:
                        range_m = 5.6;
                        break;
                    case 0b111:
                        range_m = 8.1;
                        break;
                }
                break;
            }
        }

    }
    // Invalid start address
    else
        return -1;

    return ret;
}




int LSM303::readMeasurementRegisters(sensor sen, float* dst, uint8_t start_address, uint8_t num){
    
    int ret {};

    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_OUT_X_L_A || start_address > LSM303_OUT_Z_H_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num * 2 - 1 > LSM303_OUT_Z_H_A)
            --num;

        // Perform read
        uint8_t measurement_read_input[6] {};
        ret = readRegisters(measurement_read_input, LSM303_ADDR_A, start_address, num * 2);
        if(ret < 0)
            return ret; 

        // Copy read registers to the 'dst' array
        for(int i = 0; i < num; ++i){
            int16_t tmp = (int16_t(measurement_read_input[i*2 + 1]) << 8) | int16_t(measurement_read_input[i*2]);
            dst[i] =  float(tmp) * range_a / INT16_MAX;
        }

    }
    // Magnetometer configuration
    else if (sen == MAG){

        // Check start address range
        if(start_address < LSM303_OUT_X_H_M || start_address > LSM303_OUT_Z_L_M)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_OUT_Z_L_M)
            --num;

        // Perform read
        uint8_t measurement_read_input[6] {};
        ret = readRegisters(measurement_read_input, LSM303_ADDR_M, start_address, num * 2);
        if(ret < 0)
            return ret; 

        // Copy read registers to the 'dst' array
        for(int i = 0; i < num; ++i){
            int16_t tmp = (int16_t(measurement_read_input[i*2]) << 8) | int16_t(measurement_read_input[i*2 + 1]);
            dst[i] =  float(tmp) * range_m / INT16_MAX;
        }

    }
    // Invalid start address
    else
        return -1;

    return ret; 
}




float LSM303::getAccRange(){
    return range_a;
}

float LSM303::getMagRange(){
    return range_m;
}




#ifdef ISIX

LSM303::LSM303(const char* name, I2C_InitType_ISIX * init, uint32_t used_ports)
    : valid(true),
      i2c(name),
      range_a(2),
      range_m(1.3)
{

    // Enable GPIOs clocks
    LL_AHB1_GRP1_EnableClock( used_ports );

    // Set SPI parameters
    if(i2c.open(init->timeout)){
        valid = false;
        return;
    }
    if(i2c.set_option(periph::option::speed(init->speed))){
        valid = false;
        return;
    }

    return;
}




int LSM303::readRegisters(uint8_t* dst, uint8_t addr, uint8_t start_address, uint8_t num){
        
    /**
     * Prepare structure for the mixed transaction. Extended
     * write transaction is required to maintain SCL signal
     * and make LSM303 able to write registers' content to
     * the bus. These writes are empty.
     */      
    uint8_t output {};
    if(num == 1)
        output = start_address | LSM303_WRITE | LSM303_NO_AUTOINC ;
    else
        output = start_address | LSM303_WRITE | LSM303_AUTOINC ;
    periph::blk::trx_transfer tran(&output, dst, 1, num);

    // Perform transaction
    return i2c.transaction(addr, tran);

}




int LSM303::writeRegisters(const uint8_t* src, uint8_t addr, uint8_t start_address, uint8_t num){
        
    // Prepare structure for the output transaction.
    uint8_t output[7] {};
    if(num == 1)
        output[0] = start_address | LSM303_WRITE | LSM303_NO_AUTOINC ;
    else
        output[0] = start_address | LSM303_WRITE | LSM303_AUTOINC ;
    for(int i = 0; i < num; ++i)
        output[i + 1] = src[i];
    periph::blk::tx_transfer tran(output, num + 1);

    // Perform transaction
    return i2c.transaction(addr, tran);
}

#else // ISIX endif


#endif // LL endif
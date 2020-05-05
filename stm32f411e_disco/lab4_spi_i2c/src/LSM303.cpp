#include<stm32_ll_bus.h>
#include "LSM303.h"
#include "LSM303_reg.h"




int LSM303::readControlRegisters(sensor sen, uint8_t* dst, uint8_t start_address, uint8_t num){
    
    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_CTRL_REG1_A || start_address > LSM303_CTRL_REG6_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_CTRL_REG6_A)
            --num;

        // Perform read
        if(readRegisters(dst, LSM303_ADDR_A, start_address, num) < 0)
            return -1;             

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
        if(readRegisters(dst, LSM303_ADDR_M, start_address, num) < 0)
            return -1;          

    }
    // Invalid start address
    else
        return -1;

    return num; 
}




int LSM303::writeControlRegisters(sensor sen, const uint8_t* src, uint8_t start_address, uint8_t num){
    
    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_CTRL_REG1_A || start_address > LSM303_CTRL_REG6_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num - 1 > LSM303_CTRL_REG6_A)
            --num;

        // Perform transaction
        if(writeRegisters(src, LSM303_ADDR_A, start_address, num) < 0)
            return -1;

        // Check settings for DPS
        for(int i = 0; i < num; ++i){
            if(start_address + i == LSM303_CTRL_REG4_A){
                switch((src[i] & 0x30) >> 4){
                    case 0b00:
                        resolution_a = 2;
                        break;
                    case 0b01:
                        resolution_a = 4;
                        break;
                    case 0b10:
                        resolution_a = 8;
                        break;
                    case 0b11:
                        resolution_a = 16;
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
        // Perform transaction
        if(writeRegisters(src, LSM303_ADDR_M, start_address, num) < 0)
            return -1;

        // Check settings for DPS
        for(int i = 0; i < num; ++i){
            if(start_address + i == LSM303_CRB_REG_M){
                switch((src[i] & 0xE0) >> 5){
                    case 0b001:
                        resolution_m = 1.3;
                        break;
                    case 0b010:
                        resolution_m = 1.9;
                        break;
                    case 0b011:
                        resolution_m = 2.5;
                        break;
                    case 0b100:
                        resolution_m = 4.0;
                        break;
                    case 0b101:
                        resolution_m = 4.7;
                        break;
                    case 0b110:
                        resolution_m = 5.6;
                        break;
                    case 0b111:
                        resolution_m = 8.1;
                        break;
                }
                break;
            }
        }

    }
    // Invalid start address
    else
        return -1;

    return num;
}




int LSM303::readMeasurementRegisters(sensor sen, float* dst, uint8_t start_address, uint8_t num){
    
    // Accelerometer configuration
    if(sen == ACC){

        // Check start address range
        if(start_address < LSM303_OUT_X_L_A || start_address > LSM303_OUT_Z_H_A)
            return -1;

        // Cut number of registers to read to the range of the registers
        while(start_address + num * 2 - 1 > LSM303_OUT_Z_H_A)
            --num;

        // Perform read
        uint8_t measurement_read_input[6] = {};
        if(readRegisters(measurement_read_input, LSM303_ADDR_A, start_address, num * 2) < 0)
            return -1; 

        // Copy read registers to the 'dst' array
        for(int i = 0; i < num; ++i){
            int16_t tmp = (int16_t(measurement_read_input[i*2 + 1]) << 8) | int16_t(measurement_read_input[i*2]);
            dst[i] =  float(tmp) * resolution_a / INT16_MAX;
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

        // Perform read
        uint8_t measurement_read_input[6] = {};
        if(readRegisters(measurement_read_input, LSM303_ADDR_M, start_address, num * 2) < 0)
            return -1; 

        // Copy read registers to the 'dst' array
        for(int i = 0; i < num; ++i){
            int16_t tmp = (int16_t(measurement_read_input[i*2]) << 8) | int16_t(measurement_read_input[i*2 + 1]);
            dst[i] =  float(tmp) * resolution_m / INT16_MAX;
        }

    }
    // Invalid start address
    else
        return -1;

    return num; 
}




#ifdef ISIX

LSM303::LSM303(const char* name, I2C_InitType_ISIX * init, uint32_t used_ports)
    : valid(true),
      i2c(name),
      resolution_a(2),
      resolution_m(1.3)
{

    // Enable GPIOs clocks
    LL_AHB1_GRP1_EnableClock( used_ports );

    // Set SPI parameters
    if(i2c.set_option(periph::option::speed(init->speed)) < 0){
        valid = false;
        return;
    }
    if(i2c.open(init->timeout) < 0){
        valid = false;
        return;
    }
}




int LSM303::readRegisters(uint8_t* dst, uint8_t addr, uint8_t start_address, uint8_t num){
        
        /**
         * Prepare structure for the mixed transaction. Extended
         * write transaction is required to maintain SCL signal
         * and make LSM303 able to write registers' content to
         * the bus. These writes are empty.
         */      
        const uint8_t ouput = { start_address | LSM303_READ | LSM303_AUTOINC };
        periph::blk::trx_transfer tran(&ouput, dst, 1, num);

        // Perform transaction
        int ret = i2c.transaction(addr, tran);
        if(ret < 0)
            return -1;
}




int LSM303::writeRegisters(const uint8_t* src, uint8_t addr, uint8_t start_address, uint8_t num){
        
    // Prepare structure for the output transaction.
    uint8_t config_output[7] = { start_address | LSM303_WRITE | LSM303_AUTOINC };
    for(int i = 0; i < num; ++i)
        config_output[i + 1] = src[i];
    periph::blk::tx_transfer tran(config_output, num + 1);

    // Perform transaction
    int ret = i2c.transaction(addr, tran);
    if(ret < 0)
        return -1; 
}

#else // ISIX endif


#endif // LL endif
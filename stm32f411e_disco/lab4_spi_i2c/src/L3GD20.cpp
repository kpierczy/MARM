#include "L3GD20.h"
#include "L3GD20_reg.h"

#ifdef ISIX

L3GD20::L3GD20(const char* name, SPI_InitType_ISIX * init, uint32_t used_ports)
    : valid(true),
      spi(name),
      rate(250)
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





bool L3GD20::isConected(){

    /**
     *  Initialize structure to perform mixed transaction.
     *  Two bytes will be written and two ill be read.
     *
     *   (1) First read byte can be ommitted - it doesn't
     *       carry any information. The second byte is
     *       the value of the WHO_AM_I register
     * 
     *   (2) Second written byte can be ommitted - it is
     *       written only to maintain SCL signal to
     *       give L3GD20 opportunity to write register's
     *       content to the bus
     * 
     * For more detail see L3GD20's documentation (section 5.2.1,
     * SPI read).
     */ 
    const uint8_t scan_output[2] {L3GD20_WHO_AM_I | L3GD20_READ | L3GD20_NO_AUTOINC};
    uint8_t scan_input[2] {};
    static_assert(sizeof(scan_input) == sizeof(scan_output));
    periph::blk::trx_transfer scan_tran(scan_output, scan_input, sizeof(scan_output));

    // Perform transaction
    int ret = spi.transaction(0, scan_tran);
    if(ret < 0)
        return false;

    // If read value is not one of the expected, return false 
    if(scan_input[1] != 0xD3 && scan_input[1] != 0xD4 && scan_input[1] != 0xD7)
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

    /**
     * Prepare structure for the mixed transaction. Extended
     * write transaction is required to maintain SCL signal
     * and make L3GD20 able to write registers' content to
     * the bus. These writes are empty.
     */      
    const uint8_t config_read_ouput[6] = { start_address | L3GD20_READ | L3GD20_AUTOINC };
    uint8_t config_read_input[6] = {};
    static_assert(sizeof(config_read_input) == sizeof(config_read_ouput));
    periph::blk::trx_transfer config_read_tran(config_read_ouput, config_read_input, num + 1);

    // Perform transaction
    int ret = spi.transaction(0, config_read_tran);
    if(ret < 0)
        return -1;

    // Copy read registers to the 'dst' array
    for(int i = 0; i < num; ++i)
        dst[i] = config_read_input[i + 1];

    return num;
}





int L3GD20::writeControlRegisters(const uint8_t* src, uint8_t start_address, uint8_t num){
    
    // Check if start address is valid
    if(start_address < L3GD20_CTRL_REG1 || start_address > L3GD20_CTRL_REG5)
        return -1;

    // Cut number of registers to read to the range of the registers
    while(start_address + num - 1 > L3GD20_CTRL_REG5)
        --num;

    // Prepare write structure
    uint8_t config_output[6] = { start_address | L3GD20_WRITE | L3GD20_AUTOINC };
    for(int i = 0; i < num; ++i)
        config_output[i + 1] = src[i];

    // Perform transaction
    periph::blk::tx_transfer config_tran(config_output, num + 1);
    int ret = spi.transaction(0, config_tran);
    if(ret < 0)
        return -1;

    // Check settings for DPS
    for(int i = 0; i < num; ++i){
        if(start_address + i == L3GD20_CTRL_REG4){
            switch((src[i] & 0x30) >> 4){
                case 0b00:
                    rate = 250;
                    break;
                case 0b01:
                    rate = 500;
                    break;
                default:
                    rate = 2000;
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

    /**
     * Prepare structure for the mixed transaction. Extended
     * write transaction is required to maintain SCL signal
     * and make L3GD20 able to write registers' content to
     * the bus. These writes are empty.
     */      
    const uint8_t measurement_read_ouput[7] = { start_address | L3GD20_READ | L3GD20_AUTOINC };
    uint8_t measurement_read_input[7] = {};
    static_assert(sizeof(measurement_read_input) == sizeof(measurement_read_ouput));
    periph::blk::trx_transfer measurement_read_tran(measurement_read_ouput, measurement_read_input, num * 2 + 1);

    // Perform transaction
    int ret = spi.transaction(0, measurement_read_tran);
    if(ret < 0)
        return -1;

    // Copy read registers to the 'dst' array
    for(int i = 0; i < num; ++i){
        int16_t tmp = (int16_t(measurement_read_input[i*2 + 2]) << 8) | int16_t(measurement_read_input[i*2 + 1]);
        dst[i] =  float(tmp) * rate / INT16_MAX;
    }

    return num;    
}

#else // ISIX endif

L3GD20::L3GD20(SPI_InitType * init, uint32_t used_ports){
    
}

#endif // LL endif
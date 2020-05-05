#ifndef LSM303_H
#define LSM303_H

#include <stdint.h>
#include <stm32_ll_i2c.h>
#include <periph/drivers/i2c/i2c_master.hpp>

class LSM303{

// Public types & constructors
public:

    enum sensor{
        ACC,
        MAG
    };

    typedef struct spi_init_isix_t{
        unsigned speed;
        unsigned timeout;
    } I2C_InitType_ISIX;

    typedef struct spi_init_t{
        I2C_TypeDef* I2Cx;
        uint32_t     speed;
        uint32_t     dutycycle;
        uint32_t     afilter;
        uint32_t     dfilter;
    } I2C_InitType;

    /**
     * Constructor using ISIX periph library
     * 
     * @param name : name of the I2C device registered in the dts
     * @param init : initialization structure with I2C's parameters
     * @param used_ports : flags of the ports being used by the I2C's pins
     *        given as LL ored macros
     */
    LSM303(const char* name, I2C_InitType_ISIX * init, uint32_t used_ports);

    /**
     * Constructor using bare-metal LL API
     * 
     * @param init : initialization structure with I2C's parameters
     * @param used_ports : flags of the ports being used by the I2C's pins
     *        given as LL ored macros
     * @note : fields of the I2C_InitType struct should be filled
     *         using LL macros
     */
    // LSM303(I2C_InitType * init, uint32_t used_ports);

// Public interface
public:


    /**
     * Reads required block of the control registers. If 
     * 'start_address' is out of the range of control registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of control registers, only
     * valid number of registers is read. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param sen [in] : sensor to operate on (accelertometer/magnetometer)
     * @param dst [out] : array that read registers will be written to
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise
     * 
     */
    int readControlRegisters(sensor sen, uint8_t* dst, uint8_t start_address, uint8_t num);

    /**
     * Writes required block of the control registers. If 
     * 'start_address' is out of the range of control registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of control registers, only
     * valid number of registers is written. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param sen [in] : sensor to operate on (accelertometer/magnetometer)
     * @param src [in] : array of values to write
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was written and
     *            negative value otherwise
     * 
     */
    int writeControlRegisters(sensor sen, const uint8_t* src, uint8_t start_address, uint8_t num);

    /**
     * Reads required block of the measurement registers. If 
     * 'start_address' is out of the range of measurement registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of measurement registers, only
     * valid number of registers is read. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param sen [in] : sensor to operate on (accelertometer/magnetometer)
     * @param dst [out] : array that read registers will be written to
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     */
    int readMeasurementRegisters(sensor sen, float* dst, uint8_t start_address, uint8_t num);

// Internal members
private:
    // Actual sensor's resolutions
    float resolution_a;
    float resolution_m;
    // Flag indicating if object was initialized properly (when ISIX used)
    bool valid;
    // Internal instance of the I2C driver (when ISIX used)
    periph::drivers::i2c_master i2c;

// Internal interface
private:

    /**
     * Performs simple I2C read from the registers of the device given
     * with 'addr' address. First address read is indicated with
     * 'start_address' and size of the block of registers with 'num'
     * 
     * @param dst [out] : array that read registers will be written to
     * @param addr [in] : address of the device
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     * 
     * @note : no data validation is performed. It is caller's duty to
     *         check if addresses' range is valid.
     */
    int readRegisters(uint8_t* dst, uint8_t addr, uint8_t start_address, uint8_t num);

    /**
     * Performs simple I2C write to the registers of the device given
     * with 'addr' address. First address read is indicated with
     * 'start_address' and size of the block of registers with 'num'
     * 
     * @param src [in] : array that read registers will be written with
     * @param addr [in] : address of the device
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     * 
     * @note : no data validation is performed. It is caller's duty to
     *         check if addresses' range and values are valid.
     */
    int writeRegisters(const uint8_t* src, uint8_t addr, uint8_t start_address, uint8_t num);

};

#endif

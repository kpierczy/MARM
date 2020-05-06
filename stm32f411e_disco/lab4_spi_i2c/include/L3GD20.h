/*================================================================================
 *
 *    Filename : L3GD20.h
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Header of the class interfacing L3GD20 gyroscope with a basic
 *               functions like reading and writting configuration and data
 *               registers via SPI interface.
 *              
 *               All public methods are safe in use, e.g. invalid arguments
 *               are discarded and an appropriate value is returned by the 
 *               methods.
 * 
 *               Class is implemented using ISIX SPI drivers but an appropriate
 *               interface is prepared to change these drivers to the are-metal
 *               functions working with Low Level library API.
 *
 *===============================================================================*/

#ifndef L3GD20_H
#define L3GD20_H

#include <stdint.h>
#include <stm32_ll_spi.h>
#include <periph/drivers/spi/spi_master.hpp>

class L3GD20{

// Public types & constructors
public:

    typedef struct spi_init_isix_t{
        unsigned                             speed;
        periph::option::polarity::_polarity  polarity;
        periph::option::phase::_phase        phase;
        unsigned char                        dwidth;
        enum periph::option::bitorder::order bitorder;
        unsigned                             timeout;
    } SPI_InitType_ISIX;

    typedef struct spi_init_t{
        SPI_TypeDef* SPIx;
        uint32_t     dwidth;
        uint32_t     polarity;
        uint32_t     phase;
        uint32_t     prescaler;
        uint32_t     bitorder;
    } SPI_InitType;

    /**
     * Constructor using ISIX periph library
     * 
     * @param name : name of the SPI device registered in the dts
     * @param init : initialization structure with SPI's parameters
     * @param used_ports : flags of the ports being used by the SPI's pins
     *        given as LL ored macros
     */
    L3GD20(const char* name, SPI_InitType_ISIX * init, uint32_t used_ports);

    /**
     * Constructor using bare-metal LL API
     * 
     * @param init : initialization structure with SPI's parameters
     * @param used_ports : flags of the ports being used by the SPI's pins
     *        given as LL ored macros
     * @note : fields of the SPI_InitType struct should be filled
     *         using LL macros
     */
    // L3GD20(SPI_InitType * init, uint32_t used_ports);

// Public interface
public:

    /**
     * Reads WHO_AM_I register. If register's value is valid
     * L3GD20 gyroscope is properly connected to the bus
     * and 'true' value is returned. Otherwise some problems
     * have been met and communication with module is not
     * possible. Then, false is returned.
     * 
     * @returns : true, if L3GD20 is properly connected to the bus
     */
    bool isConected();

    /**
     * Reads required block of the control registers. If 
     * 'start_address' is out of the range of control registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of control registers, only
     * valid number of registers is read. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param dst [out] : array that read registers will be written to
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise
     * 
     */
    int readControlRegisters(uint8_t* dst, uint8_t start_address, uint8_t num);

    /**
     * Writes required block of the control registers. If 
     * 'start_address' is out of the range of control registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of control registers, only
     * valid number of registers is written. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param src [in] : array of values to write
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was written and
     *            negative value otherwise
     * 
     */
    int writeControlRegisters(const uint8_t* src, uint8_t start_address, uint8_t num);

    /**
     * Reads required block of the measurement registers. If 
     * 'start_address' is out of the range of measurement registers'
     * space, no action is taken.
     * Otherwise, if 'num' indicates block of addresses that
     * is not a subset of the set of measurement registers, only
     * valid number of registers is read. Other cells of the
     * 'dst' array are not modified.
     * 
     * @param dst [out] : array that read registers will be written to
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     */
    int readMeasurementRegisters(float* dst, uint8_t start_address, uint8_t num);

    /**
     * @returns : actual sensor's range
     */
    float getRange();

// Internal members
private:
    // Actual sensor's range
    float range;
    // Flag indicating if object was initialized properly (when ISIX used)
    bool valid;
    // Internal instance of the SPI driver (when ISIX used)
    periph::drivers::spi_master spi;

// Internal interface
private:

    /**
     * Performs simple SPI read from the registers given with
     * 'start__address' and size of the block 'num'
     * 
     * @param dst [out] : array that read registers will be written to
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     * 
     * @note : no data validation is performed. It is caller's duty to
     *         check if addresses' range is valid.
     */
    int readRegisters(uint8_t* dst, uint8_t start_address, uint8_t num);

    /**
     * Performs simple SPI write to the registers given with
     * 'start__address' and size of the block 'num'
     * 
     * @param src [in] : array that read registers will be written with
     * @param start_address [in] : address of the first register
     * @param num [in] : number of registers to read
     * @returns : number of registers that was read and
     *            negative value otherwise 
     * 
     * @note : no data validation is performed. It is caller's duty to
     *         check if addresses' range and values are valid.
     */
    int writeRegisters(const uint8_t* src, uint8_t start_address, uint8_t num);

};

#endif
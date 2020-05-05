#include <stdint.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_spi.h>
#include <periph/drivers/spi/spi_master.hpp>

class L3GD20{

// Common internal members
private:
    float rate;
    bool valid;
    periph::drivers::spi_master spi;

// ISIX-specific public types & constructors
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
     * @param used_ports : flags of the ports being used by the SPI's pins
     *        given as LL ored macros
     * @param init : initialization structure with SPI's parameters
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

};
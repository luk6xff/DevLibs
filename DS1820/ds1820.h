/**
 *  @brief:   DS18B20 sensor library.
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2019-12-03
 *  @license: MIT
 */


#ifndef __DS1820_H__
#define __DS1820_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    THIS,   // Command applies to only this device
    ALL     // Command applies to all devices
} ds1820_devices;

typedef enum
{
    CELSIUS,     // Celsius degre
    FAHRENHEIT   // Fahrenheit degree
} ds1820_scale;

/**
 * @brief DS1820 dev object
 */
typedef struct
{

    /**
     * @brief Number of sensors on the bus
     */
    size_t sensors_num;

    /**
     * @brief ROM is a copy of the internal DS1820's ROM
     *        It is created during the ds1820_search_rom() or ds1820_search_alarm() commands
     *
     *        ds1820_rom[0] is the Dallas Family Code
     *        ds1820_rom[1] thru ds1820_rom[6] is the 48-bit unique serial number
     *        ds1820_rom[7] is the device CRC
    */
    uint8_t rom[8];

    /**
     * @brief RAM is a copy of the internal DS1820's RAM
     *        It's updated during the ds1820_read_ram() command
     *        which is automaticaly called from any function
     *        using the RAM values.
     */
    uint8_t ram[9];

    bool done_flag;
    int  last_descrepancy;
    uint8_t search_ds1820_rom[8];
    bool parasite_power;
    void* platform_dev;
} ds1820;




//------------------------------------------------------------------------------
// @brief FUNCTIONS
//------------------------------------------------------------------------------
/**
 * @brief This function initializes sensor state
 *
 * @param[in] parasite_powered - If device parsite-powered
 */
void ds1820_init(ds1820 *const dev, bool parasite_powered);

/**
 * @brief This function copies the DS1820's RAM into the object's
 *        RAM[].
 */
void ds1820_read_ram(ds1820 *const dev);

/**
 * @brief This routine initializes the global variables used in
 *        the ds1820_search_rom() and ds1820_search_alarm() funtions. It should
 *        be called once before looping to find devices.
 */
void ds1820_search_rom_setup(ds1820 *const dev);

/**
 * @brief This routine will search for an unidentified device
 *        on the bus. It uses the variables in ds1820_search_rom_setup
 *        to remember the pervious ROM address found.
 *        It will return FALSE if there were no new devices
 *        discovered on the bus.
 */
bool ds1820_search_rom(ds1820 *const dev);

/**
 * @brief This routine will search for an unidentified device
 *        which has the temperature alarm bit set. It uses the
 *        variables in ds1820_search_rom_setup to remember the pervious
 *        ROM address found. It will return FALSE if there were
 *        no new devices with alarms discovered on the bus.
 */
bool ds1820_search_alarm(ds1820 *const dev);

/**
 * @brief This routine will read the ROM (Family code, serial number
 *        and Checksum) from a dedicated device on the bus.
 *
 * @note This command can only be used when there is only one
 *       DS1820 on the bus. If this command is used when there
 *       is more than one slave present on the bus, a data
 *       collision will occur when all the DS1820s attempt to
 *       respond at the same time.
 */
void ds1820_read_rom(ds1820 *const dev);

/**
 * @brief This routine will initiate the temperature conversion within
 *        a DS1820. There is a built in 750ms delay to allow the
 *        conversion to complete.
 *
 *        To update all sensors on the bus, use a statement such as this:
 *        sensor[0]->ds1820_convert_temperature(DS1820::ALL);
 *
 * @param allows the fnction to apply to a specific device or
 *        to all devices on the 1-Wire bus.
 */
void ds1820_convert_temperature(ds1820 *const dev, ds1820_devices device);

/**
 * @brief This function will return the sensor temperature. This function
 *        uses the count remainding values to interpolate the temperature
 *        to about 1/150th of a degree. Whereas the sensor is not spec to
 *        that precision. It does seem to give a smooth reading to the
 *        tenth of a degree.
 *
 * @param scale, may be either CELSIUS or FAHRENHEIT
 * @returns temperature for that scale
 */
float ds1820_read_temperature(ds1820 *const dev, ds1820_scale scale);

/**
 * @brief This function calculates the ROM checksum and compares it to the
 *        CRC value stored in ROM[7].
 *
 * @returns true if the checksum matches, otherwise false.
 */
bool ds1820_rom_checksum_error(ds1820 *const dev);

/**
 * @brief This function calculates the RAM checksum and compares it to the
 *        CRC value stored in RAM[8].
 *
 * @returns true if the checksum matches, otherwise false.
 */
bool ds1820_ram_checksum_error(ds1820 *const dev);

/**
 * @brief This function sets the temperature resolution for the DS18B20
 *        in the configuration register.
 *
 * @param a number between 9 and 12 to specify the resolution
 * @returns true if successful
 */
bool ds1820_set_configuration_bits(ds1820 *const dev, unsigned int resolution);

/**
 * @brief This function returns the values stored in the temperature
 *        alarm registers.
 *
 * @returns a 16 bit integer of TH (upper byte) and TL (lower byte).
 */
int ds1820_read_scratchpad(ds1820 *const dev);

/**
 * @brief This function will store the passed data into the DS1820's RAM.
 * @note  It does NOT save the data to the EEPROM for retention
 *        during cycling the power off and on.
 *
 * @param a 16 bit integer of TH (upper byte) and TL (lower byte).
 */
void ds1820_write_scratchpad(ds1820 *const dev, int data);

/**
 * @brief This function will transfer the TH and TL registers from the
 *        DS1820's RAM into the EEPROM.
 * @note  There is a built in 10ms delay to allow for the
 *        completion of the EEPROM write cycle.
 *
 * @param allows the fnction to apply to a specific device or
 *        to all devices on the 1-Wire bus.
 */
void ds1820_store_scratchpad(ds1820 *const dev, ds1820_devices device);

/**
 * @brief This function will copy the stored values from the EEPROM
 *        into the DS1820's RAM locations for TH and TL.
 *
 * @param allows the function to apply to a specific device or
 *        to all devices on the 1-Wire bus.
 */
int ds1820_recall_scratchpad(ds1820 *const dev, ds1820_devices device);

/**
 * @brief This function will return the type of power supply for
 *        a specific device. It can also be used to query all devices
 *        looking for any device that is parasite powered.
 *
 * @returns true if the device (or all devices) are Vcc powered,
 *          returns false if the device (or ANY device) is parasite powered.
 */
bool ds1820_read_power_supply(ds1820 *const dev, ds1820_devices device);



//-----------------------------------------------------------------------------
// @brief HW DEPENDENT FUNCTIONS - must be defined for each platform
//-----------------------------------------------------------------------------
extern bool ds1820_onewire_reset(ds1820 *const dev);
extern void ds1820_onewire_bit_out(ds1820 *const dev, bool bit_data);
extern bool ds1820_onewire_bit_in(ds1820 *const dev);
/**
 * @brief Enable/Disable power parasite pin.
 *
 * @param enable  Set ot HIGH if true, LOW otherwise.
 */
extern void ds1820_set_parasite_pin(ds1820 *const dev, bool enable);

extern void ds1820_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif


#endif // __DS1820_H__

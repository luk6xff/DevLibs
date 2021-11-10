#include "ds1820.h"

//------------------------------------------------------------------------------
// @brief MACROS
//------------------------------------------------------------------------------
#define FAMILY_CODE dev->rom[0]
#define FAMILY_CODE_DS1820 0x10
#define FAMILY_CODE_DS18S20 0x10
#define FAMILY_CODE_DS18B20 0x28


//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS
//------------------------------------------------------------------------------
static uint8_t crc_byte (uint8_t crc, uint8_t byte)
{
    int j;
    for(j=0; j<8; j++)
    {
        if ((byte & 0x01 ) ^ (crc & 0x01))
        {
            // DATA ^ LSB crc = 1
            crc = crc >> 1;
            // Set the MSB to 1
            crc = crc | 0x80;
            // Check bit 3
            if (crc & 0x04)
            {
                crc = crc & 0xFB; // Bit 3 is set, so clear it
            } else
            {
                crc = crc | 0x04; // Bit 3 is clear, so set it
            }
            // Check bit 4
            if (crc & 0x08)
            {
                crc = crc & 0xF7; // Bit 4 is set, so clear it
            } else
            {
                crc = crc | 0x08; // Bit 4 is clear, so set it
            }
        }
        else
        {
            // DATA ^ LSB crc = 0
            crc = crc>>1;
            // clear MSB
            crc = crc & 0x7F;
            // No need to check bits, with DATA ^ LSB crc = 0, they will remain unchanged
        }
        byte = byte >> 1;
    }
    return crc;
}

//------------------------------------------------------------------------------
static void onewire_byte_out(ds1820 *const dev, uint8_t data)
{
    // Output data uint8_tacter (least sig bit first).
    for (int n=0; n<8; n++)
    {
        ds1820_onewire_bit_out(dev, data & 0x01);
        data = data >> 1; // now the next bit is in the least sig bit position.
    }
}

//------------------------------------------------------------------------------
static uint8_t onewire_byte_in(ds1820 *const dev)
{   // Read byte, least sig byte first
    uint8_t answer = 0x00;
    for (int i=0; i<8; i++)
    {
        answer = answer >> 1; // shift over to make room for the next bit
        if (ds1820_onewire_bit_in(dev))
            answer = answer | 0x80; // if the data port is high, make this bit a 1
    }
    return answer;
}

//------------------------------------------------------------------------------
static void match_rom(ds1820 *const dev)
{
    // Used to select a specific device
    int i;
    ds1820_onewire_reset(dev);
    onewire_byte_out(dev, 0x55);  //Match ROM command
    for (i=0; i<8; i++)
    {
        onewire_byte_out(dev, dev->rom[i]);
    }
}

//------------------------------------------------------------------------------
static void skip_rom(ds1820 *const dev)
{
    ds1820_onewire_reset(dev);
    onewire_byte_out(dev, 0xCC);   // Skip ROM command
}

//------------------------------------------------------------------------------
bool ds1820_search_rom_routine(ds1820 *const dev, uint8_t command)
{
    int descrepancy_marker, rom_bit_index;
    bool return_value, bit_a, bit_b;
    uint8_t byte_counter, bit_mask;

    return_value=false;
    if (!dev->done_flag)
    {
        if (!ds1820_onewire_reset(dev))
        {
            dev->last_descrepancy = 0; // no devices present
        }
        else
        {
            rom_bit_index = 1;
            descrepancy_marker = 0;
            onewire_byte_out(dev, command); // Search ROM command or Search Alarm command
            byte_counter = 0;
            bit_mask = 0x01;
            while (rom_bit_index <= 64)
            {
                bit_a = ds1820_onewire_bit_in(dev);
                bit_b = ds1820_onewire_bit_in(dev);
                if (bit_a & bit_b)
                {
                    descrepancy_marker = 0; // data read error, this should never happen
                    rom_bit_index = 0xFF;
                }
                else
                {
                    if (bit_a | bit_b)
                    {
                        // Set ROM bit to bit_a
                        if (bit_a)
                        {
                            dev->search_ds1820_rom[byte_counter] = dev->search_ds1820_rom[byte_counter] | bit_mask; // Set ROM bit to one
                        }
                        else
                        {
                            dev->search_ds1820_rom[byte_counter] = dev->search_ds1820_rom[byte_counter] & ~bit_mask; // Set ROM bit to zero
                        }
                    }
                    else
                    {
                        // both bits A and B are low, so there are two or more devices present
                        if (rom_bit_index == dev->last_descrepancy)
                        {
                            dev->search_ds1820_rom[byte_counter] = dev->search_ds1820_rom[byte_counter] | bit_mask; // Set ROM bit to one
                        }
                        else
                        {
                            if (rom_bit_index > dev->last_descrepancy)
                            {
                                dev->search_ds1820_rom[byte_counter] = dev->search_ds1820_rom[byte_counter] & ~bit_mask; // Set ROM bit to zero
                                descrepancy_marker = rom_bit_index;
                            }
                            else
                            {
                                if ((dev->search_ds1820_rom[byte_counter] & bit_mask) == 0x00)
                                {
                                    descrepancy_marker = rom_bit_index;
                                }
                            }
                        }
                    }
                    ds1820_onewire_bit_out(dev, dev->search_ds1820_rom[byte_counter] & bit_mask);
                    rom_bit_index++;
                    if (bit_mask & 0x80)
                    {
                        byte_counter++;
                        bit_mask = 0x01;
                    }
                    else
                    {
                        bit_mask = bit_mask << 1;
                    }
                }
            }
            dev->last_descrepancy = descrepancy_marker;
            if (rom_bit_index != 0xFF)
            {
                for(byte_counter=0;byte_counter<8;byte_counter++)
                {
                    dev->rom[byte_counter] = dev->search_ds1820_rom[byte_counter];
                }
                return_value = true;
            }
        }
        if (dev->last_descrepancy == 0)
        {
            dev->done_flag = true;
        }
    }
    return return_value;
}

//------------------------------------------------------------------------------
// @brief PUBLIC FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------
void ds1820_init(ds1820 *const dev, bool parasite_powered)
{
    dev->parasite_power = parasite_powered;

    for(int i=0; i<8; i++)
    {
        dev->rom[i] = 0xFF;
    }
    for(int i=0; i<9; i++)
    {
        dev->ram[i] = 0x00;
    }
    // Initialize global state variables
    ds1820_search_rom_setup(dev);

    // Search for new device
    size_t sensors_found = 0;
    while (ds1820_search_rom(dev) && sensors_found < dev->sensors_num)
    {
        sensors_found++;
    }

}

//------------------------------------------------------------------------------
bool ds1820_search_rom(ds1820 *const dev)
{
    return ds1820_search_rom_routine(dev, 0xF0);    // Search ROM command
}

//------------------------------------------------------------------------------
bool ds1820_search_alarm(ds1820 *const dev)
{
    return ds1820_search_rom_routine(dev, 0xEC);    // Search Alarm command
}

//------------------------------------------------------------------------------
void ds1820_search_rom_setup(ds1820 *const dev)
{
    dev->done_flag = false;
    dev->last_descrepancy = 0;
    int i;
    for (i=0; i<8; i++)
    {
        dev->search_ds1820_rom[i]=0x00;
    }
}

//------------------------------------------------------------------------------
void ds1820_read_rom(ds1820 *const dev)
{
    // NOTE: This command can only be used when there is one DS1820 on the bus. If this command
    // is used when there is more than one slave present on the bus, a data collision will occur
    // when all the DS1820s attempt to respond at the same time.
    int i;
    ds1820_onewire_reset(dev);
    onewire_byte_out(dev, 0x33);   // Read ROM id
    for (i=0; i<8; i++)
    {
        dev->rom[i] = onewire_byte_in(dev);
    }
}


//------------------------------------------------------------------------------
bool ds1820_rom_checksum_error(ds1820 *const dev)
{
    uint8_t crc=0x00;
    int i;
    for(i=0;i<7;i++) // Only going to shift the lower 7 bytes
        crc = crc_byte(crc, dev->rom[i]);
    // After 7 bytes crc should equal the 8th byte (ROM crc)
    return (crc != dev->rom[7]); // will return true if there is a crc checksum error
}

//------------------------------------------------------------------------------
bool ds1820_ram_checksum_error(ds1820 *const dev)
{
    uint8_t crc=0x00;
    int i;
    ds1820_read_ram(dev);
    for(i=0;i<8;i++) // Only going to shift the lower 8 bytes
    {
        crc = crc_byte(crc, dev->ram[i]);
    }
    // After 8 bytes crc should equal the 9th byte (RAM crc)
    return (crc != dev->ram[8]); // will return true if there is a crc checksum error
}



//------------------------------------------------------------------------------
void ds1820_convert_temperature(ds1820 *const dev, ds1820_devices device)
{
    // Convert temperature into scratchpad RAM for all devices at once
    int delay_time = 750; // Default delay time
    uint8_t resolution;
    if (device == ALL)
    {
        skip_rom(dev);          // Skip ROM command, will convert for ALL devices
    }
    else
    {
        match_rom(dev);
        if (FAMILY_CODE == FAMILY_CODE_DS18B20)
        {
            resolution = dev->ram[4] & 0x60;
            if (resolution == 0x00) // 9 bits
            {
                delay_time = 94;
            }
            else if (resolution == 0x20) // 10 bits
            {
                delay_time = 188;
            }
            else if (resolution == 0x40) // 11 bits. Note 12bits uses the 750ms default
            {
                delay_time = 375;
            }
        }
    }
    onewire_byte_out(dev, 0x44);  // perform temperature conversion
    if (dev->parasite_power)
    {
        ds1820_set_parasite_pin(dev, true);       // Parasite power strong pullup
    }
    ds1820_delay_us(delay_time*1000);
    if (dev->parasite_power)
    {
        ds1820_set_parasite_pin(dev, false);
    }
}

//------------------------------------------------------------------------------
void ds1820_read_ram(ds1820 *const dev)
{
    // This will copy the DS1820's 9 bytes of RAM data
    // into the objects RAM array. Functions that use
    // RAM values will automaticly call this procedure.
    int i;
    match_rom(dev);             // Select this device
    onewire_byte_out(dev, 0xBE);  //Read Scratchpad command
    for(i=0; i<9; i++)
    {
        dev->ram[i] = onewire_byte_in(dev);
    }
}

//------------------------------------------------------------------------------
bool ds1820_set_configuration_bits(ds1820 *const dev, unsigned int resolution)
{
    bool answer = false;
    resolution = resolution - 9;
    if (resolution < 4)
    {
        resolution = resolution << 5; // align the bits
        dev->ram[4] = (dev->ram[4] & 0x60) | resolution; // mask out old data, insert new
        ds1820_write_scratchpad(dev, (dev->ram[2]<<8) + dev->ram[3]);
        //ds1820_store_scratchpad (THIS);
        answer = true;
    }
    return answer;
}

//------------------------------------------------------------------------------
int ds1820_read_scratchpad(ds1820 *const dev)
{
    int answer;
    ds1820_read_ram(dev);
    answer = (dev->ram[2]<<8) + dev->ram[3];
    return answer;
}

//------------------------------------------------------------------------------
void ds1820_write_scratchpad(ds1820 *const dev, int data)
{
    dev->ram[3] = data;
    dev->ram[2] = data>>8;
    match_rom(dev);
    onewire_byte_out(dev, 0x4E);   // Copy scratchpad into DS1820 ram memory
    onewire_byte_out(dev, dev->ram[2]); // T(H)
    onewire_byte_out(dev, dev->ram[3]); // T(L)
    if (FAMILY_CODE == FAMILY_CODE_DS18B20)
    {
        onewire_byte_out(dev, dev->ram[4]); // Configuration register
    }
}

//------------------------------------------------------------------------------
void ds1820_store_scratchpad(ds1820 *const dev, ds1820_devices device)
{
    if (device == ALL)
    {
        skip_rom(dev);          // Skip ROM command, will store for ALL devices
    }
    else
    {
        match_rom(dev);
    }
    onewire_byte_out(dev, 0x48);   // Write scratchpad into E2 command
    if (dev->parasite_power)
    {
        ds1820_set_parasite_pin(dev, true);
    }
    ds1820_delay_us(10*1000);            // Parasite power strong pullup for 10ms
    if (dev->parasite_power)
    {
        ds1820_set_parasite_pin(dev, false);
    }
}

//------------------------------------------------------------------------------
int ds1820_recall_scratchpad(ds1820 *const dev, ds1820_devices device)
{
    // This copies the E2 values into the DS1820's memory.
    // If you specify ALL this will return zero, otherwise
    // it will return the value of the scratchpad memory.
    int answer=0;
    if (device==ALL)
    {
        skip_rom(dev);          // Skip ROM command, will recall for ALL devices
    }
    else
    {
        match_rom(dev);
    }
    onewire_byte_out(dev, 0xB8);   // Recall E2 data to scratchpad command
    ds1820_delay_us(10*1000); // not sure I like polling for completion
                 // it could cause an infinite loop
    if (device==THIS)
    {
        ds1820_read_ram(dev);
        answer = ds1820_read_scratchpad(dev);
    }
    return answer;
}

//------------------------------------------------------------------------------
float ds1820_read_temperature(ds1820 *const dev, ds1820_scale scale)
{
    float answer, remaining_count, count_per_degree;
    int reading;
    ds1820_read_ram(dev);
    reading = (dev->ram[1] << 8) + dev->ram[0];
    if (reading & 0x8000) // negative degrees C
    {
        reading = 0-((reading ^ 0xffff) + 1); // 2's comp then convert to signed int
    }
    answer = reading +0.0; // convert to floating point
    if (FAMILY_CODE == FAMILY_CODE_DS18B20)
    {
        answer = answer / 8.0;
    }
    else
    {
        remaining_count = dev->ram[6];
        count_per_degree = dev->ram[7];
        answer = answer - 0.25 + (count_per_degree - remaining_count) / count_per_degree;
    }
    if (scale == CELSIUS)
    {
        answer = answer / 2.0;
    }
    else // FAHRENHEIT
    {
        answer = answer * 9.0 / 10.0 + 32.0;
    }
    return answer;
}

//------------------------------------------------------------------------------
bool ds1820_read_power_supply(ds1820 *const dev, ds1820_devices device)
{
    // This will return true if the device (or all devices) are Vcc powered
    // This will return false if the device (or ANY device) is parasite powered
    if (device==ALL)
    {
        skip_rom(dev);          // Skip ROM command, will poll for any device using parasite power
    }
    else
    {
        match_rom(dev);
    }
    onewire_byte_out(dev, 0xB4);   // Read power supply command
    return ds1820_onewire_bit_in(dev);
}

//------------------------------------------------------------------------------
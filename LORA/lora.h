/**
 *  @brief:  LoRa library for SEMTECH SX127x devices.
 *           Code deeply based on: https://github.com/sandeepmistry/arduino-LoRa
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */

#ifndef __LORA_H__
#define __LORA_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>


#define LORA_MAX_PKT_LEN           255

/**
 * @brief Hardware IO IRQ callback function definition
 */
typedef void (*dio_irq_cb)(void*);

/**
 * @brief LoRa dev object
 */
typedef struct
{
    dio_irq_cb dio_irq;               // Callbacks for pins from dio0 to dio5
    uint8_t rx_tx_buffer[LORA_MAX_PKT_LEN];   // Data reception buffer
    long frequency;
    volatile int packet_index;
    int implicit_header_mode;
    void (*on_receive)(void*, int);
    void (*on_tx_done)(void*);
    void *platform_dev;                     // Platform dependent data
} lora;


bool lora_init(lora *const dev);
void lora_end(lora *const dev);

int lora_begin_packet(lora *const dev, int implicitHeader);
int lora_end_packet(lora *const dev, bool async);

int lora_parse_packet(lora *const dev, int size);
int lora_packet_rssi(lora *const dev);
float lora_packet_snr(lora *const dev);
long lora_packet_frequency_error(lora *const dev);

int lora_rssi();

uint32_t lora_write(lora *const dev, uint8_t byte);
uint32_t lora_write_data(lora *const dev, const uint8_t *buffer, uint32_t size);

int lora_available(lora *const dev);
int lora_read(lora *const dev);
int lora_peek(lora *const dev);
void lora_flush(lora *const dev);

void lora_on_receive(lora *const dev, void(*callback)(void*, int));
void lora_on_tx_done(lora *const dev, void(*callback)(void*));

void lora_receive(lora *const dev, int size);

void lora_idle(lora *const dev);
void lora_sleep(lora *const dev);

void lora_set_tx_power(lora *const dev, int level, int outputPin);
void lora_set_frequency(lora *const dev, long frequency);
void lora_set_spreading_factor(lora *const dev, int sf);
void lora_set_signal_bandwidth(lora *const dev, long sbw);
void lora_set_coding_rate4(lora *const dev, int denominator);
void lora_set_preamble_length(lora *const dev, long length);
void lora_set_sync_word(lora *const dev, int sw);
void lora_enable_crc(lora *const dev);
void lora_disable_crc(lora *const dev);
void lora_enable_invert_IQ(lora *const dev);
void lora_disable_invert_IQ(lora *const dev);

void lora_set_ocp(lora *const dev, uint8_t mA); // Over Current Protection control

void lora_set_gain(lora *const dev, uint8_t gain); // Set LNA gain

uint8_t lora_random(lora *const dev);

void lora_dump_registers(lora *const dev);

void lora_explicit_header_mode(lora *const dev);
void lora_implicit_header_mode(lora *const dev);

void lora_handle_dio0_rise(lora *const dev);
bool lora_is_transmitting(lora *const dev);

int lora_get_spreading_factor(lora *const dev);
long lora_get_signal_bandwidth(lora *const dev);

void lora_set_ldo_flag(lora *const dev);

/**
 * @brief Writes the radio register at the specified address
 *
 * @param[in]: addr Register address
 * @param[in]: data New register value
 */
void lora_write_reg(lora *const dev, uint8_t addr, uint8_t data);

/**
 * @brief Reads the radio register at the specified address
 *
 * @param[in]: addr Register address
 * @retval data Register value
 */
uint8_t lora_read_reg(lora *const dev, uint8_t addr);




//-------------------------------------------------------------------------
//  Board/Platform relative (HW dependent) functions,
//  Must be implemented in folder platform.
//-------------------------------------------------------------------------

/**
 * @brief Initializes the radio I/Os pins interface
 */
extern void lora_io_init(lora *const dev);

/**
 * @brief De-initializes the radio I/Os pins interface.
 *
 * @note Useful when going in MCU lowpower modes
 */
extern void lora_io_deinit(lora *const dev);

/**
 * @brief Initializes DIO IRQ handlers
 */
extern void lora_ioirq_init(lora *const dev);

/**
 * @brief Deinitializes DIO IRQ handlers
 */
extern void lora_ioirq_deinit(lora *const dev);

/**
 * @brief Resets the LORA device
 */
extern void lora_reset(lora *const dev);

/**
 * @brief Writes multiple radio registers starting at address
 *
 * @param[in] addr   First Radio register address
 * @param[in] buffer Buffer containing the new register's values
 * @param[in] size   Number of registers to be written
 */
extern void lora_write_buffer(lora *const dev, uint8_t addr, const uint8_t *buffer, const uint8_t size);

/**
 * @brief Reads multiple radio registers starting at address
 *
 * @param[in] addr First Radio register address
 * @param[out] buffer Buffer where to copy the registers data
 * @param[in] size Number of registers to be read
 */
extern void lora_read_buffer(lora *const dev, uint8_t addr, uint8_t *buffer, uint8_t size);

/**
 * @brief A simple ms sleep
 */
extern void lora_delay_ms(int ms);

/**
 * @brief Read uptime in miliseconds
 */
extern uint32_t lora_timer_read_ms();

/**
 * @brief Print all the registers
 */
extern void lora_dump_registers(lora* const dev);


#ifdef __cplusplus
}
#endif

#endif // __LORA_H__

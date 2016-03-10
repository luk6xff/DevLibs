#ifndef MPU6050_H_
#define MPU6050_H_

#include <mbed.h>
#include "I2Cdriver.h"




#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)


#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)
#define MPU6050_FLASH_SIZE      (512)
#define MPU6050_FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)


#define MAX_PACKET_LENGTH (12)
// Seven-bit device address is 110100 for ADO = 0(GND) and 110101 for ADO = 1(+3.3V)
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif
#define MPU6050_DEFAULT_INT_PIN PC_6


//REGISTERS MAP
#define MPU6050_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC     0x07
#define MPU6050_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC     0x09
#define MPU6050_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC     0x0B
#define MPU6050_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU6050_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USRL     0x14
#define MPU6050_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USRL     0x16
#define MPU6050_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_ZG_OFFS_USRL     0x18
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_FF_THR           0x1D
#define MPU6050_FF_DUR           0x1E
#define MPU6050_MOT_THR          0x1F
#define MPU6050_MOT_DUR          0x20
#define MPU6050_ZRMOT_THR        0x21
#define MPU6050_ZRMOT_DUR        0x22
#define MPU6050_FIFO_EN          0x23
#define MPU6050_I2C_MST_CTRL     0x24
#define MPU6050_I2C_SLV0_ADDR    0x25
#define MPU6050_I2C_SLV0_REG     0x26
#define MPU6050_I2C_SLV0_CTRL    0x27
#define MPU6050_I2C_SLV1_ADDR    0x28
#define MPU6050_I2C_SLV1_REG     0x29
#define MPU6050_I2C_SLV1_CTRL    0x2A
#define MPU6050_I2C_SLV2_ADDR    0x2B
#define MPU6050_I2C_SLV2_REG     0x2C
#define MPU6050_I2C_SLV2_CTRL    0x2D
#define MPU6050_I2C_SLV3_ADDR    0x2E
#define MPU6050_I2C_SLV3_REG     0x2F
#define MPU6050_I2C_SLV3_CTRL    0x30
#define MPU6050_I2C_SLV4_ADDR    0x31
#define MPU6050_I2C_SLV4_REG     0x32
#define MPU6050_I2C_SLV4_DO      0x33
#define MPU6050_I2C_SLV4_CTRL    0x34
#define MPU6050_I2C_SLV4_DI      0x35
#define MPU6050_I2C_MST_STATUS   0x36
#define MPU6050_INT_PIN_CFG      0x37
#define MPU6050_INT_ENABLE       0x38
#define MPU6050_DMP_INT_STATUS   0x39
#define MPU6050_INT_STATUS       0x3A
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_ACCEL_XOUT_L     0x3C
#define MPU6050_ACCEL_YOUT_H     0x3D
#define MPU6050_ACCEL_YOUT_L     0x3E
#define MPU6050_ACCEL_ZOUT_H     0x3F
#define MPU6050_ACCEL_ZOUT_L     0x40
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_TEMP_OUT_L       0x42
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_GYRO_XOUT_L      0x44
#define MPU6050_GYRO_YOUT_H      0x45
#define MPU6050_GYRO_YOUT_L      0x46
#define MPU6050_GYRO_ZOUT_H      0x47
#define MPU6050_GYRO_ZOUT_L      0x48
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60
#define MPU6050_MOT_DETECT_STATUS    0x61
#define MPU6050_I2C_SLV0_DO      0x63
#define MPU6050_I2C_SLV1_DO      0x64
#define MPU6050_I2C_SLV2_DO      0x65
#define MPU6050_I2C_SLV3_DO      0x66
#define MPU6050_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_SIGNAL_PATH_RESET    0x68
#define MPU6050_MOT_DETECT_CTRL      0x69
#define MPU6050_USER_CTRL        0x6A
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_PWR_MGMT_2       0x6C
#define MPU6050_BANK_SEL         0x6D
#define MPU6050_MEM_START_ADDR   0x6E
#define MPU6050_MEM_R_W          0x6F
#define MPU6050_DMP_CFG_1        0x70
#define MPU6050_DMP_CFG_2        0x71
#define MPU6050_FIFO_COUNTH      0x72
#define MPU6050_FIFO_COUNTL      0x73
#define MPU6050_FIFO_R_W        0x74
#define MPU6050_WHO_AM_I        0x75 //sould return 0x68    



//BIT MASKS in REGISTER BYTES
#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)


    /* When entering motion interrupt mode, the driver keeps track of the
     * previous state so that it can be restored at a later time.
     * TODO: This is tacky. Fix it.
     */
    struct motion_int_cache_s {
        uint16_t gyro_fsr;
        uint8_t accel_fsr;
        uint16_t lpf;
        uint16_t sample_rate;
        uint8_t sensors_on;
        uint8_t fifo_sensors;
        uint8_t dmp_on;
    };
     
    /* Information specific to a particular device. */
    typedef struct {
        uint8_t addr;
        uint16_t max_fifo;
        uint8_t num_reg;
        uint16_t temp_sens;
        int16_t temp_offset;
        uint16_t bank_size;
    } hw_s;       
    
    /* Cached chip configuration data.
     * TODO: A lot of these can be handled with a bitmask.
     */
    typedef struct{
        /* Matches gyro_cfg >> 3 & 0x03 */
        uint8_t gyro_fsr;
        /* Matches accel_cfg >> 3 & 0x03 */
        uint8_t accel_fsr;
        /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
        uint8_t sensors;
        /* Matches config register. */
        uint8_t lpf;
        uint8_t clk_src;
        /* Sample rate, NOT rate divider. */
        uint16_t sample_rate;
        /* Matches fifo_en register. */
        uint8_t fifo_enable;
        /* Matches int enable register. */
        uint8_t int_enable;
        /* 1 if devices on auxiliary I2C bus appear on the primary. */
        uint8_t bypass_mode;
        /* 1 if half-sensitivity.
         * NOTE: This doesn't beint32_t here, but everything else in hw_s is const,
         * and this allows us to save some precious RAM.
         */
        uint8_t accel_half;
        /* 1 if device in low-power accel-only mode. */
        uint8_t lp_accel_mode;
        /* 1 if interrupts are only triggered on motion events. */
        uint8_t int_motion_only;
        struct motion_int_cache_s cache;
        /* 1 for active low interrupts. */
        uint8_t active_low_int;
        /* 1 for latched interrupts. */
        uint8_t latched_int;
        /* 1 if DMP is enabled. */
        uint8_t dmp_on;
        /* Ensures that DMP will only be loaded once. */
        uint8_t dmp_loaded;
        /* Sampling rate used when DMP is enabled. */
        uint16_t dmp_sample_rate;
    }chip_cfg_s;

    
    /* Information for self-test. */
    typedef struct{
        uint32_t gyro_sens;
        uint32_t accel_sens;
        uint8_t reg_rate_div;
        uint8_t reg_lpf;
        uint8_t reg_gyro_fsr;
        uint8_t reg_accel_fsr;
        uint16_t wait_ms;
        uint8_t packet_thresh;
        float min_dps;
        float max_dps;
        float max_gyro_var;
        float min_g;
        float max_g;
        float max_accel_var;
    } test_s ;
    

    /* Gyro driver state variables. */
    typedef struct {
        const hw_s *hw;
        chip_cfg_s chip_cfg;
        const test_s *test;
    }gyro_state_s ;


class MPU6050
{
    private:
    I2Cdriver i2cDrv;
   // InterruptIn intPin; //interrupt Pin !! TODOO
    gyro_state_s* st;
   
    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t dataLength, uint8_t *data);
    bool readByte(uint8_t devAddr, uint8_t regAddr,uint8_t *destData);
    bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t dataLength, uint8_t * destData);
    
    public:
        
        static const test_s test;
        static const hw_s hw ;
              
        /* Filter configurations. */
        enum lpf_e {
            INV_FILTER_256HZ_NOLPF2 = 0,
            INV_FILTER_188HZ,
            INV_FILTER_98HZ,
            INV_FILTER_42HZ,
            INV_FILTER_20HZ,
            INV_FILTER_10HZ,
            INV_FILTER_5HZ,
            INV_FILTER_2100HZ_NOLPF,
            NUM_FILTER
        };

        /* Full scale ranges. */
        enum gyro_fsr_e {
            INV_FSR_250DPS = 0,
            INV_FSR_500DPS,
            INV_FSR_1000DPS,
            INV_FSR_2000DPS,
            NUM_GYRO_FSR
        };

        /* Full scale ranges. */
        enum accel_fsr_e {
            INV_FSR_2G = 0,
            INV_FSR_4G,
            INV_FSR_8G,
            INV_FSR_16G,
            NUM_ACCEL_FSR
        };
        
        /* Clock sources. */
        enum clock_sel_e {
            INV_CLK_INTERNAL = 0,
            INV_CLK_PLL,
            NUM_CLK
        };
        
        /* Low-power accel wakeup rates. */
        enum lp_accel_rate_e {
            INV_LPA_1_25HZ,
            INV_LPA_5HZ,
            INV_LPA_20HZ,
            INV_LPA_40HZ
        };
    
    
        MPU6050();
        MPU6050(PinName i2cSda, PinName i2cScl, PinName mpu6050IntPin, uint32_t i2cFreq= 100000);
        int getDeviceID();
        bool testConnection();
        
        void register_int_cb(void(*intcb)());
        int mpu_reset(void);
        
        int set_int_enable(uint8_t enable);
        int get_accel_prod_shift(float *st_shift);
        int accel_self_test(int32_t *bias_regular, int32_t *bias_st);
        int gyro_self_test(int32_t *bias_regular, int32_t *bias_st);
        int compass_self_test(void);
        int get_st_biases(int32_t *gyro, int32_t *accel, uint8_t hw_test);
        
        /* Set up APIs */
        int mpu_init(void);
        int mpu_init_slave(void);
        int mpu_set_bypass(uint8_t bypass_on);
        
        /* Configuration APIs */
        int mpu_lp_accel_mode(uint8_t rate);
        int mpu_lp_motion_interrupt(uint16_t thresh, uint8_t time,
            uint8_t lpa_freq);
        int mpu_set_int_level(uint8_t active_low);
        int mpu_set_int_latched(uint8_t enable);
        
        int mpu_set_dmp_state(uint8_t enable);
        int mpu_get_dmp_state(uint8_t *enabled);
        
        int mpu_get_lpf(uint16_t *lpf);
        int mpu_set_lpf(uint16_t lpf);
        
        int mpu_get_gyro_fsr(uint16_t *fsr);
        int mpu_set_gyro_fsr(uint16_t fsr);
        
        int mpu_get_accel_fsr(uint8_t *fsr);
        int mpu_set_accel_fsr(uint8_t fsr);
        
        int mpu_get_compass_fsr(uint16_t *fsr);
        
        int mpu_get_gyro_sens(float *sens);
        int mpu_get_accel_sens(uint16_t *sens);
        
        int mpu_get_sample_rate(uint16_t *rate);
        int mpu_set_sample_rate(uint16_t rate);
        
        int mpu_get_fifo_config(uint8_t *sensors);
        int mpu_configure_fifo(uint8_t sensors);
        
        int mpu_get_power_state(uint8_t *power_on);
        int mpu_set_sensors(uint8_t sensors);
        
        int mpu_set_gyro_bias_reg(int32_t * gyro_bias);
        int mpu_read_accel_bias(int32_t *accel_bias);
        int mpu_read_gyro_bias(int32_t *gyro_bias); 
        int mpu_set_accel_bias_reg(const int32_t *accel_bias);
        
        /* Data getter/setter APIs */
        int mpu_get_gyro_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_accel_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_compass_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_temperature(float *data, uint32_t *timestamp);
        
        int mpu_get_int_status(int16_t *status);
        int mpu_read_fifo(int16_t *gyro,int16_t *accel, uint32_t *timestamp,
            uint8_t *sensors, uint8_t *more);
        int mpu_read_fifo_stream(uint16_t length, uint8_t *data,
            uint8_t *more);
        int mpu_reset_fifo(void);
        
        int mpu_write_mem(uint16_t mem_addr, uint16_t length,
            uint8_t *data);
        int mpu_read_mem(uint16_t mem_addr, uint16_t length,
            uint8_t *data);
        int mpu_load_firmware(uint16_t length, const uint8_t *firmware,
            uint16_t start_addr, uint16_t sample_rate);
        
        int mpu_reg_dump(void);
        int mpu_read_reg(uint8_t reg, uint8_t *data);
        int mpu_run_self_test(int32_t *gyro, int32_t *accel);
        
        int accel_6500_self_test(int32_t *bias_regular, int32_t *bias_st, int debug);
        int gyro_6500_self_test(int32_t *bias_regular, int32_t *bias_st, int debug);
        int get_st_6500_biases(int32_t *gyro, int32_t *accel, uint8_t hw_test, int debug);
        int mpu_run_6500_self_test(int32_t *gyro, int32_t *accel, uint8_t debug);
        int mpu_register_tap_cb(void (*func)(uint8_t, uint8_t));

};


#endif  
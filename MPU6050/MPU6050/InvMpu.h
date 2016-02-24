#ifndef INVMPU_H_
#define INVMPU_H_

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




// Seven-bit device address is 110100 for ADO = 0(GND) and 110101 for ADO = 1(+3.3V)
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif


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

class InvMpu
{
    private:
    I2Cdriver i2cDrv;
    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t dataLength, uint8_t *data);
    bool readByte(uint8_t devAddr, uint8_t regAddr,uint8_t *destData);
    bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t dataLength, uint8_t * destData);
    
    public:
        InvMpu();
        InvMpu(PinName i2cSda, PinName i2cScl, uint32_t i2cFreq= 100000);
        int getDeviceID();
        bool testConnection();
        
        int mpu_reset(void);
        
        int set_int_enable(uint8_t enable);
        int get_accel_prod_shift(float *st_shift);
        int accel_self_test(int32_t *bias_regular, int32_t *bias_st);
        int gyro_self_test(int32_t *bias_regular, int32_t *bias_st);
        int compass_self_test(void);
        int get_st_biases(int32_t *gyro, int32_t *accel, uint8_t hw_test);
        int setup_compass(void);
        
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
        int mpu_get_compass_sample_rate(uint16_t *rate);
        int mpu_set_compass_sample_rate(uint16_t rate);
        
        int mpu_get_fifo_config(uint8_t *sensors);
        int mpu_configure_fifo(uint8_t sensors);
        
        int mpu_get_power_state(uint8_t *power_on);
        int mpu_set_sensors(uint8_t sensors);
        
        int mpu_read_6500_accel_bias(int32_t *accel_bias);
        int mpu_read_6500_gyro_bias(int32_t *gyro_bias);
        int mpu_set_gyro_bias_reg(int32_t * gyro_bias);
        int mpu_set_accel_bias_6500_reg(const int32_t *accel_bias);
        int mpu_read_6050_accel_bias(int32_t *accel_bias);
        int mpu_set_accel_bias_6050_reg(const int32_t *accel_bias);
        
        /* Data getter/setter APIs */
        int mpu_get_gyro_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_accel_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_compass_reg(int16_t *data, uint32_t *timestamp);
        int mpu_get_temperature(int32_t *data, uint32_t *timestamp);
        
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
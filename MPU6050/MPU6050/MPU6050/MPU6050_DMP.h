#ifndef MPU6050_DMP_H_
#define MPU6050_DMP_H_

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

#include "MPU6050.h"
#include "helper_3dmath.h"


/* To initialize the DMP:
 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
 * 2. Push the gyro and accel orientation matrix to the DMP.
 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
 *    executed unless the corresponding feature is enabled.
 * 4. Call dmp_enable_feature(mask) to enable different features.
 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
 * 6. Call any feature-specific control functions.
 *
 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
 * be called repeatedly to enable and disable the DMP at runtime.
 *
 * The following is a short summary of the features supported in the DMP
 * image provided in inv_mpu_dmp_motion_driver.c:
 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
 * 200Hz. Integrating the gyro data at higher rates reduces numerical
 * errors (compared to integration on the MCU at a lower sampling rate).
 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
 * an event at the four orientations where the screen should rotate.
 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
 * no motion.
 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
 */


class MPU6050_DMP : public MPU6050
{
    public:
    MPU6050_DMP();
    
    uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx);
    int dmp_get_packet_length();
    int dmpGetGravity(VectorFloat *v, Quaternion *q) ;
    int dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) ;
    void reset();
//    static
    int decode_gesture(uint8_t *gesture);
    
    /* Set up functions. */
    int dmp_load_motion_driver_firmware(void);
    int dmp_set_fifo_rate(uint16_t rate);
    int dmp_get_fifo_rate(uint16_t *rate);
    int dmp_enable_feature(uint16_t mask);
    int dmp_get_enabled_features(uint16_t *mask);
    int dmp_set_interrupt_mode(uint8_t mode);
    int dmp_set_orientation(uint16_t orient);
    int dmp_set_gyro_bias(int32_t *bias);
    int dmp_set_accel_bias(int32_t *bias);
    
    /* Tap functions. */
    int dmp_register_tap_cb(void (*func)(uint8_t, uint8_t));
    int dmp_set_tap_thresh(uint8_t axis, uint16_t thresh);
    int dmp_set_tap_axes(uint8_t axis);
    int dmp_set_tap_count(uint8_t min_taps);
    int dmp_set_tap_time(uint16_t time);
    int dmp_set_tap_time_multi(uint16_t time);
    int dmp_set_shake_reject_thresh(int32_t sf, uint16_t thresh);
    int dmp_set_shake_reject_time(uint16_t time);
    int dmp_set_shake_reject_timeout(uint16_t time);
    
    /* Android orientation functions. */
    int dmp_register_android_orient_cb(void (*func)(uint8_t));
    
    
    /* Pedometer functions. */
    int dmp_get_pedometer_step_count(uint32_t *count);
    int dmp_set_pedometer_step_count(uint32_t count);
    int dmp_get_pedometer_walk_time(uint32_t *time);
    int dmp_set_pedometer_walk_time(uint32_t time);
    
    
    /* Read function. This function should be called whenever the MPU interrupt is
     * detected.
     */
    int dmp_read_fifo(int16_t *gyro,int16_t *accel, int32_t *quat,
        uint32_t *timestamp,int16_t *sensors, uint8_t *more);
        
    private:
    /* DMP gyro calibration functions. */
    int dmp_enable_gyro_cal(uint8_t enable);
    
    /* LP quaternion functions. */
    int dmp_enable_lp_quat(uint8_t enable);
    int dmp_enable_6x_lp_quat(uint8_t enable);
};

#endif
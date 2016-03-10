#include "mbed.h"
#include "MPU6050_DMP.h"
#include "MPU6050test.h"
//MPU6050 INT - PC6
MPU6050test mpu6050;
//InvMpu invMpu;
MPU6050_DMP dmpMD;
Timer t;
Serial pc(USBTX, USBRX); // tx, rx


volatile bool mpuDataReady= false;

static void eventOccured(void)
{
    mpuDataReady= true;
}
InterruptIn intPin(MPU6050_DEFAULT_INT_PIN);


volatile uint32_t hal_timestamp = 0;


static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        pc.printf("Tap X+ ");
        break;
    case TAP_X_DOWN:
        pc.printf("Tap X- ");
        break;
    case TAP_Y_UP:
        pc.printf("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        pc.printf("Tap Y- ");
        break;
    case TAP_Z_UP:
        pc.printf("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        pc.printf("Tap Z- ");
        break;
    default:
        return;
    }
    pc.printf("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
    switch (orientation) {
    case ANDROID_ORIENT_PORTRAIT:
        pc.printf("Portrait\n");
        break;
    case ANDROID_ORIENT_LANDSCAPE:
        pc.printf("Landscape\n");
        break;
    case ANDROID_ORIENT_REVERSE_PORTRAIT:
        pc.printf("Reverse Portrait\n");
        break;
    case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        pc.printf("Reverse Landscape\n");
        break;
    default:
        return;
    }
}


static inline void run_self_test(void)
{
    int result;
    int32_t gyro[3], accel[3];

    result = dmpMD.mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
    pc.printf("Passed!\n");
        pc.printf("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        pc.printf("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/
        uint8_t i = 0;

        for(i = 0; i<3; i++) {
            gyro[i] = (int32_t)(gyro[i] * 32.8f); //convert to +-1000dps
            accel[i] *= 2048.f; //convert to +-16G
            accel[i] = accel[i] >> 16;
            gyro[i] = (int32_t)(gyro[i] >> 16);
        }

        dmpMD.mpu_set_gyro_bias_reg(gyro);
        dmpMD.mpu_set_accel_bias_reg(accel);

    }
    else {
            if (!(result & 0x1))
                pc.printf("Gyro failed.\n");
            if (!(result & 0x2))
                pc.printf("Accel failed.\n");
            if (!(result & 0x4))
                pc.printf("Compass failed.\n");
     }

}






int main()
{      

///////dmpMD.register_int_cb(&eventOccured);   
    intPin.fall(&eventOccured);
    float sum = 0;
    uint32_t sumCount = 0;
    pc.baud(9600);
    
    t.start();


    uint8_t accel_fsr,  new_temp = 0;
    uint16_t gyro_rate, gyro_fsr;
    uint32_t timestamp;

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami =dmpMD.getDeviceID(); //mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");
    
 #if 1
    

  
  int result = dmpMD.mpu_init();
  if (result) 
  {
      pc.printf("Could not initialize gyro.\n");
  }
  else
  {
     pc.printf("DMP initialized correctly.\n");
  }
  
    dmpMD.mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    dmpMD.mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    dmpMD.mpu_set_sample_rate(DEFAULT_MPU_HZ);

    /* Read back configuration in case it was set improperly. */
    dmpMD.mpu_get_sample_rate(&gyro_rate);
    dmpMD.mpu_get_gyro_fsr(&gyro_fsr);
    dmpMD.mpu_get_accel_fsr(&accel_fsr);

    run_self_test();
    int8_t gyroOrientation[9]= {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1};
    dmpMD.dmp_load_motion_driver_firmware();
    dmpMD.dmp_set_orientation(dmpMD.inv_orientation_matrix_to_scalar(gyroOrientation));
    dmpMD.dmp_register_tap_cb(tap_cb);
    dmpMD.dmp_register_android_orient_cb(android_orient_cb);

    dmpMD.dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |DMP_FEATURE_GYRO_CAL|DMP_FEATURE_PEDOMETER);
    dmpMD.dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    dmpMD.dmp_set_interrupt_mode(DMP_INT_GESTURE); // onTap and orient change

    dmpMD.mpu_set_dmp_state(1);  //enable dmp
    pc.printf("__INIT FINISHED__\n");
    while(1)
    {   
        float temp;
        if(dmpMD.mpu_get_temperature(&temp,NULL)>-1) 
        { // Temperature in degrees Centigrade
            pc.printf(" temperature = %f  C\n\r", temp);
        }
        else
            pc.printf(" temp read failedd  C\n\r");
            
        uint32_t steps;   
        if(!dmpMD.dmp_get_pedometer_step_count(&steps)) 
        {
            pc.printf(" steps = %d  \n\r", steps);
        }
        else
            pc.printf(" steps read failedd  \n\r");
            
            
        if( mpuDataReady)
        {
            /*int16_t gyro[3], accel[3], sensors;
            uint8_t more;
            int32_t quat[4];
            dmpMD.dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more);
            
            pc.printf("ax = %f", 1000*accel[0]);
            pc.printf(" ay = %f", 1000*accel[1]);
            pc.printf(" az = %f  mg\n\r", 1000*accel[2]);

            pc.printf("gx = %f", gyro[0]);
            pc.printf(" gy = %f", gyro[1]);
            pc.printf(" gz = %f  deg/s\n\r",gyro[2]);
            
            */
            mpuDataReady=false;
            wait_ms(500);
        }
        int16_t gyro[3], accel[3];
        dmpMD.mpu_get_gyro_reg(gyro, NULL);
        dmpMD.mpu_get_accel_reg(accel, NULL);
        pc.printf("ax = %f", 1000*accel[0]);
        pc.printf(" ay = %f", 1000*accel[1]);
        pc.printf(" az = %f  mg\n\r", 1000*accel[2]);

        pc.printf("gx = %f", gyro[0]);
        pc.printf(" gy = %f", gyro[1]);
        pc.printf(" gz = %f  deg/s\n\r",gyro[2]);
        
        wait_ms(100);

        
    }
 #endif    
    
#if 0
    if (whoami == 0x68) { // WHO_AM_I should always be 0x68
        pc.printf("MPU6050 is online...");
        wait(1);
        mpu6050.MPU6050testSelfTest(mpu6050.SelfTest); // Start by performing self test and reporting values
        pc.printf("x-axis self test: acceleration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[0]);
        pc.printf("% of factory value \n\r");
        pc.printf("y-axis self test: acceleration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[1]);
        pc.printf("% of factory value \n\r");
        pc.printf("z-axis self test: acceleration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[2]);
        pc.printf("% of factory value \n\r");
        pc.printf("x-axis self test: gyration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[3]);
        pc.printf("% of factory value \n\r");
        pc.printf("y-axis self test: gyration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[4]);
        pc.printf("% of factory value \n\r");
        pc.printf("z-axis self test: gyration trim within : ");
        pc.printf("%f", mpu6050.SelfTest[5]);
        pc.printf("% of factory value \n\r");
        wait(1);

        if(mpu6050.SelfTest[0] < 1.0f && mpu6050.SelfTest[1] < 1.0f && mpu6050.SelfTest[2] < 1.0f && mpu6050.SelfTest[3] < 1.0f && mpu6050.SelfTest[4] < 1.0f && mpu6050.SelfTest[5] < 1.0f) {
            mpu6050.initHelperVariables();
            mpu6050.resetMPU6050test(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050test(mpu6050.gyroBias, mpu6050.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050test();
            pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
            wait(2);
        } else {
            pc.printf("Device did not the pass self-test!\n\r");
        }
    } else {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }



    while(1) {

        // If data ready bit set, all data registers have new data
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
            // check if data ready interrupt
            mpu6050.readAccelData(mpu6050.accelCount);  // Read the x/y/z adc values
            mpu6050.getAres();

            // Now we'll calculate the accleration value into actual g's
            mpu6050.ax = (float)mpu6050.accelCount[0]*mpu6050.aRes - mpu6050.accelBias[0];  // get actual g value, this depends on scale being set
            mpu6050.ay = (float)mpu6050.accelCount[1]*mpu6050.aRes - mpu6050.accelBias[1];
            mpu6050.az = (float)mpu6050.accelCount[2]*mpu6050.aRes - mpu6050.accelBias[2];

            mpu6050.readGyroData(mpu6050.gyroCount);  // Read the x/y/z adc values
            mpu6050.getGres();

            // Calculate the gyro value into actual degrees per second
            mpu6050.gx = (float)mpu6050.gyroCount[0]*mpu6050.gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            mpu6050.gy = (float)mpu6050.gyroCount[1]*mpu6050.gRes; // - gyroBias[1];
            mpu6050.gz = (float)mpu6050.gyroCount[2]*mpu6050.gRes; // - gyroBias[2];

            mpu6050.tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
            mpu6050.temperature = (mpu6050.tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
        }

        mpu6050.Now = t.read_us();
        mpu6050.deltat = (float)((mpu6050.Now - mpu6050.lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
        mpu6050.lastUpdate = mpu6050.Now;

        sum += mpu6050.deltat;
        sumCount++;

        if(mpu6050.lastUpdate - mpu6050.firstUpdate > 10000000.0f) {
            mpu6050.beta = 0.04;  // decrease filter gain after stabilized
            mpu6050.zeta = 0.015; // increasey bias drift gain after stabilized
        }

        // Pass gyro rate as rad/s
        mpu6050.MadgwickQuaternionUpdate(mpu6050.ax, mpu6050.ay, mpu6050.az, mpu6050.gx*mpu6050.PI/180.0f, mpu6050.gy*mpu6050.PI/180.0f, mpu6050.gz*mpu6050.PI/180.0f);

        // Serial print and/or display at 2 s rate independent of data rates
        mpu6050.delt_t = t.read_ms() - mpu6050.count;
        if (mpu6050.delt_t > 2000) { 
            pc.printf("\n\r-------------------------\n\r");
            pc.printf("ax = %f", 1000*mpu6050.ax);
            pc.printf(" ay = %f", 1000*mpu6050.ay);
            pc.printf(" az = %f  mg\n\r", 1000*mpu6050.az);

            pc.printf("gx = %f", mpu6050.gx);
            pc.printf(" gy = %f", mpu6050.gy);
            pc.printf(" gz = %f  deg/s\n\r", mpu6050.gz);

            pc.printf(" temperature = %f  C\n\r", mpu6050.temperature);

            pc.printf("q0 = %f\n\r", mpu6050.q[0]);
            pc.printf("q1 = %f\n\r", mpu6050.q[1]);
            pc.printf("q2 = %f\n\r", mpu6050.q[2]);
            pc.printf("q3 = %f\n\r", mpu6050.q[3]);



            // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
            // In this coordinate system, the positive z-axis is down toward Earth.
            // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
            // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
            // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
            // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
            // applied in the correct order which for this configuration is yaw, pitch, and then roll.
            // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
            mpu6050.yaw   = atan2(2.0f * (mpu6050.q[1] * mpu6050.q[2] + mpu6050.q[0] * mpu6050.q[3]), mpu6050.q[0] * mpu6050.q[0] + mpu6050.q[1] * mpu6050.q[1] - mpu6050.q[2] * mpu6050.q[2] - mpu6050.q[3] * mpu6050.q[3]);
            mpu6050.pitch = -asin(2.0f * (mpu6050.q[1] * mpu6050.q[3] - mpu6050.q[0] * mpu6050.q[2]));
            mpu6050.roll  = atan2(2.0f * (mpu6050.q[0] * mpu6050.q[1] + mpu6050.q[2] * mpu6050.q[3]), mpu6050.q[0] * mpu6050.q[0] - mpu6050.q[1] * mpu6050.q[1] - mpu6050.q[2] * mpu6050.q[2] + mpu6050.q[3] * mpu6050.q[3]);
            mpu6050.pitch *= 180.0f / mpu6050.PI;
            mpu6050.yaw   *= 180.0f / mpu6050.PI;
            mpu6050.roll  *= 180.0f / mpu6050.PI;


            pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", mpu6050.yaw, mpu6050.pitch, mpu6050.roll);
            pc.printf("average rate = %f Hz\n\r", (float) sumCount/sum);

            //myled= !myled;
            mpu6050.count = t.read_ms();
            sum = 0;
            sumCount = 0;
        }
    }
  #endif  
    
}
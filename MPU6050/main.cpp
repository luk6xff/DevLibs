#include "mbed.h"
#include "MPU6050.h"
//MPU6050 INT - PC6
MPU6050 mpu6050;
Timer t;
Serial pc(USBTX, USBRX); // tx, rx

int main()
{
    float sum = 0;
    uint32_t sumCount = 0;

    pc.baud(9600);
    
    t.start();

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68) { // WHO_AM_I should always be 0x68
        pc.printf("MPU6050 is online...");
        wait(1);
        mpu6050.MPU6050SelfTest(mpu6050.SelfTest); // Start by performing self test and reporting values
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
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(mpu6050.gyroBias, mpu6050.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050();
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
}
/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1
*/ 


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
//                          INITIAL SETUP                       
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================

// Constants for the complementary filter
const float alpha1 = 0.96; // α1 coefficient
const float alpha2 = 0.04; // α2 coefficient
const float dt = 0.01;     // Time interval (in seconds) between sensor readings

// Variables to store previous pitch angle and gyroscope data
float prevPitch = 0.0;
float gyroPitch = 0.0;

void loop() {
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float roll = ypr[2] * 180/M_PI;
            float pitchAcc = ypr[1] * 180/M_PI; // Calculated pitch from accelerometer
            float yaw = ypr[0] * 180/M_PI;

            // Calculate gyro-based pitch angle
            gyroPitch += ypr[1] * dt;

            // Apply complementary filter
            float pitch = alpha1 * (prevPitch + gyroPitch * dt) + alpha2 * pitchAcc;

            // Update previous pitch for next iteration
            prevPitch = pitch;

            Serial.print(roll); // roll
            Serial.print(",");
            Serial.print(pitch); // pitch
            Serial.print(",");
            Serial.println(yaw); // yaw
        #endif
    }
}

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h" // Include Kalman filter library

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Kalman filter objects for pitch and roll angles
KalmanFilter kalmanPitch(0.001, 0.003, 0.03); // Adjust parameters as needed
KalmanFilter kalmanRoll(0.001, 0.003, 0.03); // Adjust parameters as needed

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

void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float roll = ypr[2] * 180/M_PI;
            float pitch = ypr[1] * 180/M_PI;

            // Update Kalman filters with pitch and roll angles
            float filteredPitch = kalmanPitch.update(pitch, 0); // Assuming gyro rate is 0 here
            float filteredRoll = kalmanRoll.update(roll, 0); // Assuming gyro rate is 0 here

            // Output filtered pitch and roll angles
            Serial.print("Filtered Pitch: ");
            Serial.print(filteredPitch);
            Serial.print("\tFiltered Roll: ");
            Serial.println(filteredRoll);
        #endif
    }
}

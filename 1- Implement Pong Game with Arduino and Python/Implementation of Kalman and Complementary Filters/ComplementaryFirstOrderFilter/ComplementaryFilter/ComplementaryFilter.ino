/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1
*/ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
double freq, dt;
double tau = 0.98;
double rollCom = 0;
double pitchCom = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// quaternion components in a [w, x, y, z] format
//#define OUTPUT_READABLE_QUATERNION

// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 accel;


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

      // Reset the loop timer
  loopTimer = micros();
  loopTimer2 = micros();
}


// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================

void loop() {

  freq = 1/((micros() - loopTimer2) * 1e-6);
  loopTimer2 = micros();
  dt = 1/freq;
  
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print(q.w);
            Serial.print(",");
            Serial.print(q.x);
            Serial.print(",");
            Serial.print(q.y);
            Serial.print(",");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetAccel(&accel, fifoBuffer);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            float rotation_x = ypr[2] ;
            float rotation_y = ypr[1] ;
            float yaw = ypr[0] * 180/M_PI;

            // Complementary filter
            accelPitch = atan2(accel.y, accel.z) * RAD_TO_DEG;
            accelRoll = atan2(accel.x, accel.z) * RAD_TO_DEG;
          
            pitchCom = (tau)*(pitchCom + rotation_x*dt) + (1-tau)*(accelRoll);
            rollCom = (tau)*(rollCom - rotation_y*dt) + (1-tau)*(accelPitch);
          
//            Serial.print(freq,0);   
//            Serial.print(",");
            Serial.print(rotation_x* RAD_TO_DEG); // roll
            Serial.print(",");
            Serial.print(rotation_y* RAD_TO_DEG); // pitch
            Serial.print(",");
            Serial.print(rollCom); // roll
            Serial.print(",");
            Serial.println(pitchCom); // pitch
            //Serial.print(",");
            //Serial.println(yaw); // yaw
        #endif

    }
    
  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();
}

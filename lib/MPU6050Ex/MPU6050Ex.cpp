
#include "MPU6050Ex.h"

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady()
{
    mpuInterrupt = true;
}

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

namespace QuadBoi
{
    // ================================================================
    // ===                       Initialize                         ===
    // ================================================================

    void MPU6050Ex::initializeEx(uint8_t interruptPin)
    {
        
        m_interruptPin = interruptPin;

    // initialize device
        Serial.println(F("Initializing I2C devices..."));
        initialize();
        pinMode(m_interruptPin, INPUT);

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        setXGyroOffset(90);
        setYGyroOffset(-23);
        setZGyroOffset(10);
        setXAccelOffset(-1578);
        setYAccelOffset(-529);
        setZAccelOffset(1474);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            CalibrateAccel(6);
            CalibrateGyro(6);
            PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(m_interruptPin));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(m_interruptPin), dmpDataReady, RISING);
            mpuIntStatus = getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }   
    }

    void MPU6050Ex::processPacketAndPrint()
    {
        // if programming failed, don't try to do anything
        if (!dmpReady) return;
        // read a packet from FIFO
        if (dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            #ifdef OUTPUT_READABLE_QUATERNION
                // display quaternion values in easy matrix form: w x y z
                dmpGetQuaternion(&q, fifoBuffer);
                Serial.print("quat\t");
                Serial.print(q.w);
                Serial.print("\t");
                Serial.print(q.x);
                Serial.print("\t");
                Serial.print(q.y);
                Serial.print("\t");
                Serial.println(q.z);
            #endif

            #ifdef OUTPUT_READABLE_EULER
                // display Euler angles in degrees
                dmpGetQuaternion(&q, fifoBuffer);
                dmpGetEuler(euler, &q);
                Serial.print("euler\t");
                Serial.print(euler[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(euler[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(euler[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                dmpGetQuaternion(&q, fifoBuffer);
                dmpGetGravity(&gravity, &q);
                dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("ypr\t");
                Serial.print(ypr[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(ypr[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                dmpGetQuaternion(&q, fifoBuffer);
                dmpGetAccel(&aa, fifoBuffer);
                dmpGetGravity(&gravity, &q);
                dmpGetLinearAccel(&aaReal, &aa, &gravity);
                Serial.print("areal\t");
                Serial.print(aaReal.x);
                Serial.print("\t");
                Serial.print(aaReal.y);
                Serial.print("\t");
                Serial.println(aaReal.z);
            #endif

            #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                dmpGetQuaternion(&q, fifoBuffer);
                dmpGetAccel(&aa, fifoBuffer);
                dmpGetGravity(&gravity, &q);
                dmpGetLinearAccel(&aaReal, &aa, &gravity);
                dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                Serial.print("aworld\t");
                Serial.print(aaWorld.x);
                Serial.print("\t");
                Serial.print(aaWorld.y);
                Serial.print("\t");
                Serial.println(aaWorld.z);
            #endif
        }
    }
};
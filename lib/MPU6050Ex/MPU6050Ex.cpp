
#include "MPU6050Ex.h"

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady()
{
    mpuInterrupt = true;
}


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
};
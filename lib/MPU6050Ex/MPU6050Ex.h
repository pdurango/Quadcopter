#ifndef _MPU6050Ex_H_
#define _MPU6050Ex_H_
#include "MPU6050_6Axis_MotionApps20.h"


namespace QuadBoi
{
    class MPU6050Ex : public MPU6050
    {
    public:
        //MPU6050Ex() : MPU6050() {};
        void initializeEx(uint8_t interruptPin);

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
    private:

        // MPU control/status vars
        uint8_t m_interruptPin;
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
    };
};
#endif
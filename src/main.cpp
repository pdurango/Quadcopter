#include <Arduino.h>

#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050Ex.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
QuadBoi::MPU6050Ex mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

Servo esc_m1;
Servo esc_m2;

// macros
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define POD_PIN_M1 A1
#define POD_PIN_M2 A2
#define ESC_PIN_M1 3
#define ESC_PIN_M2 4

//allowed motor speeds - 0-180 range
const int MIN_ALLOWED_MOTOR_SPEED = 45; //25% 
const int MAX_ALLOWED_MOTOR_SPEED = 135; //75% 
const int MOTOR_SPEED_INCREMENT = 10;

// additional vars
int potValue_m1;
int potValue_m2;
int motorSpeed_m1; //0-180
int motorSpeed_m2; //0-180

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    //Initalize mpu
    mpu.initializeEx(INTERRUPT_PIN);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    esc_m1.attach(ESC_PIN_M1,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
    esc_m2.attach(ESC_PIN_M2,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

    potValue_m1 = 0;
    potValue_m2 = 0;

    //Initialze motors - must start at 0
    motorSpeed_m1 = 0;
    motorSpeed_m2 = 0;

    esc_m1.write(motorSpeed_m1);
    esc_m2.write(motorSpeed_m2);
    delay(3000);
    motorSpeed_m1 = motorSpeed_m2 = MIN_ALLOWED_MOTOR_SPEED;
    esc_m1.write(motorSpeed_m1);
    esc_m2.write(motorSpeed_m2);
}

int increaseMotorSpeed(int currentSpeed)
{
    //dont let motors go past max speed
    int newSpeed = (currentSpeed + MOTOR_SPEED_INCREMENT >= MAX_ALLOWED_MOTOR_SPEED) 
        ? MAX_ALLOWED_MOTOR_SPEED : currentSpeed + MOTOR_SPEED_INCREMENT;
    return newSpeed;
}

int decreaseMotorSpeed(int currentSpeed)
{
    //dont let motors go below min speed
    int newSpeed = (currentSpeed - MOTOR_SPEED_INCREMENT <= MIN_ALLOWED_MOTOR_SPEED) 
        ? MIN_ALLOWED_MOTOR_SPEED : currentSpeed - MOTOR_SPEED_INCREMENT;
    return newSpeed;
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
    /* 
    * Gets current dmp packet, processes it, and prints mpu values
    * depending on which macro is enabled.
    */
    mpu.processPacketAndPrint();

    /////////////////////////////////////////////
    //Control 2 motors via yaw
    float yaw = mpu.ypr[0] * 180/M_PI;
    if(yaw >= 0) //rear POV - leaning right - right motor needs to increase speed
    {            
        motorSpeed_m1 = decreaseMotorSpeed(motorSpeed_m1);
        motorSpeed_m2 = increaseMotorSpeed(motorSpeed_m2);
    }
    else //rear POV - leaning left - left motor needs to increase speed
    {
        motorSpeed_m1 = increaseMotorSpeed(motorSpeed_m1);
        motorSpeed_m2 = decreaseMotorSpeed(motorSpeed_m2);
    }

    esc_m1.write(motorSpeed_m1);
    esc_m2.write(motorSpeed_m2);
    Serial.print("Motor 1 speed:\t");
    Serial.print(motorSpeed_m1);
    Serial.print("\t");
    Serial.print("Motor 2 speed:\t");
    Serial.println(motorSpeed_m2);
    /////////////////////////////////////////////

    /*
    /////////////////////////////////////////////
    //Control 2 motors via 2 pots
    //Motor 1
    potValue_m1 = analogRead(POD_PIN_M1);
    potValue_m1 = map(potValue_m1, 0, 1023, 0, 180);   
    esc_m1.write(potValue_m1);
    Serial.print("Motor 1 speed:\t");
    Serial.println(potValue_m1);
            
    //Motor 2
    potValue_m2 = analogRead(POD_PIN_M2);
    potValue_m2 = map(potValue_m2, 0, 1023, 0, 180);
    esc_m2.write(potValue_m2);   
    Serial.print("Motor 2 speed:\t");
    Serial.println(potValue_m2);
    /////////////////////////////////////////////
    */

    /*
    /////////////////////////////////////////////
    //Control 1 motor via keyboard input
    if(Serial.available() > 0) 
    {
        //only works with 1 char, will need a char array for more
        char incomingData = Serial.read(); // can be -1 if read error
        int incomingDataInt = atoi(&incomingData);
        potValue = map(incomingDataInt, 0, 9, 0, 180);  
    }

        Serial.print("pot\t");
        Serial.println(potValue);      
        
        esc.write(potValue);
    /////////////////////////////////////////////
    */

    delay(500);
}

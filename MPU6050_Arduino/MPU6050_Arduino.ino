// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// Setup Timer1
uint32_t timerTicks = 0;
bool setupTimerMsec() {
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1khz increments (1ms) timer
    OCR1A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS01 and CS00 bits for 64 prescaler
    TCCR1B |= (1 << CS01) | (1 << CS00);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

}
bool setupTimerSec() {
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments (1sec) timer
    OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10); 
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
}
// With the settings above, this IRS will trigger each 1ms.
ISR(TIMER1_COMPA_vect) {
    TCNT1  = 0;      // First, set the timer back to 0 so it resets for next interrupt
    timerTicks += 1; // increment timer tick

    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
}

// Derived timer with user-defined interval
uint32_t derivedTicks = 0;
bool timeToUpdate(uint32_t interval_ms)
{
    uint32_t now = timerTicks;
    if ((now - derivedTicks) >= interval_ms)
    {
        derivedTicks = now;
        return true;
    }
    else
    {
        return false;
    }
}

#define OUTPUT_RAW_6AXIS    (1)
#define OUTPUT_DMP_6AXIS    (2)
#define OUTPUT_SELECTION    OUTPUT_DMP_6AXIS

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setWireTimeout(3000, true); //timeout value in μs
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    setupTimerMsec();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(7);
        mpu.CalibrateGyro(7);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    Serial.println("Settings:");
    Serial.println(mpu.getFullScaleAccelRange());
    Serial.println(mpu.getFullScaleGyroRange());

    // Signal client ready to receive data
    Serial.println("MPU6050 ready.");
}

//// To use Low-Pass Filter
float lowPassFilter(float beta, float last, float raw)
{
    return last - (beta * (last - raw));
}

//// To use Kalman Filter
// #include <TrivialKalmanFilter.h>
// #define DT_COVARIANCE_RK 0.3 // Estimation of the noise covariances (process)
// #define DT_COVARIANCE_QK 0.05 // Estimation of the noise covariances (observation)
// TrivialKalmanFilter<float> filter_x(DT_COVARIANCE_RK, DT_COVARIANCE_QK);
// TrivialKalmanFilter<float> filter_y(DT_COVARIANCE_RK, DT_COVARIANCE_QK);
// TrivialKalmanFilter<float> filter_z(DT_COVARIANCE_RK, DT_COVARIANCE_QK);

uint32_t lastTicks = 0;
uint32_t skip = 10;
float accl[3] = {0,0,0};
float velo[3] = {0,0,0};
float disp[3] = {0,0,0};

// For velocity filter
float velo_diff[5] = { 0 };
uint8_t count = 0;

void loop() {

    // blink LED to indicate activity
    // if (timeToUpdate(500)) {
    //     blinkState = !blinkState;
    //     digitalWrite(LED_PIN, blinkState);
    // }
    
    float velo_temp[3] = {};

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // if (!timeToUpdate(20)) return;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        // Skip the first 10 read
        if (skip > 0)
        {
            skip--;
            lastTicks = timerTicks;
            return;
        }

        #if (OUTPUT_SELECTION == OUTPUT_RAW_6AXIS)

                // read raw accel/gyro measurements from device
                mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                // display comma-separated accel/gyro x/y/z values
                Serial.print(ax*  2.0/32767); Serial.print(",");
                Serial.print(ay*  2.0/32767); Serial.print(",");
                Serial.print(az*  2.0/32767); Serial.print(",");
                Serial.print(gx*500.0/32767); Serial.print(",");
                Serial.print(gy*500.0/32767); Serial.print(",");
                Serial.print(gz*500.0/32767); Serial.print(",");
                Serial.println("");

        #elif (OUTPUT_SELECTION == OUTPUT_DMP_6AXIS)

                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

                // Compute Acceleration
                const float GRAVITY = 9.781;
                uint32_t now = timerTicks;
                float delta_t = (now - lastTicks) / 1000.0;
                Serial.print(now - lastTicks); Serial.print(",");

                lastTicks = now;
                float accl_x = float(aaWorld.x)/8192 * GRAVITY;
                float accl_y = float(aaWorld.y)/8192 * GRAVITY;
                float accl_z = float(aaWorld.z)/8192 * GRAVITY;

                // No filter
                accl[0] = accl_x; // lowPassFilter(0.025, accl[0], accl_x);
                accl[1] = accl_y; // lowPassFilter(0.025, accl[1], accl_y);
                accl[2] = accl_z; // lowPassFilter(0.025, accl[2], accl_z);

                // LP Filter
                // accl[0] = lowPassFilter(0.025, accl[0], accl_x);
                // accl[1] = lowPassFilter(0.025, accl[1], accl_y);
                // accl[2] = lowPassFilter(0.025, accl[2], accl_z);

                // Kalman Filter
                // accl[0] = filter_x.update(accl[0]);
                // accl[1] = filter_x.update(accl[1]);
                // accl[2] = filter_x.update(accl[2]);
        
                // Minimum aceleration
                const float ACCL_LIMIT = 0.02;
                if (abs(accl[0]) < ACCL_LIMIT) { accl[0] = 0; } // velo[0] = 0; }
                if (abs(accl[1]) < ACCL_LIMIT) { accl[1] = 0; } // velo[1] = 0; }
                if (abs(accl[2]) < ACCL_LIMIT) { accl[2] = 0; } // velo[2] = 0; }

                Serial.print(accl[0]); Serial.print(",");
                Serial.print(accl[1]); Serial.print(",");
                Serial.print(accl[2]); Serial.print(",");

                float half_delta_t2 = 0.5*delta_t*delta_t;
                // Compute Displacement
                // d = u*t + 0.5*a*t*t
                disp[0] += (velo[0]*delta_t)+(accl[0]*half_delta_t2);
                disp[1] += (velo[1]*delta_t)+(accl[1]*half_delta_t2);
                disp[2] += (velo[2]*delta_t)+(accl[2]*half_delta_t2);

                // Save last velocity for later use
                velo_temp[0] =  velo[0];
                velo_temp[1] =  velo[1];
                velo_temp[2] =  velo[2];

                // Compute Velocity
                //   v = v_0 + a*t
                velo[0] += (accl[0]*delta_t);
                velo[1] += (accl[1]*delta_t);
                velo[2] += (accl[2]*delta_t);

                // Filter velocity
                float m = 0;
                if (count != 5)
                {
                    velo_diff[count] = abs(
                        sqrt(pow(velo[0],2)+pow(velo[1],2)+pow(velo[2],2)) -
                        sqrt(pow(velo_temp[0],2)+pow(velo_temp[1],2)+pow(velo_temp[2],2)));
                    count++;
                }
                else
                {
                    count = 0;
                    m = (velo_diff[0] + velo_diff[1] + velo_diff[2] +velo_diff[3] +velo_diff[4])/5;
                    if (m <= 0.002)
                    {
                        velo[0] = 0;
                        velo[1] = 0;
                        velo[2] = 0;
                    }
                }

                Serial.print(velo[0]); Serial.print(",");
                Serial.print(velo[1]); Serial.print(",");
                Serial.print(velo[2]); Serial.print(",");

                Serial.print(disp[0]); Serial.print(",");
                Serial.print(disp[1]); Serial.print(",");
                Serial.print(disp[2]); Serial.print(",");

                // Serial.print(float(ypr[2])*180.0/M_PI); Serial.print(",");
                // Serial.print(float(ypr[1])*180.0/M_PI); Serial.print(",");
                // Serial.print(float(ypr[0])*180.0/M_PI); Serial.print(",");

                // mpu.dmpGetEuler(euler, &q);
                // Serial.print(euler[0]*180.0/M_PI); Serial.print(",");
                // Serial.print(euler[1]*180.0/M_PI); Serial.print(",");
                // Serial.print(euler[2]*180.0/M_PI); Serial.print(",");

                Serial.println("");

        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

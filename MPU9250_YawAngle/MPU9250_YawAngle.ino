// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

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

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int16_t magCount[3], accelCount[3];

float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
bool newMagData = false;
float mRes = 10.*4912./32760.0;
float aRes = 2.0/32768.0;
float heading = 0.0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    //accelGyroMag.initialize();
    //Initialize using KrisWinner method instead of this one
    accelGyroMag.initMPU9250();
    accelGyroMag.initAK8963(magCalibration);

    //Calibrate magnetometer
//    accelGyroMag.magcalMPU9250(magBias, magScale, magCalibration);
//    delay(5000);// add delay to see results before serial spew of data
    
    //mag biases: 21.71, 33.16, -21.51
    //mag scale: 1.11, 0.97, 0.93 
    
    //mag biases: 27.63, 39.01, -28.68
    //mag scale: 1.01, 0.99, 0.99

    //mag biases: 21.71, 29.26, -26.88
    //mag scale: 1.08, 0.99, 0.94

    magBias[0] = 23.68;
    magBias[1] = 33.81;
    magBias[2] = -25.69;
    magScale[0] = 1.06;
    magScale[1] = 0.98;
    magScale[2] = 0.95;

    Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
    Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
    
    
    // verify connection
    /*
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
    */
}

void loop() {
    // read raw accel/gyro/mag measurements from device
//    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
    uint8_t b = (accelGyroMag.readByte(0x68, 0x3A) & 0x01);
    if (b) {
        accelGyroMag.getAcceleration(&ax, &ay, &az);
        accelGyroMag.readMagData(magCount);

        ax = (float)ax;
        ay = (float)ay;
        az = (float)az;
        
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];
    }
    else {
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2]; 
    }

    /*Serial.print("ax = "); Serial.print(ax); 
    Serial.print(" ay = "); Serial.print(ay); 
    Serial.print(" az = "); Serial.print(az); Serial.println(" raw");
    Serial.print("mx = "); Serial.print(mx); 
    Serial.print(" my = "); Serial.print(my); 
    Serial.print(" mz = "); Serial.print(mz); Serial.println(" mG");*/

    heading = atan2(my, mx);

    if (heading < 0.0) {
        heading += (2.0 * PI);
    }
    if (heading > (2.0 * PI)) {
        heading -= (2.0 * PI);
    }
    heading *= (180.0 / PI);
    Serial.print("Heading: "); Serial.println(heading);
    

    //Delay not necessary, as in readMagData it is checked if the data is available
//    delay(50);
}

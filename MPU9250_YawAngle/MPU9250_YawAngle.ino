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

float ax_base, ay_base, az_base;
float gx_base, gy_base, gz_base;
float gx_angle, gy_angle, gz_angle;

float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
bool newMagData = false;
float mRes = 10.*4912./32760.0;
float aRes = 2.0/32768.0;
float heading = 0.0;

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};
float a12, a22, a31, a32, a33;
float pitch, yaw, roll;
float aRoll, aPitch;
float RADIANS_TO_DEGREES = 180/3.14159;
float dt, last_time;
float alpha = 0.96;
float last_x_angle, last_y_angle, last_z_angle;
float magx, magy;

float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

uint32_t Now = 0, lastUpdate = 0;
float deltat = 0.0f;

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

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

    //Calibrate accelerometer and gyroscope
    /*
    accelGyroCalMPU9250();
    */
    
    //Calibrate magnetometer
    /*
    accelGyroMag.magcalMPU9250(magBias, magScale, magCalibration);
    delay(5000);// add delay to see results before serial spew of data
    */
    
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

    ax_base = -184.55;
    ay_base = 207.77;
    //az_base = 15749.66; //az must not be 0, so offset value is not applied
    gx_base = -279.67;
    gy_base = 139.34;
    gz_base = -65.23;
    
    // verify connection
    /*
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
    */

    last_time = millis();
    last_x_angle = 0;
    last_y_angle = 0;
    last_z_angle = 0;
}

void loop() {
    // read raw accel/gyro/mag measurements from device
//    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
    uint8_t b = (accelGyroMag.readByte(0x68, 0x3A) & 0x01);
    float ax2, ay2, az2, gx2, gy2, gz2;
    if (b) {
        accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        accelGyroMag.readMagData(magCount);

        ax2 = (float)ax/16384;
        ay2 = (float)ay/16384;
        az2 = (float)az/16384;

        gx2 = (float)(gx-gx_base)/131;
        gy2 = (float)(gy-gy_base)/131;
        gz2 = (float)(gz-gz_base)/131;

        float res = 4800.0/32767.0;
        float res2 = 10.*4800/32767.0;

        /*Serial.print("mx = "); Serial.print(magCount[0]); 
        Serial.print(" my = "); Serial.print(magCount[1]); 
        Serial.print(" mz = "); Serial.print(magCount[2]); Serial.println(" raw");
        Serial.print("mx = "); Serial.print(magCount[0]*res); 
        Serial.print(" my = "); Serial.print(magCount[1]*res); 
        Serial.print(" mz = "); Serial.print(magCount[2]*res); Serial.println(" uT");
        Serial.print("mx = "); Serial.print(magCount[0]*res2); 
        Serial.print(" my = "); Serial.print(magCount[1]*res2); 
        Serial.print(" mz = "); Serial.print(magCount[2]*res2); Serial.println(" mG");*/
        Serial.print(magCount[0]*res2); Serial.print(",");
        Serial.print(magCount[1]*res2); Serial.print(",");
        Serial.println(magCount[2]*res2);
        
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];

        Now = micros();
        deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        //MadgwickAHRSupdate(gx2, gy2, gz2, ax2, ay2, az2, mx, my, mz);
        MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
        
        /*
        complementaryFilter(ax2, ay2, az2, gx2, gy2, gz2, mx, my, mz);
        */
    
        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / PI;

        //Serial.println("q[0]\tq[1]\tq[2]\tq[3]");
        //Serial.print(q[0]); Serial.print("\t"); Serial.print(q[1]);
        //Serial.print("\t"); Serial.print(q[2]); Serial.print("\t"); Serial.println(q[3]);
        //Serial.println("Simple\tMadgwick");
        //Serial.print(simpleHeading()); Serial.print("\t"); Serial.println(yaw);
    }

    /*
    Serial.print("ax = "); Serial.print(ax); 
    Serial.print(" ay = "); Serial.print(ay); 
    Serial.print(" az = "); Serial.print(az); Serial.println(" raw");
    Serial.print("ax2 = "); Serial.print(ax2); 
    Serial.print(" ay2 = "); Serial.print(ay2); 
    Serial.print(" az2 = "); Serial.print(az2); Serial.println(" g");
    Serial.print("gx = "); Serial.print(gx); 
    Serial.print(" gy = "); Serial.print(gy); 
    Serial.print(" gz = "); Serial.print(gz); Serial.println(" raw");
    Serial.print("gx2 = "); Serial.print(gx2); 
    Serial.print(" gy2 = "); Serial.print(gy2); 
    Serial.print(" gz2 = "); Serial.print(gz2); Serial.println(" grad/s");
    Serial.print("mx = "); Serial.print(mx); 
    Serial.print(" my = "); Serial.print(my); 
    Serial.print(" mz = "); Serial.print(mz); Serial.println(" mG");
    */


    //Heading calculation, works but very imprecise.
    /*
    heading = simpleHeading();
    */

    //Delay not necessary, as in readMagData it is checked if the data is available
    delay(50);
}

float simpleHeading() {
    heading = atan2(my, mx);
    if (heading < 0.0) {
        heading += (2.0 * PI);
    }
    if (heading > (2.0 * PI)) {
        heading -= (2.0 * PI);
    }
    heading *= (180.0 / PI);
    return heading;
}

void complementaryFilter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    aRoll = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RADIANS_TO_DEGREES;
    aPitch = atan(-1*ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES;

    dt = (millis() - last_time)/1000.0;
    gx_angle = gx*dt + last_x_angle;
    gy_angle = gy*dt + last_y_angle;
    gz_angle = gz*dt + last_z_angle;

    roll = alpha*gx_angle + (1.0 - alpha)*aRoll;
    pitch = alpha*gy_angle + (1.0 - alpha)*aPitch;

    //Serial.print(roll); Serial.print("  "); Serial.println(pitch);

    magx = mz*sin(roll) - my*cos(roll);
    magy = mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll);

    yaw = atan(magx/magy)*RADIANS_TO_DEGREES;
    //Serial.println(yaw);
    
    last_time = millis();
    last_x_angle = roll;
    last_y_angle = pitch;
    last_z_angle = gz_angle;
}

void accelGyroCalMPU9250() {
    int                   num_readings = 100;
    float                 x_accel = 0;
    float                 y_accel = 0;
    float                 z_accel = 0;
    float                 x_gyro = 0;
    float                 y_gyro = 0;
    float                 z_gyro = 0;
    //uint16_t ax, ay, az, gx, gy, gz;
    float accelGyroBase[6];
  
    //Serial.println("Starting Calibration");
  
    // Discard the first set of values read from the IMU
    //read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Read and average the raw values from the IMU
    for (int i = 0; i < num_readings; i++) {
      accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.println("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.print(gz); Serial.print("\t");
      x_accel += (float)ax;
      y_accel += (float)ay;
      z_accel += (float)az;
      x_gyro += (float)gx;
      y_gyro += (float)gy;
      z_gyro += (float)gz;
      delay(100);
    }
    x_accel /= num_readings;
    y_accel /= num_readings;
    z_accel /= num_readings;
    x_gyro /= num_readings;
    y_gyro /= num_readings;
    z_gyro /= num_readings;
  
    // Store the raw calibration values globally
    ax_base = x_accel;
    ay_base = y_accel;
    az_base = z_accel;
    gx_base = x_gyro;
    gy_base = y_gyro;
    gz_base = z_gyro;
    
    Serial.println("Base values");
    Serial.print(ax_base); Serial.print("\t");
    Serial.print(ay_base); Serial.print("\t");
    Serial.print(az_base); Serial.println("\t");
    Serial.print(gx_base); Serial.print("\t");
    Serial.print(gy_base); Serial.print("\t");
    Serial.print(gz_base); Serial.print("\t");
  
    //Serial.println("Finishing Calibration");
}

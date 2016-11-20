// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode

#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value


uint8_t Gscale = 0;
uint8_t Ascale = 0;
uint8_t Mscale = 1; // 16-bit magnetometer resolution
uint8_t Mmode = 0x06;

int16_t accelCount[3], gyroCount[3], magCount[3];

float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
float mRes = 10.*4800.0/32767.0; // Proper scale to return milliGauss
float aRes = 2.0/32767.0;
float gRes = 250.0/32767.0;

float ax, ay, az, gx, gy, gz, mx, my, mz;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(38400);

    Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

    if (c == 0x71) {
        Serial.println("MPU9250 is online...");

        initMPU9250();
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        //magcalMPU9250(magBias, magScale);
        Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
        Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
        delay(2000); // add delay to see results before serial spew of data
    }
}

void loop() {
    // If intPin goes high, all data registers have new data
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        /*readAccelData(accelCount);  // Read the x/y/z adc values
        
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes; // - accelBias[1];   
        az = (float)accelCount[2]*aRes; // - accelBias[2];
        Serial.print("ax = "); Serial.print(ax); 
        Serial.print(" ay = "); Serial.print(ay); 
        Serial.print(" az = "); Serial.print(az); Serial.println(" g");
       
        readGyroData(gyroCount);  // Read the x/y/z adc values
     
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;  
        gz = (float)gyroCount[2]*gRes;
        Serial.print("gx = "); Serial.print(gx); 
        Serial.print(" gy = "); Serial.print(gy); 
        Serial.print(" gz = "); Serial.print(gz); Serial.println(" deg/s");
        */
        readMagData(magCount);  // Read the x/y/z adc values
        
//        magBias[0] = -335.09;
//        magBias[1] = 681.61;
//        magBias[2] = -513.17;
        
//        magBias[0] = -340.33;
//        magBias[1] = 655.26;
//        magBias[2] = -509.79;

//        magBias[0] = -376.98;
//        magBias[1] = 648.23;
//        magBias[2] = -524.99;
//        magScale[0] = 1.10;
//        magScale[1] = 0.93;
//        magScale[2] = 0.99;

//        magBias[0] = -366.51;
//        magBias[1] = 628.91;
//        magBias[2] = -535.11;
//        magScale[0] = 1.01;
//        magScale[1] = 0.96;
//        magScale[2] = 1.04;

//        magBias[0] = -396.18;
//        magBias[1] = 641.20;
//        magBias[2] = -499.66;
//        magScale[0] = 1.14;
//        magScale[1] = 0.93;
//        magScale[2] = 0.96;

        magBias[0] = -363.018;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        magBias[1] = 651.042;  // User environmental x-axis correction in milliGauss
        magBias[2] = -516.544;  // User environmental x-axis correction in milliGauss
        magScale[0] = 1.0625;
        magScale[1] = 0.945;
        magScale[2] = 0.9825;

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2]; 

        Serial.print(mx); Serial.print(","); 
        Serial.print(my); Serial.print(","); 
        Serial.println(mz);
        
    }
}


void initMPU9250() {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Wait for all registers to reset 
    
    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 
    
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  
    
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                  // determined inset in CONFIG above
    
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
    
    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer 
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
    
    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
    
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
    
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}

void initAK8963(float * destination) {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);
}

void magcalMPU9250(float * dest1, float * dest2) 
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);
  
    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < sample_count; ii++) {
        readMagData(mag_temp);  // Read the mag data   
        for (int jj = 0; jj < 3; jj++) {
          if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
          if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
    Serial.println("Mag Calibration done!");
}

void readAccelData(int16_t * destination) {
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readGyroData(int16_t * destination) {
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination) {
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
          destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
          destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
          destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
        }
    }
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;                            // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
//  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
//    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
}

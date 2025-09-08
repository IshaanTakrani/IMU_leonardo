#include <Arduino.h>
#include <Wire.h>
#include <Mouse.h>
#include "mpu6500.h"
// #include "CircularBuffer.hpp"
#include <CircularBuffer.hpp>
#include <ishaanfilter.h>


// Madgwick filter and IMU instance
float ax, ay, az, gx, gy, gz;

bfs::Mpu6500 imu;

CircularBuffer<float, 64> xAccelBuffer;
float xCalibrationMean;
float xCurrVel;
float xBufferMean;

IshaanFilter ishaanfilter(0,0,0);


CircularBuffer<float, 64> yAccelBuffer;
float yCalibrationMean;
float yCurrVel;
float yBufferMean;

float xCal;
float yCal;
float zCal;


int freeMemory() {
  extern int __bss_end;
  extern int *__brkval;
  int free_memory;

  if((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  } else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }

  Serial.print("free memory: ");
  Serial.print(free_memory);
  Serial.print("\t");
  return free_memory;
}


void calibrate(){

    xCal = 0.0;
    yCal = 0.0;
    zCal = 0.0;

    for(int i = 0;i<200;i++){
        if (imu.Read()) {
            float ax = imu.accel_x_mps2();
            float ay = imu.accel_y_mps2();
            float az = imu.accel_z_mps2();
            xCal += ax;
            yCal += ay;
            zCal += az;
            delay(5);
        }
    }
    
    xCal /= 200;
    yCal /= 200;
    zCal /= 200;

}


void setup() {

  Serial.begin(9600);
  while (!Serial) {}  // Wait for Serial to be ready

  Wire.begin();
  Wire.setClock(400000);  // Fast I2C

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  
  if (!imu.Begin()) {
    Serial.println("Error initializing IMU");
    while (1) {}
  }

  if (!imu.ConfigSrd(1)) { 
    Serial.println("Error setting SRD");
    while (1) {}

  }
  
  // filter.begin(100);  // init madgwick filter sample rate

  calibrate();
  ishaanfilter.updateCalValues(xCal, yCal, zCal);

  Serial.println("Calibration mean: ");
  Serial.println(xCalibrationMean);

}

void loop() {
  if (imu.Read()) {
    
    ax = imu.accel_x_mps2();
    ay = imu.accel_y_mps2();
    az = imu.accel_z_mps2();
    gx = imu.gyro_x_radps();
    gy = imu.gyro_y_radps();
    gz = imu.gyro_z_radps();


    
    ishaanfilter.updateFilter(ax, ay, az);


    // Serial.print(ishaanfilter.xV);
    Serial.print(ax);
    Serial.print("\t");
    // Serial.println(ishaanfilter.yV);
    Serial.print(ay);
    Serial.print("\t");
    
    Mouse.move(ishaanfilter.xV, ishaanfilter.yV, 0);
    
    
    for(int i = 0;i<ishaanfilter.xBuffer.size();i++){
      Serial.print(ishaanfilter.xBuffer[i]);
      Serial.print(", ");
    }

    // Serial.print("Accel [m/s^2]:\t");
    // Serial.print(ax, 2); Serial.print("\t");
    // Serial.print(ay, 2); Serial.print("\t");
    // Serial.print(az, 2); Serial.print("\t");

    // Serial.print("Gyro [rad/s]:\t");
    // Serial.print(gx, 2); Serial.print("\t"); 
    // Serial.print(gy, 2); Serial.print("\t");
    // Serial.print(gz, 2); Serial.println("\t");
          
 
    Serial.println();
    delay(10);

  }  
}


// #include <SoftwareSerial.h>
// SoftwareSerial BTSerial(10, 11);
// void setup() {
//  Serial.begin(115200);
//  Serial.println("Enter AT Commands: ");
//  BTSerial.begin(38400);
// }
// void loop() {
// if (BTSerial.available()){
// Serial.write(BTSerial.read());
// }
// if (Serial.available()){
// BTSerial.write(Serial.read());
// }
// }
#include <RH_RF24.h>
#include "Wire.h"
#include "SPI.h"
#include "Arduino.h"
#include "avr/interrupt.h"
const int MPU_ADDR = 0x68;

int16_t accelerometer_x, accelerometer_y, accelerometer_z; //variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; //variables for gyro raw data
int16_t temperature; //variables for temperature data
int16_t accelX_avg[9], accelY_avg[9], accelZ_avg[9];
char tmp_str[7]; //temporary variable used in convert function

char* convertToString(int16_t data) 
  {
       // converts int16 to string. 
      char convertStr[7] = {0};   //8bit str data
      sprintf(convertStr, "%6d", data);
      return convertStr;
  }
void dataReading(){
}
void mpuInit(){
  // power management
  Wire.beginTransmission(MPU_ADDR);           
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0 meaning waking MPU up
  Wire.endTransmission(true);                

  //config external synchronization and low pass filter
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1A);                           //Access the CONFIG configuration register
  Wire.write(0b00000000);                     //highest bandwidth
  Wire.endTransmission();
  
  // config gyroscope
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                           //Access the GYRO_CONFIG register
  Wire.write(0b00000001);                     //AFS_SEL = 1, 500 deg/sec
  Wire.endTransmission(true);

  // configure accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                           //Access the ACCEL_CONFIG register
  Wire.write(0b00000001);                     //AFS_SEL = 1, 4g
  Wire.endTransmission(true);  
}
void mpuRead(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                       //Starting from ACCEL_XOUT_H
  Wire.endTransmission(false);            //Keep connection active throughout the transmission
  Wire.requestFrom(MPU_ADDR, 7*2, true);  //Request a total of 7reg*2byte=14 consecutive registers, starting from ACCEL_XOUT_H
  //I use OR bitwise to concat them into 16bits data
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read();     // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read();          // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read();          // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read();          // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
}
void serialPrinti(){
  // print out data on Serial monitor
  Serial.print("--------------------------------");
  Serial.print(" | aX = "); Serial.print(convertToString(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convertToString(accelerometer_y));
  Serial.print(" | aZ = "); Serial.println(convertToString(accelerometer_z));
  Serial.print(" | tmp = "); Serial.println(temperature/340.00+36.53);        //idk why but Invensense datasheet asked to
  Serial.print(" | gX = "); Serial.print(convertToString(gyro_x));
  Serial.print(" | gY = "); Serial.print(convertToString(gyro_y));
  Serial.print(" | gZ = "); Serial.println(convertToString(gyro_z));
  Serial.println();
  // delay
  delay(1000);
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpuInit();
}
void loop() {
  
}

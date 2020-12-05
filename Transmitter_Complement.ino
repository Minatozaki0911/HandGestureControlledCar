#include <RH_NRF24.h>
#include <Wire.h>
#include <Kalman.h>
#include <Math.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccelX, AccelY, AccelZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
uint8_t c = 0;
double gyroFS = 0;
double accelFS = 0;
uint8_t sample = 200;

 RH_NRF24 nrf24;

void timerInit(){
  cli();
  TCCR0A = 0x00;
  TCCR0B = 0x00;
  TIMSK0 = 0x00;
  TCCR0B = 0x03 ; // Prescaler 64
  TCNT0 =  230; 
  TIMSK0 = 0x01;
  sei();
}
 char *command(float X, float Y){
    if (X>20 && Y<20 && Y>-20)
    return "Forward";
    if (X<-20 && Y<20 && Y>-20)
    return "Backward";
    if (Y>20 && X<20 && X>-20)
    return "RotateRight";
    if (Y<-20 && X<20 && X>-20)
    return "RotateLeft";
    if(X>20 && Y>20)
    return "ForLeft";
    if(X>20 && Y<-20)
    return "ForRight";
    if(X<-20 && Y>20)
    return "BackLeft";
    if(X<-20 && Y<-20)
    return "BackRight";
    if(X<20 && X>-20 && Y <20 && Y > -20)
    return "Idle";
  
void mpuInit(){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       
  Wire.write(0x6B);                  //POWER Management 1
  Wire.write(0x00);                  //Wake up MPU and select internal 8MHz as CLK source
  Wire.endTransmission(true);       
  //Enable Digital Low pass Filter at 188HZ  
  Wire.beginTransmission(MPU);       
  Wire.write(0x1A);                  //CONFIG
  Wire.write(0b00000110);                  //DPLF_CFG
  Wire.endTransmission(true);  
  // Configure Gyro Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   //GYRO_CONFIG
  Wire.write(0x10);                   //FS_SEL = 2 (+- 1000deg/sec)
  Wire.endTransmission(true);
  gyroFS = 1000.0;
  // Configure Accelerometer Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //ACCEL_CONFIG
  Wire.write(0x10);                  //AFS_SEL = 2 (+/- 8g full scale range)
  Wire.endTransmission(true);
  accelFS = pow(2,3);
}
  /*
   * All ACCEL_xOUT_x and GYRO_xOUT_x stored 16-bits of 2's complement value.
   * Which mean max data value we can have is 15bits ~ 32768LSB
   * Then we wil map this data to the FULL_SCALE_RANGE as in ACCEL and GYRO configuration
   */
void readAccel(){
    long double LSBperG = pow(2, 15)/accelFS;
    //Read 6 consecutive registers, starting from 0x3B (ACCEL_XOUT_H)
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);                 //ACCEL_XOUT_H
    Wire.endTransmission(false);      //Keep connection alive
    Wire.requestFrom(MPU, 6, true);
    //Slapping the g out of those nonsense data
    //concatenate 8bit register ACCEL_XOUT_H and ACCEL_XOUT_L to form a 16-bits register
    //2^14 bits for each g increment --> divide it to get the actual accel based on Earth's gravitational pull
    AccelX = (Wire.read() << 8 | Wire.read()) / (double)LSBperG ;
    AccelY = (Wire.read() << 8 | Wire.read()) / (double)LSBperG ;
    AccelZ = (Wire.read() << 8 | Wire.read()) / (double)LSBperG ;
}
void readGyro(){
    long double LSBperDegree = pow(2,15)/gyroFS;
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    //In the same manner as above
    GyroX = (Wire.read() << 8 | Wire.read())/LSBperDegree;
    GyroY = (Wire.read() << 8 | Wire.read())/LSBperDegree;
    GyroZ = (Wire.read() << 8 | Wire.read())/LSBperDegree;
}

void setup() {
  Serial.begin(9600);
  mpuInit();
  timerInit();
    if (!nrf24.init())
    Serial.println("init failed");
  delay(20);
  calculate_IMU_error();
  delay(20);
}

void loop() {
  while((TIFR0&0x01)==0);         //Wait till OVF0 is set
  TIFR0 = 0;        //clear OVF0 flag
  TCNT0 = 230;
  // === Read acceleromter data === //
  readAccel();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI) - AccErrorY;
  
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = micros();            // current time when called this function
  elapsedTime = (currentTime - previousTime) / 1000000.0; //Time passed since last read
  // === Read gyroscope data === //
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX += GyroErrorX;
  GyroY += GyroErrorY;
  GyroZ += GyroErrorZ;
  
  // Currently the raw values are in degrees per seconds, but only the degree part is value to us
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  /*
   * Complementary Filter
   * Gyro tends to drift alot over time
   * Accel is extremely noisy at short duration of time - which already filtered high frequency shot noise with DLPF
   * By combining both Gyro - Accel we can correct the data
   */
  roll = 0.90 * gyroAngleX + 0.1 * accAngleX;
  pitch = 0.90 * gyroAngleY + 0.1 * accAngleY;
  //GyroZ will drifted overtime but we cannot fuse it with AccelZ
  yaw        += GyroZ * elapsedTime; 
  Serial.print("[ ");
  Serial.print(roll);
  Serial.print(" / ");
  Serial.print(pitch);
  Serial.print(" / ");
  Serial.print(yaw);
  Serial.println(" ]");
  nrf24.send(command(roll, pitch), sizeof(command(roll,pitch)))  ;//I wont use Yaw because its trash
  nrf24.waitPacketSent();
}


void calculate_IMU_error() {
    while (c < sample) {
    //Read Accel 200 times and get average value
    readAccel();
    AccErrorX += atan(AccelY / sqrt(pow((AccelX), 2) + pow((AccelZ), 2))) * 180 / PI;
    AccErrorY += atan(AccelX / sqrt(pow((AccelY), 2) + pow((AccelZ), 2))) * 180 / PI;
    c++;
  }
  AccErrorX /= sample;
  AccErrorY /= sample;
  c = 0;
  // Read gyro values 200 times and get the average 
  while (c < sample) {
    readGyro();
    // Sum all readings
    GyroErrorX += (GyroX);
    GyroErrorY += (GyroY);
    GyroErrorZ += (GyroZ);
    c++;
  }
  GyroErrorX /= sample;
  GyroErrorY /= sample;
  GyroErrorZ /= sample;  

}

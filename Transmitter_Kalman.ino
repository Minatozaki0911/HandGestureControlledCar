#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include <RH_NRF24.h>

 float fRad2Deg = 57.295779513f; // multiply the radians to angles
 const int MPU = 0x68; //MPU-6050 I2C address
 const int nValCnt = 7; //Number of registers read at a time
 
 const int nCalibTimes = 1000; //Number of readings during calibration
 int calibData[nValCnt]; //calibration data
 
 unsigned long nLastTime = 0; //Time of the last reading
 float fLastRoll = 0.0f; //Roll angle obtained from the last filter
 float fLastPitch = 0.0f; //Pitch angle obtained from the last filtering
 RH_NRF24 nrf24;
 Kalman kalmanRoll; //Roll angle filter
 Kalman kalmanPitch; //Pitch angle filter

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
  }
  
void setup() {
     Serial.begin(9600); //Initialize the serial port, specify the baud rate
     Wire.begin(); //Initialize the Wire Library
     WriteMPUReg(0x6B, 0); //Start the MPU6050 device
 
     Calibration(); //Perform calibration
     nLastTime = micros(); //record the current time
     Serial.begin(9600);
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
    Serial.println("Sending to nrf24_server");
  // Send a message to nrf24_server
  uint8_t data[] = "Connection Established";
  nrf24.send(data, sizeof(data));
  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (nrf24.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (nrf24.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is nrf24_server running?");
  }
}
void loop() {
  int readouts[nValCnt];
     ReadAccGyr (readouts); // read the measured value
  
  float realVals[7];
     Rectify(readouts, realVals); //correct based on the offset of the calibration
 
     // Calculate the modulus length of the acceleration vector, all in g
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //calculate the Roll angle
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
     float fPitch = GetPitch(realVals, fNorm); //calculate Pitch angle
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
 
     // Calculate the time interval dt of the two measurements, in seconds
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
     // Kalman filtering the Roll angle and Pitch angle
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
     // Calculate the angular velocity according to the filtered value
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
 
   // Update the Roll angle and Pitch angle
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
     // Update the time of this test
  nLastTime = nCurTime;


  //Command sending
  nrf24.send(command(fNewRoll, fNewPitch), sizeof(command(fNewRoll, fNewPitch)));
  nrf24.waitPacketSent();
  
  Serial.print("LenXuong:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tTraiPhai:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")\n");
  delay(100);
}
 
 // Write a byte of data to the MPU6050
 // specify the register address and a byte value
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}
 
 // Read a byte of data from the MPU6050
 // Specify the register address, return the read value
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}
 
 // Read the three components of the accelerometer, temperature and three angular velocity meter from the MPU6050
 // Save in the specified array
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}
 
 // Statistics on a large number of readings, calibration average offset
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
     // First sum
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
     //Re-average
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
     calibData[2] += 16384; //Set the chip Z axis vertically downwards to set the static working point.
}
 
 // Calculated the Roll angle. The algorithm is described in the documentation.
float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}
 
 // Calculated Pitch angle. The algorithm is described in the documentation.
float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}
 
 // Correct the readings, eliminate the offset, and convert to physical quantities.
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}

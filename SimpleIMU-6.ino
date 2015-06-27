/*  Very Simple Complimentary Filter for the Invesense MPU-6050
 *  Phillip Schmidt
 *  v1.0, June 2015
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Math3D.h>


MPU6050 MPU(400, 0, 3, 3); // update rate, filtering, gyro, accel

unsigned long nextSampleIMU;
unsigned long nextDriftCorrection;
unsigned long timeDisplay = 0;

unsigned long testTimer;

int imuCount = 0;
float AccelSampleCount = 0;

Quat RotationQuat;
M3x3 Matrix;

Vec3 driftCorrectionVec, rotationRateVec;
Vec3 AccelVec, AccelVecWorld, GyroVec;

const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the Earth frame


void setup() {
  
  Serial.begin(115200);  // start serial for output
  Serial.println("*** Run On Start ***");
  
  Wire.begin();        // join i2c bus
  Wire.setClock(400000UL);// set speed to 400k
  	
  	
  MPU.initialize();
  Serial.println(MPU.samplePeriod);
  MPU.accelZero();
  MPU.gyroZero();	
  
  nextSampleIMU = micros() + MPU.samplePeriod;
  nextDriftCorrection = micros() + 20000UL;

}

void loop() // Start Main Loop
{
  
    
  if(micros() > nextSampleIMU)  // check if it is time for next sensor sample
  {
    
     imuCount++;

     nextSampleIMU += MPU.samplePeriod;  // set next sample time
     
     testTimer = micros(); // *** TIMER START *** 
     
     MPU.retrieve();	   // get data from the sensor
     MPU.convertToFloat(); // convert integer data to float


     GyroVec = Vector(MPU.gX, MPU.gY, MPU.gZ);    // move gyro data to vector structure

     AccelVec = Vector(MPU.aX, MPU.aY, MPU.aZ);   // move accel data to vector
    
     AccelVecWorld = Rotate(RotationQuat, AccelVec); // rotate accel from body frame to world frame
     
     driftCorrectionVec = CrossProd(AccelVecWorld, VERTICAL); // cross product to determine error
         
     GyroVec = Sum(GyroVec, Rotate(driftCorrectionVec, RotationQuat));  // rotate correction to body frame and add to gyro data
     
     RotationQuat = Mul(Quaternion(GyroVec, MPU.samplePeriod), RotationQuat);  // quaternion integration
     
     testTimer = micros() - testTimer; // *** TIMER END ***
     

     
     showData();
  }

} // Main Loop End



/*
 *  Support Functions
 */

void showData()
{
  if(millis() > timeDisplay)
  {
    timeDisplay = millis() + 500UL;
    
    Serial.println(testTimer);
    
    //Serial.println(imuCount); imuCount = 0;
    
    display(RotationQuat);
    //display(driftCorrectionVec);
    
    //display(GyroVec);
    //display(AccelVec);
    //display(AccelVecWorld);
    
  }
}



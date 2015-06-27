/*  Very Simple Complimentary Filter for the Invesense MPU-6050
 *  Phillip Schmidt
 *  v1.0, June 2015
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Math3D.h>
#include <PollTimers.h>


MPU6050 MPU(400, 0, 3, 3); // update rate, filtering, gyro, accel


Quat RotationQuat;

Vec3 driftCorrectionVec, rotationRateVec;
Vec3 AccelVec, AccelVecWorld, GyroVec;

const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame


void setup() {

	Serial.begin(115200);  // start serial for output
	Serial.println("*** Run On Start ***");

	Wire.begin();        // join i2c bus
	Wire.setClock(400000UL);// set speed to 400k


	MPU.initialize();
	Serial.println(MPU.samplePeriod);
	MPU.accelZero();
	MPU.gyroZero();	


	while(Timer400Hz());	// catch timer up
	while(Timer2Hz());	// catch timer up
}

void loop() // Start Main Loop
{

	if(Timer400Hz())  // check if it is time for next sensor sample, 400Hz
	{

		MPU.retrieve();	   // get data from the sensor
		MPU.convertToFloat(); // convert integer data to float

		GyroVec  = Vector(MPU.gX, MPU.gY, MPU.gZ);	// move gyro data to vector structure
		AccelVec = Vector(MPU.aX, MPU.aY, MPU.aZ);	// move accel data to vector structure

		AccelVecWorld = Rotate(RotationQuat, AccelVec); // rotate accel from body frame to world frame

		driftCorrectionVec = CrossProd(AccelVecWorld, VERTICAL); // cross product to determine error

		GyroVec = Sum(GyroVec, Rotate(driftCorrectionVec, RotationQuat));  // rotate correction to body frame and add to gyro data

		RotationQuat = Mul(Quaternion(GyroVec, MPU.samplePeriod), RotationQuat);  // quaternion integration (rotation composting through multiplication)

	}
	else if(Timer2Hz())	// only display data 2x per second
	{
		showData();
	}

} // Main Loop End



/*
 *  Support Functions
 */

void showData()
{

	display(RotationQuat);

}



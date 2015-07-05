/*  
 *  Very Simple Complimentary Filter for the Invesense MPU-6050
 *  Phillip Schmidt
 *  v1.0, June 2015
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Math3D.h>
#include <PollTimers.h>

#define _DEGREES(x) (57.29578 * x)

MPU6050 MPU(400, 0, 3, 3); // update rate, filtering, gyro, accel


Quat AttitudeEstimateQuat;

Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 GyroVec;

const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame


void setup() {

	Serial.begin(115200);  // start serial for output
	Serial.println("*** Run On Start ***");

	Wire.begin();        // join i2c bus
	Wire.setClock(400000UL);// set speed to 400k


	MPU.initialize();
	Serial.println(MPU.samplePeriod);
	MPU.accelZero();  // generate and store accel bias offsets
	MPU.gyroZero();	  // generate and store gyro bias offsets


	while(Timer400Hz());	// catch timer up
	while(Timer2Hz());	// catch timer up
}

void loop() // Start Main Loop
{

	if(Timer400Hz())  // check if it is time for next sensor sample, 400Hz
	{

		MPU.retrieve();	   // get data from the sensor

		GyroVec  = Vector(MPU.gX, MPU.gY, MPU.gZ);	// move gyro data to vector structure
		Accel_Body = Vector(MPU.aX, MPU.aY, MPU.aZ);	// move accel data to vector structure

		Accel_World = Rotate(AttitudeEstimateQuat, Accel_Body); // rotate accel from body frame to world frame

		correction_World = CrossProd(Accel_World, VERTICAL); // cross product to determine error

		Vec3 correction_Body = Rotate(correction_World, AttitudeEstimateQuat); // rotate correction vector to body frame

		GyroVec = Sum(GyroVec, correction_Body);  // add correction vector to gyro data

		Quat incrementalRotation = Quaternion(GyroVec, MPU.samplePeriod);  // create incremental rotation quat

		AttitudeEstimateQuat = Mul(incrementalRotation, AttitudeEstimateQuat);  // quaternion integration (rotation composting through multiplication)

	}
	else if(Timer2Hz())	// only display data 2x per second
	{

    Vec3 YPR = YawPitchRoll(AttitudeEstimateQuat);
    Serial.print("  Yaw:");   Serial.print(_DEGREES(-YPR.x), 2);
    Serial.print("  Pitch:"); Serial.print(_DEGREES(-YPR.y), 2);
    Serial.print("  Roll:");Serial.println(_DEGREES(-YPR.z), 2);

    
  	//display(AttitudeEstimateQuat);
    //display(GyroVec);
    //display(AccelVec);
	}

} // Main Loop End

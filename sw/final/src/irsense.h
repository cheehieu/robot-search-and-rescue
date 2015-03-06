#ifndef IRSENSE_H
#define IRSENSE_H

#include <map>
#include <motorcmd.h>
#include <compass.h>
#include <sonar.h>
using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7
//#define BACK_IR_PIN 3
#define SIDE_IR_PIN 0
#define TILE 27

class Interpolator
{
  public:

    // Inserts a new data point into the interpolation map
    // This function is done for you - don't modify it!
    void insertDataPoint(float voltage, float distance)
    {
      interpolationMap[voltage] = distance;

      if(interpolationMap.size() > 15)
      {
        std::cerr << "You should only put 10 interpolation points in your map! " <<
          "Make sure you really understand this assignment before proceeding." << std::endl;
        exit(-1);
      }
    }
    // Uses linear interpolation to convert a given voltage into a distance in centimeters.
    float getDistance(float voltage)
    {
      // TODO2: Insert your interpolation algorithm here
      float distance, keyU, keyL, valueU, valueL;
	/*  if( (voltage < 0.50) || (voltage > 2.75) )
	  {
		  std::cout << "Input voltage is out of bounds! It must be within the range 0.5V - 2.75V!\n";
		  return -1;
	  }
	  if((voltage > 2.55) && !(voltage > 2.75))
	  {
		  std::cout << "Input voltage is out of bounds (we are assuming that the robot will never get closer than 20cm to an object).\n";
		  return -1;
	  }*/
	  // Search through the keys to find the upper and lower bounds of the input voltage
	  std::map<float, float>::iterator lower = interpolationMap.lower_bound(voltage);
	  std::map<float, float>::iterator upper = lower--;
	  keyU = upper->first;
	  keyL = lower->first;
	  valueU = upper->second;
	  valueL = lower->second;
	  distance = valueL + (valueU-valueL)*(voltage-keyL)/(keyU-keyL);
	  return distance;
    }
	
  private:
    std::map<float, float> interpolationMap;
};

float getIRMedian(Interpolator myInterpolator, int iter, int pin)
{
	vector<float> temp;
	float voltageIR, distanceIR;
	float median;
	for (int i=0 ; i < iter ; i++)
	{
		voltageIR = Nemo::getADC(pin);
		distanceIR = myInterpolator.getDistance(voltageIR);
		temp.push_back(distanceIR);
		usleep(75000);
	}
	size_t size = temp.size();
	sort(temp.begin(), temp.end());
	if (size%2 == 0)
		median = (temp[size / 2 - 1] + temp[size / 2]) / 2;
	else 
		median = temp[size / 2];
	return median;
}


void IR_PID(float &wall_Error, float &wall_PreviousError)
{
	float Kp(1), Ki(0), Kd(0.5);
	float PIDControlValue = Kp*wall_Error + Kd*(wall_Error - wall_PreviousError);
	int LeftMotorValue = 100 + PIDControlValue;
	int RightMotorValue = 80 - PIDControlValue;
	if (LeftMotorValue > 100)	LeftMotorValue = 100;
	if (LeftMotorValue < -100)	LeftMotorValue = -100;
	if (RightMotorValue > 100) 	RightMotorValue = 100;
	if (RightMotorValue < -100) RightMotorValue = -100;
	Nemo::setMotor(0, LeftMotorValue);
	Nemo::setMotor(1, RightMotorValue);
	//cout << "LeftMotor: " << LeftMotorValue << "\tRightMotor: " << RightMotorValue << endl;
}


void wall_PID(float &wall_Error, float &wall_PreviousError)
{
	float Kp(2.5), Ki(0), Kd(1);
	float PIDControlValue = Kp*wall_Error + Kd*(wall_Error - wall_PreviousError);
	int LeftMotorValue = 70 + PIDControlValue;
	int RightMotorValue = 70 - PIDControlValue;
	if (LeftMotorValue > 100)	{LeftMotorValue = 100;}
	if (LeftMotorValue < -100)	{LeftMotorValue = -100;}
	if (RightMotorValue > 100) 	{RightMotorValue = 100;}
	if (RightMotorValue < -100) {RightMotorValue = 100;}
	Nemo::setMotor(0, LeftMotorValue);
	Nemo::setMotor(1, RightMotorValue);
	cout << "LeftMotor: " << LeftMotorValue << "\tRightMotor: " << RightMotorValue << endl;
}

float followWallIR(Interpolator myInterpolator, float &wall_Error, float &wall_PreviousError, int pin)
{
	float wallDistance = 33; //maintain this distance away from wall
	if (pin == SIDE_IR_PIN)
		wallDistance = 25;
	float distanceIR = getIRMedian(myInterpolator,5, pin);
	//cout << "Distance From Right Wall (IR): " << distanceIR << endl;
	wall_PreviousError = wall_Error;
	wall_Error = distanceIR - wallDistance; //too far away (positive), turn right
	if ( pin==IR_PIN || (pin==SIDE_IR_PIN && distanceIR<50) )
		IR_PID(wall_Error, wall_PreviousError);
	//if get a certain reading (wrongly too close to wall), turn left for certain time and go straight
	return distanceIR;
}


void moveForwardIR(Interpolator myInterpolator, float distance, int pin, float sideIRreading, float &wall_Error, float &wall_PreviousError)
{		
	float desiredHeading = getCompassMedian(10);
	float desiredIR, currentIR;
	int tooCloseToWallFlag = 0;
	
	currentIR = getIRMedian(myInterpolator, 5, pin);
	cout << "\nInitial IR Distance: " << currentIR << endl;

	desiredIR = currentIR + distance;
	cout << "Desired IR Distance: " << desiredIR << endl;
	
	while ( (fabs(desiredIR - currentIR)>8) )
	{
		moveForwardCompass(desiredHeading);
		if (sideIRreading < 50 && distance==TILE)
		{
			cout << "*****************Following wall with IR..." << endl;
			sideIRreading = followWallIR(myInterpolator, wall_Error, wall_PreviousError, SIDE_IR_PIN);
		}
		currentIR = getIRMedian(myInterpolator, 5, SIDE_IR_PIN);
		cout << "\nCurrent IR: " << currentIR << endl;
		cout << "IR Error: " << desiredIR - currentIR << endl;
	}
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}




#endif

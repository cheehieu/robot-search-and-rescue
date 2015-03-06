#ifndef COMPASS_H
#define COMPASS_H

#include <motorcmd.h>
#include <Particle.H>
using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7


float getCompassMedian(int iter)
{
	vector<float> temp;
	float median;
	for (int i=0 ; i < iter ; i++)
		temp.push_back(Nemo::getCompass(COMPASS_PIN));
	size_t size = temp.size();
	sort(temp.begin(), temp.end());
	if (size%2 == 0)
		median = (temp[size / 2 - 1] + temp[size / 2]) / 2;
	else 
		median = temp[size / 2];
	//cout << "compass median: " << median << endl;
	return median;
}

void orientNSEW(char cardDir, int &curDir, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	Nemo::setMotor(0, 0);	//stop moving
	Nemo::setMotor(1, 0);
	int turnRight, turnLeft;
	float currentDirection, desiredDirection, turnError, range;
	float compassCommand = 220;	//default is north
	if (cardDir=='N')	{compassCommand = 210-5; range = 12; curDir = 0; ang = 270;}
	if (cardDir=='S')	{compassCommand = 327; range = 12; curDir = 1; ang = 90;}
	if (cardDir=='E')	{compassCommand = 65+15; range = 12; curDir = 2; ang = 0;}
	if (cardDir=='W')	{compassCommand = 275-7; range = 10; curDir = 3; ang = 180;}
	cout << "\nOrienting to: " << cardDir << endl;
	currentDirection = getCompassMedian(10);
	//cout << "Initial Direction: " << currentDirection << endl;
	desiredDirection = compassCommand;
	//cout << "Desired Direction: " << desiredDirection << endl;
	turnError = currentDirection - desiredDirection;
	//cout << "Turn Error: " << turnError << endl << endl;

	turnRight = 1;
	
	while( fabs(turnError) > range )
	{
		if (turnRight==1)
		{
			Nemo::setMotor(0, 90);	//turn right
			Nemo::setMotor(1, -90);
		}
		else if (turnLeft==1)
		{
			Nemo::setMotor(0, -90);	//turn left
			Nemo::setMotor(1, 90);
		}
		turnError = currentDirection - desiredDirection;
		currentDirection = Nemo::getCompass(COMPASS_PIN);
		//cout << "Current Direction: " << currentDirection << endl << endl;
		//cout << "Turn Error: " << turnError << endl;
	}
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	dist = 0;
	var = 10;
	for (int i=0; i<particles.size(); i++)		//update angle particles
		particles[i].moveParticle(ang, dist, var);
}


void compassTurn90(char turnDir, int &currentDirection, float leftReading, float midReading, float rightReading, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	if (turnDir == 'R')
	{
		switch(currentDirection) {
			case 0:
				if (leftReading>midReading && leftReading>rightReading)
				{
					orientNSEW('W', currentDirection, particles, ang, dist, var);
				}
				else if (rightReading>leftReading && rightReading>midReading)
				{
					orientNSEW('E', currentDirection, particles, ang, dist, var);
				}
				else if (midReading>leftReading && midReading>rightReading)
				{
					orientNSEW('N', currentDirection, particles, ang, dist, var);
				}
				else
				{
					cout << "\n\nTURN ERROR!!!\n\n";
				}	
				cout << "Current Direction: " << currentDirection << endl;
				break;
				
			case 1:
				if (leftReading>midReading && leftReading>rightReading)
				{
					orientNSEW('E', currentDirection, particles, ang, dist, var);
				}
				else if (rightReading>leftReading && rightReading>midReading)
				{
					orientNSEW('W', currentDirection, particles, ang, dist, var);
				}
				else if (midReading>leftReading && midReading>rightReading)
				{
					orientNSEW('S', currentDirection, particles, ang, dist, var);
				}
				else
				{
					cout << "\n\nTURN ERROR!!!\n\n";
				}	
				cout << "Current Direction: " << currentDirection << endl;
				break;
				
			case 2:
				if (leftReading>midReading && leftReading>rightReading)
				{
					orientNSEW('N', currentDirection, particles, ang, dist, var);
				}
				else if (rightReading>leftReading && rightReading>midReading)
				{
					orientNSEW('S', currentDirection, particles, ang, dist, var);
				}
				else if (midReading>leftReading && midReading>rightReading)
				{
					orientNSEW('E', currentDirection, particles, ang, dist, var);
				}
				else
				{
					cout << "\n\nTURN ERROR!!!\n\n";
				}	
				cout << "Current Direction: " << currentDirection << endl;
				break;
				
			case 3:
				if (leftReading>midReading && leftReading>rightReading)
				{
					orientNSEW('S', currentDirection, particles, ang, dist, var);
				}
				else if (rightReading>leftReading && rightReading>midReading)
				{
					orientNSEW('N', currentDirection, particles, ang, dist, var);
				}
				else if (midReading>leftReading && midReading>rightReading)
				{
					orientNSEW('W', currentDirection, particles, ang, dist, var);
				}
				else
				{
					cout << "\n\nTURN ERROR!!!\n\n";
				}	
				cout << "Current Direction: " << currentDirection << endl;
				break;
			
			default:
				cout << "Default.\n";
		}
	}
}

void moveForwardCompass(float desiredHeading)
{
	float currentHeading, headingError;
	currentHeading = getCompassMedian(5);
	headingError = desiredHeading - currentHeading;
	
	//cout << "Current Heading: " << currentHeading << endl;
	//cout << "Heading Error: " << headingError << endl;
	
	if ( (currentHeading>3) && (currentHeading<357) )
	{
		float Kp(1), Ki(0), Kd(1);
		float PIDControlValue = Kp*headingError;// + Kd*(headingError - wall_PreviousError);
		int LeftMotorValue = 90 - PIDControlValue;
		int RightMotorValue = 80 + PIDControlValue;
		if (LeftMotorValue > 100)	{LeftMotorValue = 100;}
		if (LeftMotorValue < -100)	{LeftMotorValue = -100;}
		if (RightMotorValue > 100) 	{RightMotorValue = 100;}
		if (RightMotorValue < -100) {RightMotorValue = 100;}

		Nemo::setMotor(0, LeftMotorValue);
		Nemo::setMotor(1, RightMotorValue);
		//cout << "LeftMotor: " << LeftMotorValue << "\tRightMotor: " << RightMotorValue << endl;
	}
	else if (currentHeading<3)
	{
	
	}
	else if (currentHeading>357)
	{
	
	}
}

void moveBackwardCompass(float desiredHeading)
{
	float currentHeading, headingError;
	currentHeading = getCompassMedian(5);
	headingError = desiredHeading - currentHeading;
	
	cout << "Current Heading: " << currentHeading << endl;
	cout << "Heading Error: " << headingError << endl;
	
	if ( (currentHeading>3) && (currentHeading<357) )
	{
		float Kp(0.5), Ki(0), Kd(1);
		float PIDControlValue = Kp*headingError;// + Kd*(headingError - wall_PreviousError);
		int LeftMotorValue = -90 + PIDControlValue;
		int RightMotorValue = -80 - PIDControlValue;
		if (LeftMotorValue > 100)	{LeftMotorValue = 100;}
		if (LeftMotorValue < -100)	{LeftMotorValue = -100;}
		if (RightMotorValue > 100) 	{RightMotorValue = 100;}
		if (RightMotorValue < -100) {RightMotorValue = 100;}

		Nemo::setMotor(0, LeftMotorValue);
		Nemo::setMotor(1, RightMotorValue);
		cout << "LeftMotor: " << LeftMotorValue << "\tRightMotor: " << RightMotorValue << endl;
	}
	else if (currentHeading<3)
	{
	
	}
	else if (currentHeading>357)
	{
	
	}
}


/*
void orientNSEW(char dir)
{
	float compassCommand = 34;	//default is east
	float compassReading = getCompassMedian(20);
	if (dir=='N')	compassCommand = 224;
	if (dir=='S')	compassCommand = 322;
	if (dir=='E')	compassCommand = 34;
	if (dir=='W')	compassCommand = 275;
	head_PreviousError = head_Error;
	head_Error = compassReading - compassCommand;
	cout << "head_error: " << head_Error << endl;
	while( fabs(head_Error) > 5 )
	{
		head_PreviousError = head_Error;
		head_Error = compassReading - compassCommand;
		cout << "head_error: " << head_Error << endl;
		float Kp(0.75), Ki(0.6), Kd(0);
		float iError;
		int PIDControlValue;
		if ( fabs(head_Error) < 40)
		{
			cout << "1" << endl;
			//iError += head_Error;
			PIDControlValue = head_Error/head_Error * 40;	//(Kp * head_Error) + (Ki * iError) + Kd*(head_Error - head_PreviousError);
			cout << "PIDControlValue" << PIDControlValue << endl;
		}
		else
			PIDControlValue = (Kp * head_Error) + Kd*(head_Error - head_PreviousError);
			
		if ( fabs(head_Error) > 180 )
			PIDControlValue = -1 * PIDControlValue;
			
		int LeftMotorValue = PIDControlValue;
		int RightMotorValue = -1*PIDControlValue;
		if (LeftMotorValue > 100)			{LeftMotorValue = 100;}
		else if (LeftMotorValue < -100)		{LeftMotorValue = -1*100;}
		if (RightMotorValue > 100)			{RightMotorValue = 100;}
		else if (RightMotorValue < -100)	{RightMotorValue = -1*100;}
		Nemo::setMotor(0, LeftMotorValue);
		Nemo::setMotor(1, RightMotorValue);
		cout << "leftMotor: " << LeftMotorValue << endl;
		cout << "rightMotor: " << RightMotorValue << endl << endl;
		compassReading = getCompassMedian(5);
	}
	
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}
*/

#endif

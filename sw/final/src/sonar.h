#ifndef SONAR_H
#define SONAR_H

#include <irsense.h>
#include <compass.h>
#include <Particle.H>
using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7
#define BACK_IR_PIN 3
#define SIDE_IR_PIN 0
#define TILE 27

float getSonarMedian(int iter)
{
	vector<float> temp;
	float median;
	for (int i=0 ; i < iter ; i++)
	{
		temp.push_back(Nemo::getSonar(SONAR_PIN));
		//usleep(50000);
		usleep(100000);
	}
	size_t size = temp.size();
	sort(temp.begin(), temp.end());
	if (size%2 == 0)
		median = (temp[size / 2 - 1] + temp[size / 2]) / 2;
	else 
		median = temp[size / 2];
	return median;
}

void senseSonar(double &leftReading, double &midReading, double &rightReading)
{
	Nemo::setServo(SERVO_PIN, 165);		// Get Sonar Reading Left
	usleep(1000000);
    leftReading = getSonarMedian(5);	//Nemo::getSonar(SONAR_PIN);
	std::cout<< "Left Reading: " << leftReading << std::endl;
	Nemo::setServo(SERVO_PIN, 80);		// Get Sonar Reading Middle
	usleep(1000000);
    midReading = getSonarMedian(5);		//Nemo::getSonar(SONAR_PIN);
	std::cout<< "Middle Reading: " << midReading << std::endl;
	Nemo::setServo(SERVO_PIN, 0);		// Get Sonar Reading Right
	usleep(1000000);
    rightReading = getSonarMedian(5);	//Nemo::getSonar(SONAR_PIN);
	std::cout<< "Right Reading: " << rightReading << std::endl;
	Nemo::setServo(SERVO_PIN, 80);		//reset servo to middle
	usleep(750000);
}

int findFarthestSonar()
{
	float leftReading, midReading, rightReading, farthestReading(0);
	int left(90), mid(80), right(70), servoAngle;
	vector<float> sonarReadings;
	
	Nemo::setServo(SERVO_PIN, left);	// left offset
	usleep(500000);
    leftReading = getSonarMedian(5);
	std::cout<< "Left Reading: " << leftReading << std::endl;
	sonarReadings.push_back(leftReading);
	
	Nemo::setServo(SERVO_PIN, mid);		// middle
	usleep(500000);
    midReading = getSonarMedian(5);
	std::cout<< "Middle Reading: " << midReading << std::endl;
	sonarReadings.push_back(midReading);
	
	Nemo::setServo(SERVO_PIN, right);	// right offset
	usleep(500000);
    rightReading = getSonarMedian(5);
	std::cout<< "Right Reading: " << rightReading << std::endl;
	sonarReadings.push_back(rightReading);
	
	Nemo::setServo(SERVO_PIN, 80);		//reset servo to middle
	usleep(500000);
	
	for (int i=0 ; i<3 ; i++)
	{
		if (sonarReadings[i] > farthestReading)
			farthestReading = sonarReadings[i];
	}
	if (farthestReading == leftReading)			servoAngle = left;
	else if (farthestReading == midReading)		servoAngle = mid;
	else if (farthestReading == rightReading)	servoAngle = right;
	
	std::cout<< "Farthest Reading: " << farthestReading << std::endl;
	std::cout<< "Servo Angle: " << servoAngle << std::endl;
	return servoAngle;
}

void moveFarForwardSonar(float distance, int servoAngle)
{
	float desiredHeading = getCompassMedian(10);
	float desiredSonar, currentSonar;
	int tooCloseToWallFlag = 0;
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	Nemo::setServo(SERVO_PIN, servoAngle);
	usleep(250000);
	
	currentSonar = getSonarMedian(5);
	cout << "\nInitial Sonar Distance: " << currentSonar << endl;
	while (currentSonar < distance)
	{
		currentSonar = getSonarMedian(5);
		cout << "\nstuck in sonar loop... sonar reading too small: " << currentSonar << endl;
	}
	desiredSonar = currentSonar - distance;
	cout << "Desired Sonar Distance: " << desiredSonar << endl;
	
	while ( (fabs(currentSonar-desiredSonar)>5) && tooCloseToWallFlag!=1 )
	{
		moveForwardCompass(desiredHeading);
		currentSonar = getSonarMedian(5);
		//cout << "\nCurrent Sonar: " << currentSonar << endl;
		cout << "Sonar Error: " << currentSonar-desiredSonar << endl;
		if (currentSonar < 25)
			tooCloseToWallFlag = 1;
	}
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

/*
void moveForwardSonar(float distance, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	float desiredHeading = getCompassMedian(10);
	float desiredSonar, currentSonar;
	float sonarIncrement = 5;
	float totalDistance = distance;
	int tooCloseToWallFlag = 0;
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	
	while (sonarIncrement < totalDistance)
	{
		currentSonar = getSonarMedian(4);
		cout << "\nInitial Sonar Distance: " << currentSonar << endl;
		while (currentSonar < distance){
			currentSonar = getSonarMedian(4);
			cout << "\nstuck in sonar loop... sonar reading too small: " << currentSonar << endl;
		}
		
		desiredSonar = currentSonar - sonarIncrement;
		cout << "Desired Sonar Distance: " << desiredSonar << endl;
		
		while ( (fabs(currentSonar-desiredSonar)>2) && tooCloseToWallFlag!=1 )
		{
			moveForwardCompass(desiredHeading);
			// currentSonar = getSonarMedian(4);
			currentSonar = Nemo::getSonar(SONAR_PIN);
			//cout << "\nCurrent Sonar: " << currentSonar << endl;
			cout << "Sonar Error: " << currentSonar-desiredSonar << endl;
			if (currentSonar < 28)
				tooCloseToWallFlag = 1;
		}
		if (tooCloseToWallFlag == 1)
		{
			Nemo::setMotor(0, 0);
			Nemo::setMotor(1, 0);
		}

		ang = ang;
		dist = (double)sonarIncrement;
		var = 2;
		for (int i=0; i<particles.size(); i++)
			particles[i].moveParticle(ang, dist, var);

		totalDistance = totalDistance - sonarIncrement;
	}
	
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}
*/

void moveBackwardSonar(float distance, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	float desiredHeading = getCompassMedian(10);
	float desiredSonar, currentSonar;
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	Nemo::setServo(SERVO_PIN, 80);
	usleep(500000);
	
	distance = distance - 3;
	currentSonar = getSonarMedian(10);
	cout << "\nInitial Sonar Distance: " << currentSonar << endl;
	desiredSonar = currentSonar + distance;
	cout << "Desired Sonar Distance: " << desiredSonar << endl;
	
	while ( fabs(currentSonar-desiredSonar) > 3 )
	{
		Nemo::setMotor(0, -78);	//move backward
		Nemo::setMotor(1, -80);
		// moveBackwardCompass(desiredHeading);
		currentSonar = getSonarMedian(5);
		cout << "CurrentSonar: " << currentSonar << endl;
		cout << "Error: " << currentSonar-desiredSonar << endl;
	}
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	
	ang = ang + 180;
	if (ang > 360)
		ang = ang - 360;
	dist = (double)distance;
	var = 4;
	for (int i=0; i<particles.size(); i++)
		particles[i].moveParticle(ang, dist, var);
}

void moveForwardSonar(float distance, vector<Particle> &particles, double &ang, double &dist, double &var, Interpolator myInterpolator, float &wall_Error, float &wall_PreviousError, int &currentDirection)
{		
	float desiredHeading = getCompassMedian(5);
	float sideIRreading = getIRMedian(myInterpolator, 5, SIDE_IR_PIN);
	float backIRreading = getIRMedian(myInterpolator, 5, BACK_IR_PIN);
	cout << "\n************IR Reading: " << sideIRreading << endl << endl;
	float desiredSonar, currentSonar, sonarError, prevSonarError;
	int tooCloseToWallFlag = 0;
	int moveForwardIRFlag = 0;
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	
	currentSonar = getSonarMedian(5);
	cout << "\nInitial Sonar Distance: " << currentSonar << endl;
	desiredSonar = currentSonar - distance;
	cout << "Desired Sonar Distance: " << desiredSonar << endl;
	
	if ( (currentSonar > 160) )//&& (backIRreading < 120) )
	{
		moveForwardIRFlag = 1;
		// moveForward(4500000);
		// cout << "***************Moving forward based on time..." << endl;
		// cout << "***************Moving forward using back IR sensor..." << endl;
		// moveForwardIR(myInterpolator, distance, BACK_IR_PIN, sideIRreading, wall_Error, wall_PreviousError);
		if (currentDirection == 0)		orientNSEW('S', currentDirection, particles, ang, dist, var);
		else if (currentDirection == 1)	orientNSEW('N', currentDirection, particles, ang, dist, var);
		else if (currentDirection == 2)	orientNSEW('W', currentDirection, particles, ang, dist, var);
		else if (currentDirection == 3)	orientNSEW('E', currentDirection, particles, ang, dist, var);
		moveBackwardSonar(distance, particles, ang, dist, var);
	}
	
	sonarError = currentSonar-desiredSonar;
	while ( (fabs(sonarError)>5) && tooCloseToWallFlag!=1 && moveForwardIRFlag==0 )
	{
		moveForwardCompass(desiredHeading);
		if (sideIRreading < 50 && distance==TILE)
		{
			cout << "*****************Following wall with IR..." << endl;
			sideIRreading = followWallIR(myInterpolator, wall_Error, wall_PreviousError, SIDE_IR_PIN);
		}
		currentSonar = getSonarMedian(5);
		//cout << "\nCurrent Sonar: " << currentSonar << endl;
		prevSonarError = sonarError;
		sonarError = currentSonar-desiredSonar;
		cout << "Sonar Error: " << sonarError << endl;
		if (currentSonar < 28)
			tooCloseToWallFlag = 1;
		if ( ((prevSonarError < 0) && (sonarError < 0)) || (sonarError > prevSonarError+10) )
			moveForwardIRFlag = 1;
	}
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	
	ang = ang;
	dist = (double)distance;
	var = 4;
	if (currentSonar>160)
		var = 8;
	if (moveForwardIRFlag==0)
	{
		for (int i=0; i<particles.size(); i++)
			particles[i].moveParticle(ang, dist, var);
	}
}

void followWallSonar(float &wall_Error, float &wall_PreviousError, int &currentDirection, float leftReading, float midReading, float rightReading)
{
	Nemo::setServo(SERVO_PIN, 0);	//face the right
	usleep(500000);
	float wallDistance = 20; //maintain this distance away from right wall
	float distanceSonar = Nemo::getSonar(SONAR_PIN);
	cout << "distance from wallSonar: " << distanceSonar << endl;
	
	if (distanceSonar > 50)
	{
		if (distanceSonar < 400)
		{
			cout << "\n!!!!!Right Turn Detected!!!!!\n" << endl;
			//moveForwardSonar(TILE);
			leftReading = 0; midReading = 0; rightReading = 1;	//always turn right
			//compassTurn90('R', currentDirection, leftReading, midReading, rightReading, particles, ang, dist, var);
			cout << "\n!!!!!Moving Forward!!!!!!\n" << endl;
			//moveForwardSonar(TILE/1.25);
			cout << "\n!!!!!Finshed Turning Right!!!!!\n" << endl;
		}
		else
			moveForward(3000000);
	}
	
	wall_PreviousError = wall_Error;
	wall_Error = distanceSonar - wallDistance; //too far away (positive), turn right
	wall_PID(wall_Error, wall_PreviousError);
}


#endif

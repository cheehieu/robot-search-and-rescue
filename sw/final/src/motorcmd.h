#ifndef MOTORCMD_H
#define MOTORCMD_H

#include <string>
#include <vector>
using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define IR_PIN 7


void straightDelay()
{
	Nemo::setMotor(0, 60);	//go straight
	Nemo::setMotor(1, 50);
	usleep(1000000);		//clear wall
	Nemo::setMotor(0, 0);	//stop
	Nemo::setMotor(1, 0);
	usleep(500000);
}

void moveForward(float duration)
{
	//5s = 43cm
	usleep(500000);
	// Nemo::setMotor(0, 60);
	// Nemo::setMotor(1, 50);
	Nemo::setMotor(0, 83);
	Nemo::setMotor(1, 70);
	usleep(duration);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void turnRight(float duration)
{
	usleep(500000);
	Nemo::setMotor(0, 60);
	Nemo::setMotor(1, -60);
	usleep(duration);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void turnRight90()
{
	usleep(500000);
	Nemo::setMotor(0, 60);
	Nemo::setMotor(1, -60);
	usleep(890000);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void turnRight180()
{
	usleep(500000);
	Nemo::setMotor(0, 60);
	Nemo::setMotor(1, -60);
	usleep(2300000);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
	//usleep(1000000);
}

void turnLeft(float duration)
{
	usleep(500000);
	Nemo::setMotor(0, -60);
	Nemo::setMotor(1, 60);
	usleep(duration);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void turnLeft90()
{
	usleep(500000);
	Nemo::setMotor(0, -60);
	Nemo::setMotor(1, 60);
	usleep(800000);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

void turnLeft180()
{
	usleep(500000);
	Nemo::setMotor(0, -60);
	Nemo::setMotor(1, 60);
	usleep(2300000);
	Nemo::setMotor(0, 0);
	Nemo::setMotor(1, 0);
}

#endif

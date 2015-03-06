#ifndef GRABBER_H
#define GRABBER_H

#include <sonar.h>
#include <Particle.H>

using namespace std;

#define SONAR_PIN 0
#define SERVO_PIN 1
#define COMPASS_PIN 2
#define GRAB_SERVO_PIN 11
#define SWITCH_PIN 5
#define IR_PIN 7
#define TILE 27



void grab()
{
	while(Nemo::getADC(SWITCH_PIN) != 0)
	{
		cout << "Switch: " << Nemo::getADC(SWITCH_PIN) << endl;
		Nemo::setServo(GRAB_SERVO_PIN, 0);	//open
		usleep(2000000);
		Nemo::setServo(GRAB_SERVO_PIN, 90);	//close
		usleep(2000000);
	}
	cout << "\n\nGrabbed green object!\n\n";
}

void release()
{
	Nemo::setServo(GRAB_SERVO_PIN, 0);	//open
	usleep(2000000);
	//moveBackwardSonar(TILE*.75);
	Nemo::setServo(GRAB_SERVO_PIN, 90);	//close
	usleep(1000000);
	cout << "\n\nReleased green object!\n\n";
}


#endif

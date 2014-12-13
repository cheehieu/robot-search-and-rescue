#include <nemo/Nemo.H>

int main()
{
	Nemo::setServo(1,90);
	while(Nemo::getButton(0) == true)
	{
		std::cout << "Distance: " << Nemo::getSonar(0) << endl;
		std::cout << "Compass: " << Nemo::getCompass(2) << endl;
	}
}

/*
void beep(int freq)
{
	for(int i=0; i<3; i++)
	{	
		Nemo::setSpeaker(freq);
		sleep;
		Nemo::setSpeaker(0);
		sleep;
	}
}
void blink()
{
	for(...
	Nemo::setLED(0,true);
}
	

void foundCenter()
{
	beep(440);
	blink();
}
*/

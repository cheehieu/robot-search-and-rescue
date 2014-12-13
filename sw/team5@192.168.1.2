// ######################################################################
// 2010 Fall CS445 Lab3 Assignment 09/01/2010
// Due: Sunday Sept. 12th 2010 11:59:59 pm
// Instructions:
//
//   The purpose of this exercise is to have you create a reusable interpolation class that will
//   help you remap highly non-linear data to a more usable linear format.  For example, in the next
//   lab you will be using a Sharp infrared rangefinder in order to keep your robot from crashing
//   into walls.  The rangefinder works by projecting a small infrared spot in front of itself
//   through a built-in IR emitter, and then measuring at what angle the reflected spot returns to a
//   built-in IR receiver (take a look here:
//   http://www.societyofrobots.com/sensors_sharpirrange.shtml for more details on this if you're
//   interested).  The rangefinder then outputs an analog voltage which is proportional to this
//   reflectance angle and thus to the distance from the object.  In order to use the rangefinders,
//   we can read this voltage with our Nemo boards, however we must somehow translate voltage levels
//   into distances.  This voltage->distance translation is actually a bit of a problem because the
//   function relating them is very non-linear: it looks almost like an inverse exponential! 
//
//   So, how do we go about turning these voltage values into centimeters? First we'll need to take
//   some measurements to find a few voltage/distance datapoints.  You can create a table of these
//   datapoints by holding your rangefinder in front of a wall at various increments and marking
//   down the voltage at each distance increment.  It would obviously be a pain to take a
//   measurement at every possible voltage level (our ADCs are 10-bit, so you would need to take
//   1024 measurements!), so let's say we just take 10 datapoints.  We then need a way to calculate
//   the distance when our voltage is between our recorded datapoints.
//
//   Attempt 1) Many of you may be thinking that a good solution is to simply fit a curve to our
//   datapoints in Excel, and then copy and paste the resulting formula into your code.  This
//   solution may work, but is Bad for at least two reasons: 
//   1) You're invoking a bit of 'magic' with this curve fitting - the resulting function fit may be
//   hard to debug if it gives unexpected results. 
//   2) When (not if!) you need to recalibrate your sensor, you'll need to go back to Excel to
//   recalculate a curve.
//   
//   Attempt 2) Some of you may be thinking that a good alternative solution is to just chain a
//   bunch of 'if/else' statements together to create something like this:
//
//   if (voltage < 0.4 || voltage > 2.55) distance = -1; 
//   else if (voltage >= 0.4 && voltage < 0.6)   distance = -200 * voltage + 230; 
//   else if (voltage >= 0.6 && voltage < 0.75) distance = -133.3*voltage + 190; 
//   else if (voltage >= 0.75 && voltage < 0.80) distance = -200 *voltage + 240;
//   else if (voltage >= 0.80 && voltage < 0.9) distance = -100 *voltage + 160; 
//   else if (voltage >= 0.9 && voltage < 1.1) distance = -50 * voltage + 115; 
//   else if (voltage >= 1.1 && voltage < 1.25) distance = -66.7 *voltage + 133.3; 
//   else if (voltage >= 1.25 && voltage < 1.55) distance = -33.3* voltage + 94.4; 
//   else distance = -20 * voltage + 71;
//
//   This solution is poor because it is extremely difficult to maintain - every time you need to
//   recalibrate your sensor, you'll have to wade through this whole mess of logic.
//   
//   A Good Solution) C++'s STL includes a data structure called a "map" which can hold data
//   associated as key/value pairs.  Using a std::map, we can enter in all of our 10 measured
//   datapoints like this: 'myMap[someVoltage] = someDistance', where someVoltage is called the
//   "key", and someDistance is called the "value". We can then retrieve a stored value by indexing
//   the std::map by the key like this 'std::cout << "My Value is: " << myMap[someVoltage]; '. You
//   can read all about the std::map class here: http://www.cppreference.com/wiki/stl/map/start.
//   Conveniently, the std::map class stores all of it's data ordered by the keys, so it is easy to
//   search through.    Now, with a map of datapoints we should be able to easily write an algorithm
//   which searches through them, and finds one the two datapoints bounding a given voltage and
//   interpolates between them to calculate the interpolated distance.
//
//
// Here is what you should do:
//
//   1) Go to http://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf and look at Fig.2 on page 5.  
//
//   2) Choose up to 10 datapoints from this graph, and insert them into the Interpolator in your
//   main function.  Look at "TODO1" to see where to do this. For now, just assume that your robot
//   will never get closer than 20cm to an object, so only choose datapoints that are further than
//   20cm.  
//   
//   3) Write a linear interpolation algorithm in the Interpolator's getDistance() method which can
//   accept any voltage between 0.5V to 2.75V, and will return the distance in centimeters as given
//   by the graph. Look for "TODO2" to see where to do this. Hint: std::map stores its key/value
//   pairs in order of the keys.
//
// How To Submit: 
//
//    1) Email to "csci445@gmail.com" with the subject of "CS445 LAB3 your_last_name"
//    2) C++ File with the filename as <team_number>-<last_name>-<student_ID>-lab3.cpp
//	  For instance: team0-itti-0123456789-lab3.cpp for your lab3 code
//    3) Do not zip your file. You should only send one .cpp file and nothing else.
//    4) Your code must be able to compile on aludra.usc.edu using g++  	
//    5) You get zero point after due day(Sep 12,11:59:59pm).
//    6) If you have any problem regarding this assignment, please e-mail TA(Kai chinkaic@usc.edu)
//       BEFORE due day.
//
//    THIS IS AN INDIVIDUAL ASSIGNMENT. THAT MEANS YOU ARE NOT ALLOWED TO DISCUSS IT WITH ANY OTHER
//    PERSON.  WE WILL MANUAL LOOK IT AND RUN IT THROUGH A PLAGIARISM DETECTION SYSTEM (MOSS -
//    http://theory.stanford.edu/~aiken/moss/) TO ENSURE THAT EVERYONE'S SOLUTION IS DIFFERENT.
//
//    
// #######################################################################
//	Name: Hieu Nguyen
//	ID:   4463833114
//	Team: team5
// #######################################################################

#include <cstdlib>
#include <iostream>
#include <map>

class Interpolator
{
  public:

    // Inserts a new data point into the interpolation map
    // This function is done for you - don't modify it!
    void insertDataPoint(float voltage, float distance)
    {
      interpolationMap[voltage] = distance;

      if(interpolationMap.size() > 10)
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
	  if( (voltage < 0.50) || (voltage > 2.75) )
	  {
		  std::cout << "Input voltage is out of bounds! It must be within the range 0.5V - 2.75V!\n";
		  return -1;
	  }
	  if((voltage > 2.00) && !(voltage > 2.75))
	  {
		  std::cout << "Input voltage is out of bounds (we are assuming that the robot will never get closer than 20cm to an object).\n";
		  return -1;
	  }
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

int main(int argc, char* argv[])
{
  if(argc != 2)
  {
    std::cerr << "Usage: " << argv[0] 
      << " VOLTAGE" << std::endl <<
      "    VOLTAGE should be a voltage that a Sharp rangefinder can produce." << std::endl << 
      "    Your program will then print out the corresponding distance as per the rangefinder's datasheet."<< std::endl;
    exit(0);
  }

  Interpolator myInterpolator;
  
  //TODO1: 
  //Insert your datapoints here like this:
  //myInterpolator.insertDataPoint(someVoltage1, someDistance1);
  //myInterpolator.insertDataPoint(someVoltage2, someDistance2);
  //myInterpolator.insertDataPoint(someVoltage3, someDistance3);
  //etc...
  myInterpolator.insertDataPoint(2.00, 30);
  myInterpolator.insertDataPoint(1.55, 40);
  myInterpolator.insertDataPoint(1.25, 50);
  myInterpolator.insertDataPoint(1.07, 60);
  myInterpolator.insertDataPoint(0.90, 70);
  myInterpolator.insertDataPoint(0.80, 80);
  myInterpolator.insertDataPoint(0.70, 90);
  myInterpolator.insertDataPoint(0.65, 100);
  myInterpolator.insertDataPoint(0.60, 110);
  myInterpolator.insertDataPoint(0.50, 130);


  // This output is done for you - don't modify it!
  std::cout << myInterpolator.getDistance(atof(argv[1]))<< std::endl;
  return 0;
}

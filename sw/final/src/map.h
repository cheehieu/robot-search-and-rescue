// Read map file to edges
// Remember COPY your map.txt file to the robot!
// #####################################################################

#include <sonar.h>
using namespace std;

void parseFile(std::vector<Point2D<int> > &edges,
				std::vector<Point2D<int> > &door,
				std::vector<Point2D<int> > &dock,
				std::vector<Point2D<int> > &purpleDocks,
				std::vector<Point2D<int> > &orangeDoors,
				Point2D<int> &initLoc,
				std::vector< vector<int> > &map
				)
{
	int p1ft_i, p1ft_j, p2ft_i, p2ft_j;
	int min_i, min_j;
	std::ifstream myfile;
	myfile.open ("map.txt");
	if (myfile.is_open())
	{
		while (!myfile.eof())
		{
			Point2D<int> p1;
			Point2D<int> p2;
			Point2D<int> p1ft;
			Point2D<int> p2ft;
			Point2D<int> avg;
			int type;// 0 is wall,1 is door, 2 is docking
			if (myfile >> p1.i >> p1.j >> p2.i >> p2.j >> type != NULL)
			{
				p1ft.i = p1.i;
				p1ft.j = p1.j;
				p2ft.i = p2.i;
				p2ft.j = p2.j;
				min_i = p1ft.i;
				min_j = p1ft.j;
				if (p2ft.i < min_i)	min_i = p2ft.i;
				if (p2ft.j < min_j)	min_j = p2ft.j;
				p1.i = p1.i*30;//convert ft to cm
				p1.j = p1.j*30;//convert ft to cm
				p2.i = p2.i*30;//convert ft to cm
				p2.j = p2.j*30;//convert ft to cm
				if(type == 0)	//wall
				{
					edges.push_back(p1);
					edges.push_back(p2);
					for (int y=0 ; y<=abs(p1ft.j-p2ft.j) ; y++)
					{
						for (int x=0 ; x<=abs(p1ft.i-p2ft.i) ; x++)
						{
							map[min_j+y][min_i+x] = -1;
						}
					}
				}
				else if(type == 1)	//door
				{
					door.push_back(p1);
					door.push_back(p2);
					avg.i = (p1ft.i + p2ft.i) / 2;
					avg.j = (p1ft.j + p2ft.j) / 2;
					orangeDoors.push_back(Point2D<int>(avg.i, avg.j));
				}
				else if(type ==2)	//dock
				{
					//Docking station format is X Y W H
					// p1 is center of docking station
					// p2.i is width and p2.j is height
					dock.push_back(p1);
					dock.push_back(p2);
					purpleDocks.push_back(p1ft);
				}
				else if(type == 3)	//injured robot
				{
					initLoc.i = p1.i;
					initLoc.j = p1.j;
				}
			}
		}
	}
	myfile.close();
}


Image<PixRGB<byte> > drawMap(Image<PixRGB<byte> > &img,
    std::vector<Point2D<int> > &edges,
    std::vector<Point2D<int> > &doors,
    std::vector<Point2D<int> > &docks,
    Point2D<int> &initLoc
    ){
 
  //Draw Wall
  for(int i = 0;i < edges.size()-1; i+=2){
    Point2D<int> p1 = edges.at(i);
    Point2D<int> p2 = edges.at(i+1);
    drawLine(img,p1,p2,PixRGB<byte>(255,255,255));
  }
  //Draw Door
  for(int i = 0;i < doors.size()-1; i+=2){
    Point2D<int> p1 = doors.at(i);
    Point2D<int> p2 = doors.at(i+1);
    drawLine(img,p1,p2,PixRGB<byte>(255,153,0),2);//Orange
  }
 
  //Draw Docking Station
  for(int i = 0;i < docks.size()-1; i+=2){
    Point2D<int> p1 = docks.at(i);
    Point2D<int> p2 = docks.at(i+1);
    Point2D<int> topLeft = Point2D<int>(p1.i-(p2.i/2),p1.j-(p2.j/2));
    Point2D<int> topRight = Point2D<int>(p1.i+(p2.i/2),p1.j-(p2.j/2));
    Point2D<int> bottomLeft = Point2D<int>(p1.i-(p2.i/2),p1.j+(p2.j/2));
    Point2D<int> bottomRight = Point2D<int>(p1.i+(p2.i/2),p1.j+(p2.j/2));
 
    drawRect(img,topLeft,bottomRight,PixRGB<byte>(204,0,204),2);
    drawLine(img,topLeft,bottomRight,PixRGB<byte>(204,0,204),2);
    drawLine(img,topRight,bottomLeft,PixRGB<byte>(204,0,204),2);
  }
 
  //Draw Injured Robot Location
  drawCircle(img,initLoc,5,PixRGB<byte>(238,169,184),3);
 
  return img;
}

/*
Image<PixRGB<byte> > drawMapNoDoors(Image<PixRGB<byte> > &img,
    std::vector<Point2D<int> > &edges,
    std::vector<Point2D<int> > &doors,
    std::vector<Point2D<int> > &docks,
    Point2D<int> &initLoc
    ){
 
  //Draw Wall
  for(int i = 0;i < edges.size()-1; i+=2){
    Point2D<int> p1 = edges.at(i);
    Point2D<int> p2 = edges.at(i+1);
    drawLine(img,p1,p2,PixRGB<byte>(255,255,255));
  }
  //Draw Door
  for(int i = 0;i < doors.size()-1; i+=2){
    Point2D<int> p1 = doors.at(i);
    Point2D<int> p2 = doors.at(i+1);
    drawLine(img,p1,p2,PixRGB<byte>(255,255,255));
  }
 
  //Draw Docking Station
  for(int i = 0;i < docks.size()-1; i+=2){
    Point2D<int> p1 = docks.at(i);
    Point2D<int> p2 = docks.at(i+1);
    Point2D<int> topLeft = Point2D<int>(p1.i-(p2.i/2),p1.j-(p2.j/2));
    Point2D<int> topRight = Point2D<int>(p1.i+(p2.i/2),p1.j-(p2.j/2));
    Point2D<int> bottomLeft = Point2D<int>(p1.i-(p2.i/2),p1.j+(p2.j/2));
    Point2D<int> bottomRight = Point2D<int>(p1.i+(p2.i/2),p1.j+(p2.j/2));
 
    drawRect(img,topLeft,bottomRight,PixRGB<byte>(204,0,204),2);
    drawLine(img,topLeft,bottomRight,PixRGB<byte>(204,0,204),2);
    drawLine(img,topRight,bottomLeft,PixRGB<byte>(204,0,204),2);
  }
 
  //Draw Injured Robot Location
  drawCircle(img,initLoc,5,PixRGB<byte>(238,169,184),3);
 
  return img;
}
*/

void generatePurpleDockMap(Point2D<int> purpleDock, std::vector< vector<int> > &purpleMap)
{
	Point2D<int> iterLoc = purpleDock;
	Point2D<int> Nneigh, Sneigh, Eneigh, Wneigh;
	purpleMap[iterLoc.j][iterLoc.i] = 0;
	int num_zeros = 10;
	
	while(num_zeros != 1)
	{
		num_zeros = 0;
		Nneigh.i = iterLoc.i;
		Nneigh.j = iterLoc.j - 1;
		Sneigh.i = iterLoc.i;
		Sneigh.j = iterLoc.j + 1;
		Eneigh.i = iterLoc.i + 1;
		Eneigh.j = iterLoc.j;
		Wneigh.i = iterLoc.i - 1;
		Wneigh.j = iterLoc.j;
		
		if (iterLoc.i == 0)			Wneigh.i = iterLoc.i;
		else if (iterLoc.i >= 12)	Eneigh.i = iterLoc.i;
		if (iterLoc.j == 0)			Nneigh.j = iterLoc.j;
		else if (iterLoc.j >= 12)	Sneigh.j = iterLoc.j;
		
		if( (purpleMap[Nneigh.j][Nneigh.i] >= 0) && (Nneigh!=purpleDock) && (iterLoc==purpleDock || 
			(purpleMap[iterLoc.j][iterLoc.i]>0 && ((purpleMap[Nneigh.j][Nneigh.i]>purpleMap[iterLoc.j][iterLoc.i]+1) || purpleMap[Nneigh.j][Nneigh.i]==0))) )
		{
			purpleMap[Nneigh.j][Nneigh.i] = purpleMap[iterLoc.j][iterLoc.i] + 1;
		}
		if( (purpleMap[Sneigh.j][Sneigh.i] >= 0) && (Sneigh!=purpleDock) && (iterLoc==purpleDock || 
			(purpleMap[iterLoc.j][iterLoc.i]>0 && ((purpleMap[Sneigh.j][Sneigh.i]>purpleMap[iterLoc.j][iterLoc.i]+1) || purpleMap[Sneigh.j][Sneigh.i]==0))) )
		{	
			purpleMap[Sneigh.j][Sneigh.i] = purpleMap[iterLoc.j][iterLoc.i] + 1;
		}
		if( (purpleMap[Eneigh.j][Eneigh.i] >= 0) && (Eneigh!=purpleDock) && (iterLoc==purpleDock || 
			(purpleMap[iterLoc.j][iterLoc.i]>0 && ((purpleMap[Eneigh.j][Eneigh.i]>purpleMap[iterLoc.j][iterLoc.i]+1) || purpleMap[Eneigh.j][Eneigh.i]==0))) )
		{
			purpleMap[Eneigh.j][Eneigh.i] = purpleMap[iterLoc.j][iterLoc.i] + 1;
		}
		if( (purpleMap[Wneigh.j][Wneigh.i] >= 0) && (Wneigh!=purpleDock) && (iterLoc==purpleDock || 
			(purpleMap[iterLoc.j][iterLoc.i]>0 && ((purpleMap[Wneigh.j][Wneigh.i]>purpleMap[iterLoc.j][iterLoc.i]+1) || purpleMap[Wneigh.j][Wneigh.i]==0))) )
		{
			purpleMap[Wneigh.j][Wneigh.i] = purpleMap[iterLoc.j][iterLoc.i] + 1;
		}
		
		iterLoc.i = iterLoc.i + 1;
		if (iterLoc.i > 12)	//end of row
		{
			iterLoc.i = 0;
			iterLoc.j = iterLoc.j + 1;
			if ( iterLoc.j > 12) //end of column
			{
				iterLoc.j = 0;
			}
		}
		for(int y=0; y<13; y++){
			for(int x=0; x<13; x++){
				if (purpleMap[y][x] == 0)
					num_zeros = num_zeros + 1;
			}
		}
	}
}

int findClosestDock(Point2D<int> curLoc, std::vector< vector<int> > map0, std::vector< vector<int> > map1, std::vector< vector<int> > map2)
{
	//check to see if dock is occupied or not
	cout << "Current Location: (" << curLoc.i << ", " << curLoc.j << ")" << endl;
	int dockIndex, smallestDistance(1000);
	vector<int> dockDistance;
	
	dockDistance.push_back(map0[curLoc.j][curLoc.i]);
	dockDistance.push_back(map1[curLoc.j][curLoc.i]);
	dockDistance.push_back(map2[curLoc.j][curLoc.i]);
	
	for (int i=0 ; i<dockDistance.size() ; i++)
	{
		if (dockDistance[i] < smallestDistance)
		{
			smallestDistance = dockDistance[i];
			dockIndex = i;
		}
	}
	if (smallestDistance < 0)
		cout << "Error! Current location is most likely a wall." << endl;
	cout << "Smallest Distance is " << smallestDistance << " from Dock#" << dockIndex << endl;
	return dockIndex;
}

char nextDirection(Point2D<int> curLoc, std::vector< vector<int> > map)
{
	char direction;
	Point2D<int> Nneigh, Sneigh, Eneigh, Wneigh;
	vector<int> NSEW;
	vector<int>::iterator min_distance_p;
	int min_distance;
	NSEW.resize(4);
	NSEW[0] = 1000; NSEW[1] = 1000; NSEW[2] = 1000; NSEW[3] = 1000;	//initialize to big value
	
	Nneigh.i = curLoc.i;
	Nneigh.j = curLoc.j - 1;
	Sneigh.i = curLoc.i;
	Sneigh.j = curLoc.j + 1;
	Eneigh.i = curLoc.i + 1;
	Eneigh.j = curLoc.j;
	Wneigh.i = curLoc.i - 1;
	Wneigh.j = curLoc.j;
	if (curLoc.i == 0)			Nneigh.j = curLoc.j;
	else if (curLoc.i == 12)	Sneigh.j = curLoc.j;
	if (curLoc.j == 0)			Wneigh.i = curLoc.j;
	else if (curLoc.j == 12)	Eneigh.i = curLoc.j;
	
	if( (map[Nneigh.j][Nneigh.i]>=0) )		//north neighbor
			NSEW[0] = map[Nneigh.j][Nneigh.i];
	if( (map[Sneigh.j][Sneigh.i]>=0) )		//south neighbor
			NSEW[1] = map[Sneigh.j][Sneigh.i];
	if( (map[Eneigh.j][Eneigh.i]>=0) )		//east neighbor
			NSEW[2] = map[Eneigh.j][Eneigh.i];
	if( (map[Wneigh.j][Wneigh.i]>=0) )		//west neighbor
			NSEW[3] = map[Wneigh.j][Wneigh.i];
	
	min_distance_p = min_element(NSEW.begin(), NSEW.end());
	min_distance = *min_distance_p;
		
	if (min_distance == NSEW[0])			direction = 'N';
	else if (min_distance == NSEW[1])		direction = 'S';
	else if (min_distance == NSEW[2])		direction = 'E';
	else if (min_distance == NSEW[3])		direction = 'W';
	cout << "Next Desired Direction: " << direction << endl;
	return direction;
}


char dirToDock(Point2D<int> curLoc, int &dock_index, std::vector<Point2D<int> > &purpleDocks,
			std::vector< vector<int> > &map0, std::vector< vector<int> > &map1, std::vector< vector<int> > &map2)
{
	//for all docks, generate map
	//choose closest dock that isn't already occupied*
	//find next direction
	char dir;
	generatePurpleDockMap(purpleDocks[0], map0);
	generatePurpleDockMap(purpleDocks[1], map1);
	generatePurpleDockMap(purpleDocks[2], map2);
	
	dock_index = findClosestDock(curLoc, map0, map1, map2);
	if(dock_index==0)		dir = nextDirection(curLoc, map0);
	else if(dock_index==1)	dir = nextDirection(curLoc, map1);
	else if(dock_index==2)	dir = nextDirection(curLoc, map2);
	
	return dir;
}

void moveToDock(Point2D<int> &curLoc, Point2D<int> dockLoc, char desiredDirection, int currentDirection, std::vector< vector<int> > &histmap, int &state, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	//move in desired direction
	//update particle filter
	//update curLoc
	//moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
	histmap[curLoc.j][curLoc.i] = -2;
	
	if (desiredDirection=='N')
	{
		curLoc.j = curLoc.j - 1;
	}
	else if (desiredDirection=='S')
	{
		curLoc.j = curLoc.j + 1;
	}
	else if (desiredDirection=='E')
	{
		curLoc.i = curLoc.i + 1;
	}
	else if (desiredDirection=='W')
	{
		curLoc.i = curLoc.i - 1;
	}
	
	if (curLoc == dockLoc)
	{
		for (int i=0 ; i<3 ; i++)	//beep 3 times
		{
			Nemo::setSpeaker(750);
			usleep(200000);
			Nemo::setSpeaker(0);
			usleep(200000);
		}
		//state = 7;
		state = 13;
	}
}

//given, current location and angle, return distance from right wall, middle wall, and left wall for particle filter
//iterate through drawn map using x,y coordinates to find closest walls (white pixel value)

/*

// for(int y=0; y<13; y++){	
	// for(int x=0; x<13; x++){
		// cout << purpleMap0[y][x] << "\t";
		// if(x==12)	cout << endl << endl;}}
// cout << endl << endl;

MAP0
-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1

-1      18      17      16      -1      10      9       8       7       6       -1      0       -1

-1      19      -1      15      -1      11      -1      9       -1      5       -1      1       -1

-1      20      -1      14      13      12      -1      10      -1      4       3       2       -1

-1      -1      -1      -1      -1      11      -1      -1      -1      5       -1      -1      -1

-1      14      13      12      11      10      9       8       7       6       -1      12      -1

-1      15      -1      -1      -1      11      -1      9       -1      7       -1      11      -1

-1      16      -1      14      13      12      -1      10      -1      8       9       10      -1

-1      17      -1      -1      -1      13      -1      11      -1      -1      -1      -1      -1

-1      18      17      16      15      14      -1      12      13      14      -1      20      -1

-1      -1      -1      17      -1      15      -1      13      -1      15      -1      19      -1

-1      20      19      18      -1      16      15      14      -1      16      17      18      -1

-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1


MAP1
-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1

-1      8       7       6       -1      0       1       2       3       4       -1      10      -1

-1      9       -1      5       -1      1       -1      3       -1      5       -1      9       -1

-1      10      -1      4       3       2       -1      4       -1      6       7       8       -1

-1      -1      -1      -1      -1      3       -1      -1      -1      7       -1      -1      -1

-1      8       7       6       5       4       5       6       7       8       -1      14      -1

-1      9       -1      -1      -1      5       -1      7       -1      9       -1      13      -1

-1      10      -1      8       7       6       -1      8       -1      10      11      12      -1

-1      11      -1      -1      -1      7       -1      9       -1      -1      -1      -1      -1

-1      12      11      10      9       8       -1      10      11      12      -1      18      -1

-1      -1      -1      11      -1      9       -1      11      -1      13      -1      17      -1

-1      14      13      12      -1      10      11      12      -1      14      15      16      -1

-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1


MAP2
-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1

-1      2       3       4       -1      10      11      12      13      14      -1      20      -1

-1      1       -1      5       -1      9       -1      13      -1      15      -1      19      -1

-1      0       -1      6       7       8       -1      14      -1      16      17      18      -1

-1      -1      -1      -1      -1      9       -1      -1      -1      15      -1      -1      -1

-1      14      13      12      11      10      11      12      13      14      -1      20      -1

-1      15      -1      -1      -1      11      -1      13      -1      15      -1      19      -1

-1      16      -1      14      13      12      -1      14      -1      16      17      18      -1

-1      17      -1      -1      -1      13      -1      15      -1      -1      -1      -1      -1

-1      18      17      16      15      14      -1      16      17      18      -1      24      -1

-1      -1      -1      17      -1      15      -1      17      -1      19      -1      23      -1

-1      20      19      18      -1      16      17      18      -1      20      21      22      -1

-1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1      -1

*/

char dirToDoor(Point2D<int> curLoc, int door_index, std::vector<Point2D<int> > &orangeDoors,
				std::vector< vector<int> > &map0, std::vector< vector<int> > &map1)
{
	char dir;
	if(door_index==0)
	{
		generatePurpleDockMap(orangeDoors[door_index], map0);
		dir = nextDirection(curLoc, map0);
	}
	else if(door_index==1)
	{
		generatePurpleDockMap(orangeDoors[door_index], map1);
		dir = nextDirection(curLoc, map1);
	}
	return dir;
}

void moveToDoor(Point2D<int> &curLoc, Point2D<int> doorLoc, char desiredDirection, int currentDirection, int &exitOrangeFlag, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	//moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
	
	if (desiredDirection=='N')
	{
		curLoc.j = curLoc.j - 1;
	}
	else if (desiredDirection=='S')
	{
		curLoc.j = curLoc.j + 1;
	}
	else if (desiredDirection=='E')
	{
		curLoc.i = curLoc.i + 1;
	}
	else if (desiredDirection=='W')
	{
		curLoc.i = curLoc.i - 1;
	}
	
	if (curLoc == doorLoc)
	{
		for (int i=0 ; i<3 ; i++)	//beep 1 times
		{
			Nemo::setSpeaker(300);
			usleep(200000);
			Nemo::setSpeaker(0);
			usleep(200000);
		}
		exitOrangeFlag = 1;
	}
}





Point2D<int> findClosestUnvisitedOne(Point2D<int> curLoc, std::vector< vector<int> > &distmap, std::vector< vector<int> > &histmap ){
  int mindis(1000), xcor(0),ycor(0);
  bool find=false;
  Point2D<int> goalLoc;
  generatePurpleDockMap(curLoc, distmap);

  for(int j=1; j<12; j++){
    for(int i=1; i<12; i++){
      if(histmap[j][i]!=-2&&histmap[j][i]!=-1){
		if(mindis>=distmap[j][i]&&distmap[j][i]!=0){
		  mindis = distmap[j][i];
		  ycor=j;
		  xcor=i;
		}
      }
    }
    goalLoc = Point2D<int>(xcor, ycor);
  }
  return goalLoc;
}

char dirToDummy(Point2D<int> goalLoc,Point2D<int> curLoc, std::vector< vector<int> > &distmap)
{
	//for all docks, generate map
	//choose closest dock that isn't already occupied*
	//find next direction
	char dir;
	generatePurpleDockMap(goalLoc, distmap);
	dir = nextDirection(curLoc, distmap);
	return dir;
}

void moveToGoal(Point2D<int> &curLoc, Point2D<int> goalLoc, char desiredDirection, int currentDirection, std::vector< vector<int> > &histmap, vector<Particle> &particles, double &ang, double &dist, double &var)
{
	// moveForwardSonar(TILE, particles, ang, dist, var, myInterpolator, wall_Error, wall_PreviousError, currentDirection);
	histmap[curLoc.j][curLoc.i] = -2;
	
	if (desiredDirection=='N')
	{
		curLoc.j = curLoc.j - 1;
	}
	else if (desiredDirection=='S')
	{
		curLoc.j = curLoc.j + 1;
	}
	else if (desiredDirection=='E')
	{
		curLoc.i = curLoc.i + 1;
	}
	else if (desiredDirection=='W')
	{
		curLoc.i = curLoc.i - 1;
	}
	
	if (curLoc == goalLoc)
	{
		for (int i=0 ; i<1 ; i++)	//beep 1 times
		{
			Nemo::setSpeaker(900);
			usleep(200000);
			Nemo::setSpeaker(0);
			usleep(200000);
		}
	}
}

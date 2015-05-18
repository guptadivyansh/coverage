#include <iostream>
#include <cassert>
#include <cstdlib>
#include "VoronoiDiagramGenerator.h"
#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"
#include "Coverage.h"
#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <patrolling.h>
#include <graphs.h>
#include "graphs.cpp"
#include "coverage/point.h"
#include "coverage/path.h"
#include "patrolling.h"
#include "patrolling.cpp"
//#define n_agents 4
double radius=10;
//Change later
using namespace cv;
using namespace std;

vector<ros::Publisher> pub;
int n_agents;

void createEnv(const swarm_simulator::obstacleList msg){
	ros::Rate loop_rate(5);
	Mat img(480, 480, CV_8U, Scalar(255));
	for(int i = 0; i <msg.obstacles.size(); ++i){
		if(msg.obstacles[i].radius>1.9){
		int x = (int)(((msg.obstacles[i].x)+12.0)*20.0);
		int y = (int)(((msg.obstacles[i].y)+12.0)*20.0);
		circle(img, Point(x, y), (int)((msg.obstacles[i].radius)*10), Scalar(0), -1);
		}
	}
	Coverage obj(img,n_agents);
	vector<Point> locations;
	obj.GetBestPositions(locations);
	Patrolling pat_obj(img,n_agents,radius,locations);
	vector< vector<Point> > bestpatrol=pat_obj.GetBestPatrol();
	vector<coverage::path> publish_path=vector<coverage::path>(n_agents);
	coverage::point publish_point;
	for(int i = 0; i < n_agents; i++)
	{	
		double center_x = -5 + 10 * (i % 2); //0, 1, 0, 1
		double center_y = -5 + 10 * (i > 1); //0, 0, 1, 1  
		for(int j = 0; j < 4; ++j)
		{
			publish_point.x= center_x - 2.5 + 5 * (j % 2);
			publish_point.y= center_y - 2.5 + 5 * (j > 1);
			publish_path[i].parray.push_back(publish_point);
		}
	}
	for(int i=0;i<n_agents;i++)
	{
		pub[i].publish(publish_path[i]);
	}
    /*swarm_simulator::obstacleList coveragePoints;
	for(int i = 0; i<locations.size(); ++i){
	    swarm_simulator::obstacleData toAdd;
	    toAdd.x = ((double)(locations[i].x))/20.0-12.0;
	    toAdd.y = ((double)(locations[i].y))/20.0-12.0;
	    toAdd.shape = i;
	    toAdd.radius = 0.0;
	    coveragePoints.obstacles.push_back(toAdd);
	}
	pub.publish(coveragePoints);
	loop_rate.sleep();*/
}

int main(int argc,char *argv[])
{
	ros::init(argc, argv, "Coverage");
	ros::NodeHandle n;

	//Command line args are the map file and the number of agents
	assert(argc==2);
	n_agents = atoi(argv[1]);
	pub=vector<ros::Publisher>(n_agents);
	for(int i=0;i<n_agents;i++){
		char str[10];
		char path[100]="/path";
		std::sprintf(str,"%d",i);
		strcat(path,str);
		printf("%s started\n",path);
		pub[i] = n.advertise<coverage::path>(path, 1);

	}
	ros::Subscriber msg = n.subscribe("/obstacleList", 1, createEnv);
	ros::spin();

	//Now you can use the points in locations to proceed
	return 0;
}




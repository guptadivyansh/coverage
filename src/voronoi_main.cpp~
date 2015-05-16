#include <iostream>
#include <cassert>
#include <cstdlib>
#include "VoronoiDiagramGenerator.h"
#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"
#include "Coverage.h"
#include "ros/ros.h"

using namespace cv;
using namespace std;

ros::Publisher pub;
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
    swarm_simulator::obstacleList coveragePoints;
	for(int i = 0; i<locations.size(); ++i){
	    swarm_simulator::obstacleData toAdd;
	    toAdd.x = ((double)(locations[i].x))/20.0-12.0;
	    toAdd.y = ((double)(locations[i].y))/20.0-12.0;
	    toAdd.shape = i;
	    toAdd.radius = 0.0;
	    coveragePoints.obstacles.push_back(toAdd);
	}
	pub.publish(coveragePoints);
	loop_rate.sleep();
}

int main(int argc,char *argv[]) 
{	
	ros::init(argc, argv, "Coverage");
	ros::NodeHandle n;

	//Command line args are the map file and the number of agents
	assert(argc==2);
	n_agents = atoi(argv[1]);
	pub = n.advertise<swarm_simulator::obstacleList>("/coverage", 1);
	ros::Subscriber msg = n.subscribe("/obstacleList", 1, createEnv);
	ros::spin();

	//Now you can use the points in locations to proceed
	return 0;	
}




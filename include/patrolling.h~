/*This is the class for dealing with obtaining a trajectory pattern for patrolling
of a known environment with obstacles, by a known number of agents.*/

#ifndef PATROLLING
#define PATROLLING

#include <vector>
#include "opencv2/core/core.hpp"
#include "graphs.h"

using namespace std;
using namespace cv;

class Patrolling
{
public:
	Patrolling();
	Patrolling(Mat img, int n, int r, vector<Point>& positions): map(img), n_agents(n), radius(r), centroids(positions){};
	~Patrolling();
	void GetBestPatrol();

private:
	Mat map;
	Graph graph_obj;
	int n_agents;
	vector<Point> centroids;
	int radius;

	double GetPathForIndex(int idx, bool cw, vector<vector<int> > assignments,
							int count_cells, int start_idx, vector<int>& best_index_path);
	int GetInitCellAssignments(vector<vector<int> >& assignments, vector<int>& starting);

};


#endif
#ifndef GRAPHS
#define GRAPHS

#include "opencv2/core/core.hpp"

#include <vector>

using namespace cv;
using namespace std;

struct Vertex{
    int v_id;
    Point v_pt;

    Vertex(int id, Point pt): v_id(id), v_pt(pt){}
};

//I have not explicitly kept an Edge struct as I did not need to
//But you can change accordingly if you need

class Graph
{
public:

	vector<Vertex> v_list; //List of vertices with IDs and point co-ordinates
	vector< vector<int> > adj_list; //For each vertex, upto the 8-connected neighbours

	
	Graph():n_vertices(0),n_edges(0){}

	void GenerateRoadMap(int rows, int cols, int step);
	int GetNVertices(){return n_vertices;}
    int GetNEdges(){return n_edges;}



private:

	int n_vertices;
	int n_edges;
    void AddVertex(int id, Point& pt);
    void AddEdge(int u, int v);


};

#endif

    
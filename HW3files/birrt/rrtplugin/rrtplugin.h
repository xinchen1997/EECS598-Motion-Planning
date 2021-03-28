#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <openrave/openrave.h>

using namespace std;
using namespace OpenRAVE;

class RRTNode
{
private:
	std::vector<dReal> _q;

public:
	RRTNode* _parent;
	RRTNode();
	RRTNode(std::vector<dReal> q);
	RRTNode(std::vector<dReal> q, RRTNode* parent);
	std::vector<dReal> getConfig();
	void printNodeInfo();
	void printParentInfo();
};

class NodeTree
{
public:
	std::vector<RRTNode*> _nodes;
	RRTNode* _root;
	RRTNode* _goal;

	// RRT parameters
	double threshold = 1e-2;
	double probablity = 0.21;
	int maxIter = 10000;
	// bool reachGoal = false;
	std::vector<vector<dReal>> _path;
	double path_length = 0.0;
	// random number generator
	// std::default_random_engine gen_;
	int seed = 1;
	std::random_device rd_;

	// sampling parameter
	std::vector<RRTNode*> sample_nodes;
	// std::vector<dReal> _stepSize;
	// int _step = 50;
	double _stepSize = 0.3;
	int _smoothingMaxIter = 200;
	
	// Initialize
	NodeTree();
	NodeTree(RRTNode* root, RRTNode* goal);

	// Sampling
	RRTNode* probSample();
	RRTNode* addGoalOrientNode();
	RRTNode* addRandomNode();

	// Implement a nearest-neighbor function that finds the closest node in a NodeTree to a given configuration.
	RRTNode* findNearestNode(RRTNode* rdNode);
	// double distance(RRTNode* node1, RRTNode* node2);
	double distance(vector<dReal> q1, vector<dReal> q2);
	bool isGoal(RRTNode* node);
	RRTNode* expandNode(RRTNode* nearestNode, std::vector<dReal> stepSize, int step);

	// Include methods to add, delete, and get nodes from the node set
	void addNode(RRTNode* nearestNode, RRTNode* newNode);
	void deleteNode(RRTNode* node);
	RRTNode* getNode(int index);
	bool isNodeVisited(RRTNode* node);
	// Include a method that returns the path of nodes from the root to a node with a given index
	// (including that node in the path)
	void getPath(RRTNode* goal);
	void printPath();
	double getPathLength();
	// Path smoothing
	// void shortcutSmoothing();
};

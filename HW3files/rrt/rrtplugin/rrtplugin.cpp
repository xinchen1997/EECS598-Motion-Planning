#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <float.h>
#include <stdlib.h>
#include "rrtplugin.h"

using namespace std;
using namespace OpenRAVE;

#define DOF 7

vector<double> step_path_length;

class rrtModule : public ModuleBase
{
public:
    std::vector<GraphHandlePtr> handles;

    rrtModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("runRRT",boost::bind(&rrtModule::RRT,this,_1,_2),
                        "This is an example command");
    }
    virtual ~rrtModule() {}
    
    bool RRT(std::ostream& sout, std::istream& sinput)
    {
        clock_t start, end;
        start = clock();

        std::string input;
        sinput >> input;
        // sout << "output";

        vector<dReal> startConfig = {-0.15, 0.075, -1.008, 0, 0, -0.11, 0};
        vector<dReal> goalConfig = {0.449, -0.201, -0.151, 0, 0, -0.11, 0};
        RRTNode* root = new RRTNode(startConfig);
        RRTNode* goal = new RRTNode(goalConfig);
        NodeTree* Tree = new NodeTree(root, goal);

        for(int i = 0; i < Tree->maxIter; i++)
        {
            cout << endl;
            cout << "iter: " << i << endl;
            RRTNode* rdNode = Tree->probSample();
            RRTNode* nearestNode = Tree->findNearestNode(rdNode);
            RRTNode* newNode;

            cout << "distance: " << Tree->distance(nearestNode->getConfig(), rdNode->getConfig()) << endl;
            if (Tree->distance(nearestNode->getConfig(), rdNode->getConfig()) < 1e-1)
            {
                newNode = rdNode;
            }
            else{
                newNode = RRTConnect(Tree, rdNode, nearestNode);
            }

            // if nearestNode == newNode
            if(Tree->distance(nearestNode->getConfig(), newNode->getConfig()) < 1e-5) 
            {
                cout << "same node" << endl;
                continue;
            }

            // node info
            // cout << "rdNode: ";
            // rdNode->printNodeInfo();
            // cout << "nearestNode: ";
            // nearestNode->printNodeInfo();
            // cout << "newNode: ";
            // newNode->printNodeInfo();
            // cout << "parentNode: ";
            // newNode->printParentInfo();

            // if (!collisionCheck(newNode->getConfig())) {
            //     Tree->addNode(nearestNode, newNode);
            //     cout << "parentNode: ";
            //     newNode->printParentInfo();
            // }

            if (Tree->isGoal(newNode)) {
                cout << "reach goal" << endl;
                end = clock();
                Tree->getPath(newNode);
                cout << "unsmoothed path" << endl;
                Tree->printPath();
                double unsmoothed_path_length = Tree->getPathLength();
                cout << "draw unsmoothed path" << endl;
                plotRed(Tree->_path);
                cout << endl;

                cout << "smoothing path" << endl;
                clock_t smooth_start, smooth_end;
                smooth_start = clock();
                shortcutSmoothing(Tree);
                smooth_end = clock();
                cout << "smoothed path" << endl;
                Tree->printPath();
                double smoothed_path_length = Tree->getPathLength();
                plotBlue(Tree->_path);
                cout << endl;

                cout << "move" << endl;
                moveTraj(Tree->_path);

                cout << "goal bias: " << Tree->probablity << endl;
                cout << "Run time for RRT: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
                cout << "Run time for smoothing: " << (double)(smooth_end - smooth_start) / CLOCKS_PER_SEC << " seconds" << endl;
                cout << "Number of nodes sampled: " << Tree->sample_nodes.size() << endl;
                cout << "Length of path(unsmoothed): " << unsmoothed_path_length << endl;
                cout << "Length of path(smoothed): " << smoothed_path_length << endl;

                // for (auto l : step_path_length){
                //     cout << l << " ";
                // }
                // cout << endl;
                // cout << step_path_length.size() <<end;
                return true;
            }
        }
        cout << "Can't find path in limited time" << endl;
        end = clock();
        cout << "Run time for RRT: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
        return false;
    }

    RRTNode* RRTConnect(NodeTree* Tree, RRTNode* rdNode, RRTNode* nearestNode)
    {
        // RRT Connect
        // Tree->_stepSize.clear();
        std::vector<dReal> delta;
        double dis = Tree->distance(rdNode->getConfig(), nearestNode->getConfig());
        for (int i = 0; i < DOF; i++)
        {
            delta.push_back((rdNode->getConfig()[i] - nearestNode->getConfig()[i]) / dis);
        }

        RRTNode* lastNode = nearestNode;
        RRTNode* newNode;
        // bool isCollided = false;
        int stepMax = (int)((rdNode->getConfig()[0] - nearestNode->getConfig()[0]) / (delta[0] * Tree->_stepSize));
        int step = 1;
        while(true)
        {
            newNode = Tree->expandNode(nearestNode, delta, step);

            // if collided, using lastnode
            if (collisionCheck(newNode->getConfig()))
            {
                if (step == 1) 
                {
                    newNode = lastNode;
                    break;
                }
                newNode = lastNode;
                Tree->addNode(lastNode, newNode);
                break;
            }
            // if reach the rdNode
            if (step == stepMax)
            {   
                // Tree->addNode(lastNode, newNode);
                // cout << "step: " << step << endl;
                // cout << "stepMax: " << stepMax << endl;
                if (collisionCheck(rdNode->getConfig()))
                {
                    Tree->addNode(lastNode, newNode);
                    return newNode;
                }
                Tree->addNode(lastNode, rdNode);
                return rdNode;
            }

            Tree->addNode(lastNode, newNode);
            lastNode = newNode;
            step++;
        }

        return newNode;
    }

    bool collisionCheck(vector<dReal> config)
    {
        // vector<dReal> q = node->getConfig();
        std::vector<RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot = robots[0];
        robot->SetActiveDOFValues(config);
        bool isCollided = (GetEnv()->CheckCollision(robot) || robot->CheckSelfCollision());

        // GetEnv()->UpdatePublishedBodies();
        // cout << "is collision: " << isCollided << endl;
        return isCollided;
    }

    void moveTraj(std::vector<vector<dReal>> path)
    {
        // EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex()); // lock environment
        std::vector<RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot = robots[0];

        TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());
        for(int i = 0; i < path.size(); i++) {
            traj->Insert(i, path[i]);
        }
        // planningutils::RetimeActiveDOFTrajectory(traj, robot);
        std::vector<dReal> max_v = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        std::vector<dReal> max_a = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        planningutils::RetimeAffineTrajectory(traj, max_v, max_a);
        // planningutils::SmoothActiveDOFTrajectory(traj, robot);

        // move robot through trajectory
        robot->GetController()->SetPath(traj);
        
        // cout << "move" << endl;
        // unlock the environment and wait for the robot to finish
        // while(!robot->GetController()->IsDone()) {
        //     boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        // }

    }

    void plotRed(std::vector<vector<dReal>> path)
    {
        std::vector<RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot = robots[0];

        RobotBase::ManipulatorPtr manip = nullptr;
        Transform T;
        float red[4] = {1, 0, 0, 1};

        if(path.size()) {
            for (int i = 0; i < path.size(); i++)
            {
                robot->SetActiveDOFValues(path[i]);
                manip = robot->GetActiveManipulator();
                T = manip->GetEndEffectorTransform();
                std::vector<float> points;
                points.push_back((float)T.trans.x);
                points.push_back((float)T.trans.y);
                points.push_back((float)T.trans.z);
                points.push_back(1);
                handles.push_back(GetEnv()->plot3(&points[0], 1, 1, 5, red, 0));
            } 
        }
    }

    void plotBlue(std::vector<vector<dReal>> path)
    {
        std::vector<RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot = robots[0];

        RobotBase::ManipulatorPtr manip = nullptr;
        Transform T;
        float blue[4] = {0, 0, 1, 1};

        if(path.size()) {
            for (int i = 0; i < path.size(); i++)
            {
                robot->SetActiveDOFValues(path[i]);
                manip = robot->GetActiveManipulator();
                T = manip->GetEndEffectorTransform();
                std::vector<float> points;
                points.push_back((float)T.trans.x);
                points.push_back((float)T.trans.y);
                points.push_back((float)T.trans.z);
                points.push_back(1);
                handles.push_back(GetEnv()->plot3(&points[0], 1, 1, 5, blue, 0));
            } 
        }
    }

    void shortcutSmoothing(NodeTree* Tree)
    {
        int number_nodes = Tree->_path.size();
        std::vector<dReal> delta;
        for (int i = 0; i < Tree->_smoothingMaxIter; i++)
        {
            step_path_length.push_back(Tree->_path.size());
            int idx1 = rand() % Tree->_path.size();
            int idx2 = rand() % Tree->_path.size();
            if (idx1 == idx2) continue;
            if (idx1 > idx2)
            {
                int tmp = idx1;
                idx1 = idx2;
                idx2 = tmp;
            }
            if (idx2 == (idx1 + 1)) continue;
            std::vector<dReal> q1 = Tree->_path[idx1];
            std::vector<dReal> q2 = Tree->_path[idx2];

            delta.clear();
            double dis = Tree->distance(q1, q2);
            for(int j = 0; j < DOF; j++)
            {
                delta.push_back((q2[j] - q1[j]) / dis);
            }

            bool isCollided = false;
            std::vector<dReal> config;
            int step = 1;
            int stepMax = (int)((q2[0] - q1[0]) / (delta[0] * Tree->_stepSize));
            while(true)
            {
                config.clear();
                for(int n = 0; n < DOF; n++) {
                    config.push_back(q1[n] + delta[n] * Tree->_stepSize * step);
                }
                isCollided = collisionCheck(config);
                if(isCollided) break;
                if(step == stepMax) break;
                step++;
            }
            
            // if no collision, connect q1 and q2
            if(!isCollided && step == stepMax)
            {   
                Tree->_path.erase(Tree->_path.begin()+idx1+1, Tree->_path.begin()+idx2);
            }
        }
    }
};

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new rrtModule(penv,sinput));
    }
    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("rrtModule");    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin(){}

RRTNode::RRTNode()
{
    _q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _parent = nullptr;
}

RRTNode::RRTNode(std::vector<dReal> q)
{
    _q = q;
    _parent = nullptr;
}

RRTNode::RRTNode(std::vector<dReal> q, RRTNode* parent)
{
    _q = q;
    _parent = parent;
}

std::vector<dReal> RRTNode::getConfig()
{
    return _q;
}

void RRTNode::printNodeInfo()
{
    for (int i = 0; i < _q.size(); i++){
        cout << _q[i] << " ";
    }
    cout << endl;
}

void RRTNode::printParentInfo()
{
    if(!_parent) cout << "null" << endl;
    else    
    {
        _parent->printNodeInfo();
    }
}

NodeTree::NodeTree(){}

NodeTree::NodeTree(RRTNode* root, RRTNode* goal)
{
    _root = root;
    _goal = goal;
    _nodes.push_back(root);
    // reachGoal = false;
}

RRTNode* NodeTree::probSample()
{    
    srand(seed);
    seed = rand();
    double prob = rand() % 100 * 0.01;
    // double prob = rd_() % 100 * 0.01;
    cout << "sample probablity: " << prob << endl;
    
    if(prob < probablity)
    {
        return addGoalOrientNode();
    }
    else
    {
        return addRandomNode();
    }
}

RRTNode* NodeTree::addGoalOrientNode()
{
    sample_nodes.push_back(_goal);
    return _goal;
}

RRTNode* NodeTree::addRandomNode()
{
    std::vector<dReal> rd_q;
    vector<dReal> lower, upper;
    lower = {-0.56, -0.36, -2.12, -0.65, -3.14, -2, -3.14};
    upper = {2.14, 1.3, -0.1, 3.14, 3.14, -0.1, 3.14};
    // srand(time(NULL));
    srand(seed);
    seed = rand();
    for (int i = 0; i < DOF; i++)
    {
        dReal tmp = rand() % 1000 / 1000.0;  // 0-1
        tmp = tmp * (upper[i] - lower[i]) + lower[i];
        rd_q.push_back(tmp);
    }
    RRTNode* rdNode = new RRTNode(rd_q);
    sample_nodes.push_back(rdNode);
    return rdNode;
}

RRTNode* NodeTree::findNearestNode(RRTNode* rdNode)
{
    int index = 0;
    double path_length = DBL_MAX;
    int max_size = 300;
    if (_nodes.size() > max_size) {
        int delta = _nodes.size() - max_size;
        for(int i = 0; i < max_size; i++)
        {
            double dis = distance(_nodes[delta + i]->getConfig(), rdNode->getConfig());
            if(dis <= path_length)
            {
                path_length = dis;
                index = i;
            }
        }
    }
    else {
        for(int i = 0; i < _nodes.size(); i++)
        {
            double dis = distance(_nodes[i]->getConfig(), rdNode->getConfig());
            if(dis <= path_length)
            {
                path_length = dis;
                index = i;
            }
        }
    }
    return _nodes[index];
}

double NodeTree::distance(vector<dReal> q1, vector<dReal> q2)
{
    // vector<dReal> q1 = node1->getConfig();
    // vector<dReal> q2 = node2->getConfig();
    double dis = 0.0;
    for(int i = 0; i < DOF; i++)
    {
        dis += pow((q1[i] - q2[i]), 2);
    }
    dis = sqrt(dis);
    return dis;
}

bool NodeTree::isGoal(RRTNode* node)
{
    if(distance(node->getConfig(), _goal->getConfig()) < threshold) return true;
    return false;
}

RRTNode* NodeTree::expandNode(RRTNode* nearestNode, std::vector<dReal> delta, int step)
{
    std::vector<dReal> new_q(DOF);
    for(int i = 0; i < DOF; i++) {
        new_q[i] = nearestNode->getConfig()[i] + step * delta[i] * _stepSize;
    }

    RRTNode* newNode = new RRTNode(new_q);
    return newNode;
}

void NodeTree::addNode(RRTNode* nearestNode, RRTNode* newNode)
{
    // if newNode is visited
    if(isNodeVisited(newNode)) return;
    // cout << "add new node" << endl;
    _nodes.push_back(newNode);
    newNode->_parent = nearestNode;
    // cout << "parentNode: ";
    // newNode->printParentInfo();
}

void NodeTree::deleteNode(RRTNode* node)
{
    int index = 0;
    for(int i = 0; i < _nodes.size(); i++){
        if(distance(node->getConfig(), _nodes[i]->getConfig()) < threshold) {
            index = i;
        }
    }
    auto iter = _nodes.erase(_nodes.begin() + index);
}
    
RRTNode* NodeTree::getNode(int index)
{
    return _nodes[index];
}

bool NodeTree::isNodeVisited(RRTNode* node)
{
    for(int i = 0; i < _nodes.size(); i++){
        if(distance(node->getConfig(), _nodes[i]->getConfig()) < threshold) {
            // cout << "visitied" << endl;
            return true;
        }
    }
    return false;
}

void NodeTree::getPath(RRTNode* node)
{
    while (node->_parent != nullptr)
    {
        _path.push_back(node->getConfig());
        RRTNode* last_node = new RRTNode(node->getConfig());
        node = node->_parent;
        path_length += distance(node->getConfig(), last_node->getConfig());
    }
    _path.push_back(node->getConfig());
    reverse(_path.begin(), _path.end());
}

void NodeTree::printPath()
{
    cout << "path size：" << _path.size() << endl;
    for (int i = 0; i < _path.size(); i++) {
        cout << i << " ";
        for (int j = 0; j < _path[i].size(); j++){
            cout << _path[i][j] << " ";
        }
        cout << endl;
    }
}

double NodeTree::getPathLength()
{
    double length = 0;
    for(int i = 0; i < _path.size() - 1; i++)
    {
        length += distance(_path[i], _path[i+1]);
    }
    return length;
}

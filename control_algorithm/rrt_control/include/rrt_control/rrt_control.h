#ifndef RRT_H
#define RRT_H

#include "rrt_control/obstacles.h"
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *>  children;
    Node            *parent;
    Vector3d        position;
    Quaterniond     orientation;
};

class RRT
{
public:
  RRT(
    const ros::NodeHandle &nh,
    vector<double>        workspace_limit
    );
public:
  void initialize();
  Node* getRandomNode();
  double distance(Vector3d &p, Vector3d &q);

  Node* nearest(Vector3d point);

  Vector3d newConfig(Node *q, Node *qNearest);
  void add(Node *qNearest, Node *qNew);
  bool reached();
  void setStepSize(double step);
  void setMaxIterations(int iter);
  void deleteNodes(Node *root);
public:
  boost::shared_ptr<Obstacles>  obstacles;
  Node                          *root, *lastNode;
  Vector3d                      startPos, endPos;
private:
  vector<double>                workspace_limit_;



  vector<Node *>                nodes;

  int                           max_iter;
  double                           step_size;
};

#endif // RRT_H
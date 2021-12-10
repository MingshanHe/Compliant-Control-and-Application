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
  RRT(const ros::NodeHandle &nh);
private:
  void initialize();
  Node* getRandomNode();
  int distance(Vector3d &p, Vector3d &q);

  Node* nearest(Vector3d point);

  Vector3d newConfig(Node *q, Node *qNearest);
  void add(Node *qNearest, Node *qNew);
  bool reached();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  void deleteNodes(Node *root);
private:
  boost::shared_ptr<Obstacles>  obstacles;
  Vector3d                      startPos, endPos;
  Node                          *root, *lastNode;

  vector<Node *>                nodes;

  int                           max_iter;
  int                           step_size;
};

#endif // RRT_H
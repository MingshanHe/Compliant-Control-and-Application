#include "rrt_control/rrt_control.h"

RRT::RRT(
    const ros::NodeHandle &nh,
    vector<double>        workspace_limit):
    workspace_limit_(workspace_limit)
{
  double check_step = 0.01;
  obstacles.reset(new Obstacles(nh, check_step));
  startPos  << 2,0,-2;
  endPos    << 0,2,2;

  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  lastNode = root;
  
  nodes.push_back(root);
  step_size = 3;
  max_iter = 3000;
  initialize();
}


/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRT::getRandomNode()
{
  //TODO: WORKSPACE
    Node* ret;
    Vector3d point(
        workspace_limit_[0]+drand48()*(workspace_limit_[3]-workspace_limit_[0]),
        workspace_limit_[1]+drand48()*(workspace_limit_[4]-workspace_limit_[1]),
        workspace_limit_[2]+drand48()*(workspace_limit_[5]-workspace_limit_[2]));
    ret = new Node;
    ret->position = point;
    // printf("workspace: %f, %f, %f, %f, %f, %f\n", workspace_limit_[0],workspace_limit_[1],workspace_limit_[2],workspace_limit_[3],workspace_limit_[4],workspace_limit_[5]);
    // printf("getRandomNode: %f, %f, %f\n", point[0],point[1],point[2]);
    return ret;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRT::distance(Vector3d &p, Vector3d &q)
{
    Vector3d v = p - q;
    return sqrt(powf(v(0), 2) + powf(v(1), 2) + powf(v(2), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRT::nearest(Vector3d point)
{
    double minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector3d RRT::newConfig(Node *q, Node *qNearest)
{
    Vector3d to = q->position;
    Vector3d from = qNearest->position;
    Vector3d intermediate = to - from;
    // printf("from: %f, %f, %f\n", from[0],from[1],from[2]);
    // printf("to: %f, %f, %f\n", to[0],to[1],to[2]);
    intermediate = intermediate / intermediate.norm();
    // printf("intermediate: %f, %f, %f\n", intermediate[0],intermediate[1],intermediate[2]);
    Vector3d ret = from + step_size * intermediate;
    // printf("newConfig: %f, %f, %f\n", ret[0],ret[1],ret[2]);
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached()
{
    //TODO: END_DIST_THRESHOLD CONFIGURATION
    double  END_DIST_THRESHOLD = 0.01;
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
        return true;
    return false;
}

void RRT::setStepSize(double step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}
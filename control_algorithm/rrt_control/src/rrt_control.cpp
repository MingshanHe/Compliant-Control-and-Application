#include "rrt_control/rrt_control.h"

RRT::RRT(const ros::NodeHandle &nh)
{
  double check_step = 0.01;
  obstacles.reset(new Obstacles(nh, check_step));
  startPos  << 0,0,0;
  endPos    << 1,1,1;

  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  lastNode = root;
  
  nodes.push_back(root);
  step_size = 3;
  max_iter = 3000;
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
    // Node* ret;
    // Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    // if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
    //     ret = new Node;
    //     ret->position = point;
    //     return ret;
    // }
    // return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
int RRT::distance(Vector3d &p, Vector3d &q)
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
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance(point, nodes[i]->position);
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

    intermediate = intermediate / intermediate.norm();
    Vector3d ret = from + step_size * intermediate;
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
    // if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
    //     return true;
    // return false;
}

void RRT::setStepSize(int step)
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
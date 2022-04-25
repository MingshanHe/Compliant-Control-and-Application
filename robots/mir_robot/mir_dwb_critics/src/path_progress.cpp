/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, DFKI GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <mir_dwb_critics/path_progress.h>
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <vector>

namespace mir_dwb_critics {
bool PathProgressCritic::prepare(const geometry_msgs::Pose2D &pose, const nav_2d_msgs::Twist2D &vel,
                                 const geometry_msgs::Pose2D &goal,
                                 const nav_2d_msgs::Path2D &global_plan) {
  dwb_critics::MapGridCritic::reset();

  unsigned int local_goal_x, local_goal_y;
  if (!getGoalPose(pose, global_plan, local_goal_x, local_goal_y, desired_angle_)) {
    return false;
  }

  // Enqueue just the last pose
  cell_values_.setValue(local_goal_x, local_goal_y, 0.0);
  queue_->enqueueCell(local_goal_x, local_goal_y);

  propogateManhattanDistances();

  return true;
}

void PathProgressCritic::onInit() {
  dwb_critics::MapGridCritic::onInit();
  critic_nh_.param("xy_local_goal_tolerance", xy_local_goal_tolerance_, 0.20);
  critic_nh_.param("angle_threshold", angle_threshold_, M_PI_4);
  critic_nh_.param("heading_scale", heading_scale_, 1.0);

  // divide heading scale by position scale because the sum will be multiplied by scale again
  heading_scale_ /= getScale();
}

void PathProgressCritic::reset() {
  reached_intermediate_goals_.clear();
}

double PathProgressCritic::scoreTrajectory(const dwb_msgs::Trajectory2D &traj) {
  double position_score = MapGridCritic::scoreTrajectory(traj);
  double heading_diff = fabs(remainder(traj.poses.back().theta - desired_angle_, 2 * M_PI));
  double heading_score = heading_diff * heading_diff;

  return position_score + heading_scale_ * heading_score;
}

bool PathProgressCritic::getGoalPose(const geometry_msgs::Pose2D &robot_pose, const nav_2d_msgs::Path2D &global_plan,
                                     unsigned int &x, unsigned int &y, double &desired_angle) {
  const nav_core2::Costmap &costmap = *costmap_;
  const nav_grid::NavGridInfo &info = costmap.getInfo();

  if (global_plan.poses.empty()) {
    ROS_ERROR_NAMED("PathProgressCritic", "The global plan was empty.");
    return false;
  }

  std::vector<geometry_msgs::Pose2D> plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution).poses;

  // find the "start pose", i.e. the pose on the plan closest to the robot that is also on the local map
  unsigned int start_index = 0;
  double distance_to_start = std::numeric_limits<double>::infinity();
  bool started_path = false;
  for (unsigned int i = 0; i < plan.size(); i++) {
    double g_x = plan[i].x;
    double g_y = plan[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y)
        && costmap(map_x, map_y) != nav_core2::Costmap::NO_INFORMATION) {
      // Still on the costmap. Continue.
      double distance = nav_2d_utils::poseDistance(plan[i], robot_pose);
      if (distance_to_start > distance) {
        start_index = i;
        distance_to_start = distance;
        started_path = true;
      } else {
        // Plan is going away from the robot again. It's possible that it comes back and we would find a pose that's
        // even closer to the robot, but then we would skip over parts of the plan.
        break;
      }
    } else if (started_path) {
      // Off the costmap after being on the costmap.
      break;
    }
    // else, we have not yet found a point on the costmap, so we just continue
  }

  if (!started_path) {
    ROS_ERROR_NAMED("PathProgressCritic", "None of the points of the global plan were in the local costmap.");
    return false;
  }

  // find the "last valid pose", i.e. the last pose on the plan after the start pose that is still on the local map
  unsigned int last_valid_index = start_index;
  for (unsigned int i = start_index + 1; i < plan.size(); i++) {
    double g_x = plan[i].x;
    double g_y = plan[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y)
        && costmap(map_x, map_y) != nav_core2::Costmap::NO_INFORMATION) {
      // Still on the costmap. Continue.
      last_valid_index = i;
    } else {
      // Off the costmap after being on the costmap.
      break;
    }
  }

  // find the "goal pose" by walking along the plan as long as each step leads far enough away from the starting point,
  // i.e. is within angle_threshold_ of the starting direction.
  unsigned int goal_index = start_index;

  while (goal_index < last_valid_index) {
    goal_index = getGoalIndex(plan, start_index, last_valid_index);

    // check if goal already reached
    bool goal_already_reached = false;
    for (auto &reached_intermediate_goal : reached_intermediate_goals_) {
      double distance = nav_2d_utils::poseDistance(reached_intermediate_goal, plan[goal_index]);
      if (distance < xy_local_goal_tolerance_) {
        goal_already_reached = true;
        // choose a new start_index by walking along the plan until we're outside xy_local_goal_tolerance_ and try again
        // (start_index is now > goal_index)
        for (start_index = goal_index; start_index <= last_valid_index; ++start_index) {
          distance = nav_2d_utils::poseDistance(reached_intermediate_goal, plan[start_index]);
          if (distance >= xy_local_goal_tolerance_) {
            break;
          }
        }
        break;
      }
    }
    if (!goal_already_reached) {
      // new goal not in reached_intermediate_goals_
      double distance = nav_2d_utils::poseDistance(plan[goal_index], robot_pose);
      if (distance < xy_local_goal_tolerance_) {
        // the robot has currently reached the goal
        reached_intermediate_goals_.push_back(plan[goal_index]);
        ROS_DEBUG_NAMED("PathProgressCritic", "Number of reached_intermediate goals: %zu", reached_intermediate_goals_.size());
      } else {
        // we've found a new goal!
        break;
      }
    }
  }
  if (start_index > goal_index)
    start_index = goal_index;
  ROS_ASSERT(goal_index <= last_valid_index);

  // save goal in x, y
  worldToGridBounded(info, plan[goal_index].x, plan[goal_index].y, x, y);
  desired_angle = plan[start_index].theta;
  return true;
}

unsigned int PathProgressCritic::getGoalIndex(const std::vector<geometry_msgs::Pose2D> &plan,
                                              unsigned int start_index,
                                              unsigned int last_valid_index) const {
  if (start_index >= last_valid_index)
    return last_valid_index;

  unsigned int goal_index = start_index;
  const double start_direction_x = plan[start_index + 1].x - plan[start_index].x;
  const double start_direction_y = plan[start_index + 1].y - plan[start_index].y;
  if (fabs(start_direction_x) > 1e-9 || fabs(start_direction_y) > 1e-9) {  // make sure that atan2 is defined
    double start_angle = atan2(start_direction_y, start_direction_x);

    for (unsigned int i = start_index + 1; i <= last_valid_index; ++i) {
      const double current_direction_x = plan[i].x - plan[i - 1].x;
      const double current_direction_y = plan[i].y - plan[i - 1].y;
      if (fabs(current_direction_x) > 1e-9 || fabs(current_direction_y) > 1e-9) {
        double current_angle = atan2(current_direction_y, current_direction_x);

        // goal pose is found if plan doesn't point far enough away from the starting point
        if (fabs(remainder(start_angle - current_angle, 2 * M_PI)) > angle_threshold_)
          break;

        goal_index = i;
      }
    }
  }
  return goal_index;
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathProgressCritic, dwb_local_planner::TrajectoryCritic)

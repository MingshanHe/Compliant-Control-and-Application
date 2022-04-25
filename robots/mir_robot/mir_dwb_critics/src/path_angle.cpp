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

#include <mir_dwb_critics/path_angle.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <nav_grid/coordinate_conversion.h>
#include <vector>

namespace mir_dwb_critics {

bool PathAngleCritic::prepare(const geometry_msgs::Pose2D &pose, const nav_2d_msgs::Twist2D &vel,
                              const geometry_msgs::Pose2D &goal, const nav_2d_msgs::Path2D &global_plan) {
  const nav_core2::Costmap &costmap = *costmap_;
  const nav_grid::NavGridInfo &info = costmap.getInfo();
  nav_2d_msgs::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution);

  if (global_plan.poses.empty()) {
    ROS_ERROR_NAMED("PathAngleCritic", "The global plan was empty.");
    return false;
  }

  // find the angle of the plan at the pose on the plan closest to the robot
  double distance_min = std::numeric_limits<double>::infinity();
  bool started_path = false;
  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); i++) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y) && costmap(map_x, map_y) != costmap.NO_INFORMATION) {
      double distance = nav_2d_utils::poseDistance(adjusted_global_plan.poses[i], pose);
      if (distance_min > distance) {
        // still getting closer
        desired_angle_ = adjusted_global_plan.poses[i].theta;
        distance_min = distance;
        started_path = true;
      } else {
        // plan is going away from the robot again
        break;
      }
    } else if (started_path) {
      // Off the costmap after being on the costmap.
      break;
    }
    // else, we have not yet found a point on the costmap, so we just continue
  }

  if (!started_path) {
    ROS_ERROR_NAMED("PathAngleCritic", "None of the points of the global plan were in the local costmap.");
    return false;
  }
  return true;
}

double PathAngleCritic::scoreTrajectory(const dwb_msgs::Trajectory2D &traj) {
  double diff = fabs(remainder(traj.poses.back().theta - desired_angle_, 2 * M_PI));
  return diff * diff;
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathAngleCritic, dwb_local_planner::TrajectoryCritic)

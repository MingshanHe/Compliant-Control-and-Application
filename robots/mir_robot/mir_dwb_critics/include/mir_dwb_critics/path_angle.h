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
#ifndef MIR_DWB_CRITICS_PATH_ANGLE_H_
#define MIR_DWB_CRITICS_PATH_ANGLE_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <vector>

namespace mir_dwb_critics
{
/**
 * @class PathAngleCritic
 * @brief Scores trajectories based on the difference between the path's current angle and the trajectory's angle
 *
 * This trajectory critic helps to ensure that the robot is roughly aligned with the path (i.e.,
 * if the path specifies a forward motion, the robot should point forward along the path and vice versa).
 * This critic is not a replacement for the PathAlignCritic: The PathAlignCritic tries to point the robot
 * towards a point on the path that is in front of the trajectory's end point, so it helps guiding the
 * robot back towards the path. The PathAngleCritic on the other hand uses the path point that is closest
 * to the robot's current position (not the trajectory end point, or even a point in front of that) as a
 * reference, so it always lags behind. Also, it tries to keep the robot parallel to the path, not towards
 * it. For these reasons, the error is squared, so that the PathAngleCritic really only affects the final
 * score if the error is large. The PathAlignCritic doesn't take the path orientation into account though,
 * so that's why the PathAngleCritic is a useful addition.
 */
class PathAngleCritic: public dwb_local_planner::TrajectoryCritic
{
public:
  virtual bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                       const geometry_msgs::Pose2D& goal,
                       const nav_2d_msgs::Path2D& global_plan) override;

  virtual double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

protected:
  double desired_angle_;
};

}  // namespace mir_dwb_critics
#endif  // MIR_DWB_CRITICS_PATH_ANGLE_H_

/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * Publishes a square-wave wrench that imitates an operator pushing the end
 * effector, so the admittance controller can be tested without contact.
 */
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class WrenchSignalGenerate : public rclcpp::Node
{
public:
  WrenchSignalGenerate()
  : Node("wrench_signal_generate")
  {
    const auto topic = declare_parameter<std::string>("topic", "/wrench_signal");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    rate_ = declare_parameter<double>("rate", 125.0);
    amplitude_ = declare_parameter<double>("amplitude", 3.0);
    period_ = declare_parameter<double>("period", 10.0);
    start_delay_ = declare_parameter<double>("start_delay", 5.0);
    // Direction of the force in frame_id (normalized).
    const auto direction = declare_parameter<std::vector<double>>(
      "direction", std::vector<double>{0.0, 0.0, 1.0});
    if (direction.size() != 3 || rate_ <= 0.0 || period_ <= 0.0) {
      throw std::invalid_argument("Invalid 'direction', 'rate' or 'period' parameter");
    }
    const double norm = std::sqrt(
      direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (norm < 1e-9) {
      throw std::invalid_argument("Parameter 'direction' must not be zero");
    }
    for (int i = 0; i < 3; ++i) {
      direction_[i] = direction[i] / norm;
    }

    pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic, rclcpp::SensorDataQoS());
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {on_timer();});
    RCLCPP_INFO(
      get_logger(), "Publishing a +/-%.1f N square wave (period %.1f s) on '%s' in frame '%s' "
      "after %.1f s", amplitude_, period_, topic.c_str(), frame_id_.c_str(), start_delay_);
  }

private:
  void on_timer()
  {
    const rclcpp::Time now = get_clock()->now();
    if (!started_) {
      if (now.nanoseconds() == 0) {
        return;  // sim time not received yet
      }
      start_time_ = now;
      started_ = true;
    }
    const double elapsed = (now - start_time_).seconds() - start_delay_;
    if (elapsed < 0.0) {
      return;
    }
    // Same signal as the ROS 1 version: +amplitude for the first half period, then -amplitude.
    const double sign = std::fmod(elapsed, period_) < 0.5 * period_ ? 1.0 : -1.0;
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.wrench.force.x = sign * amplitude_ * direction_[0];
    msg.wrench.force.y = sign * amplitude_ * direction_[1];
    msg.wrench.force.z = sign * amplitude_ * direction_[2];
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  std::array<double, 3> direction_{};
  double rate_;
  double amplitude_;
  double period_;
  double start_delay_;
  bool started_{false};
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchSignalGenerate>());
  rclcpp::shutdown();
  return 0;
}

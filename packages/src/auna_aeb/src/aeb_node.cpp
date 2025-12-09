#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class AebNode : public rclcpp::Node
{
public:
  AebNode()
  : Node("aeb_node"),
    velocity_x_(0.0),
    ttc_threshold_(0.3),
    last_debug_time_(this->get_clock()->now())
  {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/robot1/scan",
      10,
      std::bind(&AebNode::laserCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot1/odom",
      10,
      std::bind(&AebNode::odomCallback, this, std::placeholders::_1));

    emergency_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/trigger_emergency_stop");

    RCLCPP_INFO(this->get_logger(), "AEB node initialized (correct TTC).");
  }

private:

  // --------------------
  //  ODOM CALLBACK
  // --------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    velocity_x_ = msg->twist.twist.linear.x;
  }

  // --------------------
  //  LASER CALLBACK
  // --------------------
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // AEB-Bereich: ±60° = Index 300 bis 780 (exakt berechnet)
    const size_t idx_start = 300;
    const size_t idx_end   = 780;

    double min_ttc = std::numeric_limits<double>::infinity();

    // Startwinkel passend zum Index
    double angle = msg->angle_min + idx_start * msg->angle_increment;

    // Debug placeholders
    double dbg_angle = 0.0;
    double dbg_range = 0.0;
    double dbg_rdot  = 0.0;

    for (size_t i = idx_start; i <= idx_end; i++)
    {
      double r = msg->ranges[i];

      // Abstand unbrauchbar? -> überspringen
      if (std::isnan(r) || r < msg->range_min || r > msg->range_max)
      {
        angle += msg->angle_increment;
        continue;
      }

      // Range-Rate: r_dot = -v * cos(theta)
      double r_dot = -velocity_x_ * std::cos(angle);

      // Closing speed (positiv bei Annäherung)
      double closing_speed = std::max(-r_dot, 0.0);

      // TTC
      double ttc = (closing_speed > 0.0)
                     ? r / closing_speed
                     : std::numeric_limits<double>::infinity();

      // Minimum TTC tracken
      if (ttc < min_ttc)
      {
        min_ttc = ttc;
        dbg_angle = angle;
        dbg_range = r;
        dbg_rdot  = r_dot;
      }

      angle += msg->angle_increment;
    }

    // Debug-Ausgabe (0.5 Hz)
    rclcpp::Time now = this->get_clock()->now();
    if ((now - last_debug_time_).seconds() > 0.5)
    {
      RCLCPP_INFO(this->get_logger(),
        "minTTC=%.3f  angle=%.3f rad (%.1f deg)  r=%.2f m  r_dot=%.3f m/s  vx=%.2f",
        min_ttc,
        dbg_angle,
        dbg_angle * 180.0 / M_PI,
        dbg_range,
        dbg_rdot,
        velocity_x_);

      last_debug_time_ = now;
    }

    // An AEB-Logik übergeben
    evaluateEmergencyBrake(min_ttc, velocity_x_, dbg_angle, dbg_range, dbg_rdot);
  }

  // --------------------
  //  AEB ACTIVATION LOGIC
  // --------------------1
  void evaluateEmergencyBrake(
      double min_ttc,
      double velocity_x,
      double angle,
      double r,
      double r_dot)
  {
    if (!emergency_client_->wait_for_service(50ms))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Service /trigger_emergency_stop not available");
      return;
    }

    if (min_ttc < ttc_threshold_ && velocity_x > 0.0)
    {
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = true;
      emergency_client_->async_send_request(req);

      RCLCPP_ERROR(this->get_logger(),
        "AEB ACTIVATED – TTC=%.3f  angle=%.3f rad (%.1f deg)  r=%.2f m  r_dot=%.3f m/s  vx=%.2f",
        min_ttc,
        angle,
        angle * 180.0 / M_PI,
        r,
        r_dot,
        velocity_x);
    }
  }

  // --------------------
  //  MEMBER VARIABLES
  // --------------------
  double velocity_x_;
  double ttc_threshold_;
  rclcpp::Time last_debug_time_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
};


// --------------------
//  MAIN
// --------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AebNode>());
  rclcpp::shutdown();
  return 0;
}
/// \file drag_race.cpp
/// \brief Commands the car to drive in a circle
///
/// PARAMETERS:
///     rate (int): the publishing frequency (Hz)
///     angular_velocity (double): the angular velocity of the robot (m)
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): velocity commands for the robot

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class Drag_Race : public rclcpp::Node
{
public:
  Drag_Race()
  : Node("drag_race")
  {
    // Parameters and default values
    declare_parameter("rate", 100);
    declare_parameter("cmd_max", 2000);
    declare_parameter("race_time", 5.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    cmd_max = get_parameter("cmd_max").as_int();
    race_time = get_parameter("race_time").as_double();

    // Other variables
    time = race_time + 1.;
    speed = 0.;
    race_on = false;

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Drag_Race::odom_callback, this, std::placeholders::_1));

    // Servers
    start_race_srv = create_service<std_srvs::srv::Empty>(
      "start_race",
      std::bind(&Drag_Race::start_race_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Drag_Race::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double angular_velocity, race_time;
  int loop_rate, cmd_max;
  int state = 1;
  double time, speed;
  bool race_on;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_race_srv;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    std_msgs::msg::Int32 drive_cmd_msg;

    if (time < race_time) {
      race_on = true;

      // cmd_vel and servo commands
      cmd_vel_msg.angular.z = 0.0;
      drive_cmd_msg.data = cmd_max;
        
      // Publish commands
      cmd_vel_pub->publish(cmd_vel_msg);
      drive_cmd_pub->publish(drive_cmd_msg);
      
      // update time
      time +=  1. / loop_rate;
      RCLCPP_INFO(this->get_logger(), "SPEED: %f", speed);
    } else {
      // send neutral cmd_vel and servo commands
      cmd_vel_msg.angular.z = 0.0;
      drive_cmd_msg.data = 1500;
    }
    
    // Publish commands
    cmd_vel_pub->publish(cmd_vel_msg);
    drive_cmd_pub->publish(drive_cmd_msg);
    speed = 0.;
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    speed = msg.twist.twist.linear.x;
  }

  /// \brief Callback for the enable drive server, enables/disables motors
  void start_race_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    time = 0.0;
    RCLCPP_INFO(this->get_logger(), "Start Race!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drag_Race>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/// \file steering_control.cpp
/// \brief Controls the car to drive and steer at a specified velocity
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
///     cmd_max (int): the maximum command sent to the servo
///     cmd_min (int): the minimum command sent to the servo
///     use_wheel_speed (bool): if the wheel encoder should be used for robot speed feedback
///     wheel_diameter (double): the publishing rate of the main loop (Hz)
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): the robot velocity commands
///     wheel_speed (std_msgs::msg::Float64): the speed of the rear wheels
/// PUBLISHES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class Steering_Control : public rclcpp::Node
{
public:
  Steering_Control()
  : Node("steering_control")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 100.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("use_wheel_speed", false);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("P", 5.0);
    declare_parameter("I", 0.0);
    declare_parameter("D", 0.0);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    use_wheel_speed = get_parameter("use_wheel_speed").as_bool();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    P = get_parameter("P").as_double();
    I = get_parameter("I").as_double();
    D = get_parameter("D").as_double();

    cmd_neutral = (cmd_min + cmd_max) / 2;
    angular_vel = 0.;
    angular_vel_cmd = 0.;
    angular_error = 0.;
    angular_error_prev = 0.;
    angular_error_cum = 0.;
    angular_error_der = 0.;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10, std::bind(&Steering_Control::cmd_vel_callback, this, std::placeholders::_1));
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Steering_Control::odom_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Steering_Control::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, cmd_max, cmd_min, cmd_neutral, steer_cmd;
  bool use_wheel_speed;
  double wheel_diameter;
  double P, I, D;
  double angular_vel, angular_vel_cmd;
  double angular_error, angular_error_prev, angular_error_cum, angular_error_der;
  rclcpp::Time now;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, loop for both wheels
  void timer_callback()
  {
    // Initialize messages
    std_msgs::msg::Int32 steer_msg;

    // Calculate error, integral/cumulative error, derivative of error
    angular_error = angular_vel_cmd - angular_vel;
    angular_error_cum += angular_error * (1.0 / loop_rate);
    angular_error_der = (angular_error - angular_error_prev) / (1.0 / loop_rate);

    // Calculate steering command with PID
    steer_cmd = (P * angular_error) + (I * angular_error_cum) + (D * angular_error_der);

    // Limit servo commands and add to message
    steer_msg.data = limit_cmd(steer_cmd);

    // Store errors as previous errors
    angular_error_prev = angular_error;
    
    // Publish command messages
    steering_cmd_pub->publish(steer_msg);
  }

  /// \brief The cmd_vel callback function, extracts angular velocity command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    angular_vel_cmd = msg.angular.z;
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    angular_vel = msg.twist.twist.angular.z;
  }

  /// \brief Limits the range of a cmd to [-motor_cmd_max, motor_cmd_max]
  int limit_cmd(int cmd)
  {
    if (cmd > cmd_max) {
      cmd = cmd_max;
    } else if (cmd < cmd_min) {
      cmd = cmd_min;
    }
    return cmd;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Steering_Control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

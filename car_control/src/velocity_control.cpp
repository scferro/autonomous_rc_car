/// \file velocity_control.cpp
/// \brief Controls the car to drive and steer at a specified velocity
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
///     cmd_max (int): the maximum command sent to the servo
///     cmd_min (int): the minimum command sent to the servo
///     wheel_diameter (double): the wheel diameter of the robot
///     Kp_steer (double): Kp for steering control
///     Ki_steer (double): Ki for steering control
///     Kd_steer (double): Kd for steering control
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): the robot velocity commands
///     odom (nav_msgs::msg::Odometry): odometry
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

class Velocity_Control : public rclcpp::Node
{
public:
  Velocity_Control()
  : Node("velocity_control")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 100.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("wheel_diameter", 0.108);
<<<<<<< HEAD
    declare_parameter("Kp_steer", 40.0);
    declare_parameter("Ki_steer", 1.0);
=======
    declare_parameter("Kp_steer", 1000.0);
    declare_parameter("Ki_steer", 10.0);
>>>>>>> refs/remotes/origin/main
    declare_parameter("Kd_steer", 0.1);
    declare_parameter("Kp_drive", 5.0);
    declare_parameter("Ki_drive", 0.0);
    declare_parameter("Kd_drive", 0.0);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    Kp_steer = get_parameter("Kp_steer").as_double();
    Ki_steer = get_parameter("Ki_steer").as_double();
    Kd_steer = get_parameter("Kd_steer").as_double();
    Kp_drive = get_parameter("Kp_drive").as_double();
    Ki_drive = get_parameter("Ki_drive").as_double();
    Kd_drive = get_parameter("Kd_drive").as_double();

    cmd_neutral = (cmd_min + cmd_max) / 2;
    angular_vel = 0.;
    angular_vel_cmd = 0.;
    angular_error = 0.;
    angular_error_prev = 0.;
    angular_error_cum = 0.;
    angular_error_der = 0.;
    linear_vel = 0.;
    linear_vel_cmd = 0.;
    linear_error = 0.;
    linear_error_prev = 0.;
    linear_error_cum = 0.;
    linear_error_der = 0.;
    steer_cmd = 1500;
    drive_cmd = 1500;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10, std::bind(&Velocity_Control::cmd_vel_callback, this, std::placeholders::_1));
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Velocity_Control::odom_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Velocity_Control::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int cmd_max, cmd_min, cmd_neutral, steer_cmd, drive_cmd;
  double loop_rate, wheel_diameter;
  double Kp_steer, Ki_steer, Kd_steer;
  double Kp_drive, Ki_drive, Kd_drive;
  double angular_vel, angular_vel_cmd, linear_vel, linear_vel_cmd;
  double angular_error, angular_error_prev, angular_error_cum, angular_error_der;
  double linear_error, linear_error_prev, linear_error_cum, linear_error_der;
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
    std_msgs::msg::Int32 drive_msg;

    // Calculate error, integral/cumulative error, derivative of error
    angular_error = angular_vel - angular_vel_cmd;
    angular_error_cum += angular_error * (1.0 / loop_rate);
    angular_error_der = (angular_error - angular_error_prev) / (1.0 / loop_rate);
    linear_error = linear_vel_cmd - linear_vel;
    linear_error_cum += linear_error * (1.0 / loop_rate);
    linear_error_der = (linear_error - linear_error_prev) / (1.0 / loop_rate);

    // Calculate steering command with PID
    steer_cmd = (Kp_steer * angular_error) + (Ki_steer * angular_error_cum) + (Kd_steer * angular_error_der) + 1500;
    drive_cmd = (Kp_drive * linear_error) + (Ki_drive * linear_error_cum) + (Kd_drive * linear_error_der) + 1500;

    // Limit servo commands and add to message
    steer_msg.data = limit_cmd(steer_cmd);
    drive_msg.data = limit_cmd(drive_cmd);

    // Store errors as previous errors
    angular_error_prev = angular_error;
    linear_error_prev = linear_error;
    
    // Publish command messages
    steering_cmd_pub->publish(steer_msg);
    drive_cmd_pub->publish(drive_msg);

    // RCLCPP_INFO(this->get_logger(), "steer_cmd: %i", steer_cmd);
    // RCLCPP_INFO(this->get_logger(), "angular_error: %f", angular_error);
    // RCLCPP_INFO(this->get_logger(), "drive_cmd: %i", drive_cmd);
    // RCLCPP_INFO(this->get_logger(), "linear_error: %f", linear_error);
  }

  /// \brief The cmd_vel callback function, extracts angular velocity command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    angular_vel_cmd = msg.angular.z;
    linear_vel_cmd = msg.linear.x;
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    angular_vel = msg.twist.twist.angular.z;
    linear_vel = msg.twist.twist.linear.x;
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
  auto node = std::make_shared<Velocity_Control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

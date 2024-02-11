/// \file drive_and_steer.cpp
/// \brief Controls the car to drive and steer at a specified velocity
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): the robot velocity commands
/// PUBLISHES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

// Used ChatGPT for debugging
// Refer to Citation [5] ChatGPT

using namespace std::chrono_literals;

class Drive_and_Steer : public rclcpp::Node
{
public:
  Drive_and_Steer()
  : Node("drive_and_steer")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 100.);
    declare_parameter("cmd_max", 180);
    declare_parameter("cmd_min", 0);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();

    // Define other variables
    P_l = 0.0;
    I_l = 0.0;
    D_l = 0.0;
    P_a = 0.0;
    I_a = 0.0;
    D_a = 0.0;
    cmd_neutral = (cmd_min + cmd_max) / 2;
    linear_vel = 0.;
    angular_vel = 0.;
    linear_error = 0.;
    linear_error_prev = 0.;
    linear_error_cum = 0.;
    linear_error_der = 0.;
    angular_error = 0.;
    angular_error_prev = 0.;
    angular_error_cum = 0.;
    angular_error_der = 0.;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10, std::bind(&Drive_and_Steer::cmd_vel_callback, this, std::placeholders::_1))

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Drive_and_Steer::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, cmd_max, cmd_min, cmd_neutral;
  int drive_cmd, steer_cmd;
  double wheel_radius;
  double P_l, I_l, D_l, P_a, I_a, D_a;
  double linear_vel, angular_vel, linear_vel_cmd, angular_vel_cmd;
  double linear_error, linear_error_prev, linear_error_cum, linear_error_der;
  double angular_error, angular_error_prev, angular_error_cum, angular_error_der;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, loop for both wheels
  void timer_callback()
  {
    // Initialize messages
    std_msgs::msg::Int32 drive_msg, steer_msg;

    // Calculate error
    linear_error = linear_vel_cmd - linear_vel;
    angular_error = angular_vel_cmd - angular_vel;
    
    // Calculate cumulative error
    linear_error_cum += linear_error * (1.0 / loop_rate);
    angular_error_cum += angular_error * (1.0 / loop_rate);
    
    // Calculate derivative of error
    linear_error_der = (linear_error - linear_error_prev) / (1.0 / loop_rate);
    angular_error_der = (angular_error - angular_error_prev) / (1.0 / loop_rate);

    // Calculate commands
    drive_cmd = (P_l * linear_error) + (I_l * linear_error_cum) + (D_l * linear_error_der);
    steer_cmd = (P_a * angular_error) + (I_a * angular_error_cum) + (D_a * angular_error_der);

    // Limiting servo commands
    drive_msg.data = limit_cmd(drive_cmd);
    steer_msg.data = limit_cmd(steer_cmd);
    
    // Publish command messages
    drive_cmd_pub->publish(drive_msg);
    steering_cmd_pub->publish(steer_msg);

    // Set previous errors
    linear_error_prev = linear_error;
    angular_error_prev = angular_error;
  }

  /// \brief The cmd_vel callback function, publishes motor speed commands based on received twist command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    linear_vel_cmd = msg.linear.x;
    angular_vel_cmd = msg.angular.z;
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
  auto node = std::make_shared<Drive_and_Steer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

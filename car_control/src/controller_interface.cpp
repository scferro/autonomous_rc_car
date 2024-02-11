/// \file controller_interface.cpp
/// \brief Allows controlling the robot with a game controller
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
/// SUBSCRIBES:
///     joy (sensor_msgs::msg::Joy): the controller inputs
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
    declare_parameter("loop_rate", 50.);
    declare_parameter("cmd_max", 180);
    declare_parameter("cmd_min", 0);
    declare_parameter("gear_ratio", 5.);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();

    // Define other variables
    cmd_neutral = (cmd_min + cmd_max) / 2;
    linear_vel = 0.;
    fwd_input = 1.;
    rev_input = 1.;
    steer_input = 0.;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    joy_sub = create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10, std::bind(&Drive_and_Steer::joy_callback, this, std::placeholders::_1))
    wheel_speed_sub = create_subscription<std_msgs::msg::Float64>(
      "wheel_speed",
      10, std::bind(&Drive_and_Steer::wheel_speed_callback, this, std::placeholders::_1))

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
  double wheel_speed, gear_ratio, motor_speed;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_speed_sub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes servo commands for the drive and steering
  void timer_callback()
  {
    // Adding servo commands to message
    drive_msg.data = drive_cmd;
    steer_msg.data = steer_cmd;
    
    // Publish command messages
    drive_cmd_pub->publish(drive_msg);
    steering_cmd_pub->publish(steer_msg);

    RCLCPP_INFO(this->get_logger(), "REV COUNTER: %f RPM", (motor_speed * 60 / (2 * 3.1415926)));
  }

  /// \brief The joystick callback function, calculates servo commands from the controller inputs
  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {
    double fwd_input, rev_input, steer_input, drive_input;

    // Get drive and steeing inputs from controller
    fwd_input = msg.axes[5]
    rev_input = msg.axes[4]
    steer_input = msg.axes[0]

    // Only use reverse command if drive is not pressed
    if (fwd_input==1.0) {
        drive_input = rev_input - 1.0;
    } else {
        drive_input = fwd_input + 1.0;
    }

    // Publish commands
    drive_cmd = ((drive_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;
    steer_cmd = ((steer_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;

  }

  /// \brief The wheel_speed callback function, stores the current wheel speed reported by the encoder
  void wheel_speed_callback(const std_msgs::msg::Float64 & msg)
  {
    wheel_speed = msg.data;
    motor_speed = wheel_speed * gear_ratio;
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

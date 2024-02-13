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
/// CLIENTS:
///     mode (rc_car_interfaces::srv::SetMode): sets the current mode of the car based on button inputs

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rc_car_interfaces/srv/set_mode.hpp"

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
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("gear_ratio", 5.);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();

    // Define other variables
    cmd_neutral = (cmd_min + cmd_max) / 2;
    wheel_speed = 0.;
    motor_speed = 0.;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    joy_sub = create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10, std::bind(&Drive_and_Steer::joy_callback, this, std::placeholders::_1));
    wheel_speed_sub = create_subscription<std_msgs::msg::Float64>(
      "wheel_speed",
      10, std::bind(&Drive_and_Steer::wheel_speed_callback, this, std::placeholders::_1));

    // Clients
    //mode_cli = create_client<rc_car_interfaces::srv::SetMode>("set_mode");

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
  //rclcpp::Client<rc_car_interfaces::srv::SetMode>::SharedPtr mode_cli;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes servo commands for the drive and steering
  void timer_callback()
  {
    std_msgs::msg::Int32 drive_msg, steer_msg;

    // Adding servo commands to message
    drive_msg.data = drive_cmd;
    steer_msg.data = steer_cmd;
    
    // Publish command messages
    drive_cmd_pub->publish(drive_msg);
    steering_cmd_pub->publish(steer_msg);

    //RCLCPP_INFO(this->get_logger(), "REV COUNTER: %f RPM", (motor_speed * 60 / (2 * 3.1415926)));
  }

  /// \brief The joystick callback function, calculates servo commands from the controller inputs
  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {
    double fwd_input, rev_input, steer_input, drive_input;
    int mode_in = 0;
    auto mode_request = std::make_shared<rc_car_interfaces::srv::SetMode::Request>();

    // Get drive and steeing inputs from controller
    fwd_input = msg.axes[5];
    rev_input = msg.axes[2];
    steer_input = msg.axes[0];

    // Only use reverse command if drive is not pressed
    if (fwd_input==1.0 && rev_input!=1.0) {
        drive_input = rev_input - 1.0;
    } else {
        drive_input = -fwd_input + 1.0;
    }
    
    // Publish commands
    drive_cmd = ((drive_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;
    steer_cmd = ((steer_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;

    // Check if mode buttons are pressed
    if (msg.buttons[0]==1) {
      mode_request->data = 0;
      mode_in = 1;
    } else if (msg.buttons[1]==1) {
      mode_request->data = 1;
      mode_in = 1;
    } else if (msg.buttons[2]==1) {
      mode_request->data = 2;
      mode_in = 1;
    } else if (msg.buttons[3]==1) {
      mode_request->data = 3;
      mode_in = 1;
    }

    // // If mode button pressed, change mode
    // if (mode_in==1) {
    //   while (!mode_cli->wait_for_service(1s)) {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for set_mode service...");
    //   }
    //   auto result = mode_cli->async_send_request(mode_request);
    // }
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

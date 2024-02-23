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
/// SERVERS:
///     enable_controller (std_srvs::srv::SetBool): enables/disables sending servo commands from the controller interface node
/// CLIENTS:
///     enable_drive (std_srvs::srv::SetBool): enables/disbales drive

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class Controller_Interface : public rclcpp::Node
{
public:
  Controller_Interface()
  : Node("controller_interface")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 50.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("enable_controller", true);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    enable_controller = get_parameter("enable_controller").as_bool();

    // Define other variables
    cmd_neutral = (cmd_min + cmd_max) / 2;
    drive_cmd = cmd_neutral;
    steer_cmd = cmd_neutral;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    joy_sub = create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10, std::bind(&Controller_Interface::joy_callback, this, std::placeholders::_1));

    // Servers
    enable_controller_srv = create_service<std_srvs::srv::SetBool>(
      "enable_controller",
      std::bind(&Controller_Interface::enable_controller_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Clients
    enable_drive_cli = create_client<std_srvs::srv::SetBool>("enable_drive");

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Controller_Interface::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, cmd_max, cmd_min, cmd_neutral;
  int drive_cmd, steer_cmd;
  bool enable_controller;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_drive_cli;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_controller_srv;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes servo commands for the drive and steering
  void timer_callback()
  {
    std_msgs::msg::Int32 drive_msg, steer_msg;

    // Adding servo commands to message
    drive_msg.data = drive_cmd;
    steer_msg.data = steer_cmd;
    
    // Publish command messages if enable_controller is set to true
    if (enable_controller==true){
      drive_cmd_pub->publish(drive_msg);
      steering_cmd_pub->publish(steer_msg);
    }

    //RCLCPP_INFO(this->get_logger(), "REV COUNTER: %f RPM", (motor_speed * 60 / (2 * 3.1415926)));
  }

  /// \brief The joystick callback function, calculates servo commands from the controller inputs
  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {
    double fwd_input, rev_input, steer_input, drive_input;
    bool enable_drive_cmd;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

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
    
    // Scale and store command values
    drive_cmd = ((drive_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;
    steer_cmd = ((steer_input + 2.0) / 4.0) * (cmd_max - cmd_min) + cmd_min;

    // Check if enable_drive buttons are pressed
    if (msg.buttons[0]==1) {
      request->data = true;
      enable_drive_cmd = true;
      RCLCPP_INFO(this->get_logger(), "Enabling drive motor.");
    } else if (msg.buttons[1]==1) {
      request->data = false;
      enable_drive_cmd = true;
      RCLCPP_INFO(this->get_logger(), "Disabling drive motor.");
    }

    // If mode button pressed, change disable drive
    if (enable_drive_cmd==true) {
      int count = 0;
      while ((!enable_drive_cli->wait_for_service(1s)) && (count < 5)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for enable_drive service...");
        count++;
      }
      auto result = enable_drive_cli->async_send_request(request);
      enable_drive_cmd = false;
    }

    // Check if enable_controller buttons are pressed
    if (msg.buttons[5]==1) {
      RCLCPP_INFO(this->get_logger(), "Enabling controller.");
      enable_controller = true;
    } else if (msg.buttons[4]==1) {
      RCLCPP_INFO(this->get_logger(), "Disabling controller.");
      enable_controller = false;
    }
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

  /// \brief Callback for the enable drive server, enables/disables motors
  void enable_controller_callback(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr)
  {
    if (request->data==true) {
      RCLCPP_INFO(this->get_logger(), "Enabling controller.");
      enable_controller = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Disabling controller.");
      enable_controller = false;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller_Interface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

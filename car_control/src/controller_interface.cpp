/// \file controller_interface.cpp
/// \brief Allows controlling the robot with a game controller
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
///     cmd_max (int): the maximum command from servo driver
///     cmd_min (int): the minimum command from servo driver
///     enable_controller (bool): enables publishing drive_cmd and steering_cmd from the controller node
/// SUBSCRIBES:
///     joy (sensor_msgs::msg::Joy): the controller inputs
///     wheel_speed (std_msgs::msg::Float64): the speed of the rear wheels
/// PUBLISHES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC
/// SERVERS:
///     enable_controller (std_srvs::srv::SetBool): enables/disables sending servo commands from the controller interface node
/// CLIENTS:
///     enable_drive (std_srvs::srv::SetBool): enables/disables drive
///     reset_imu (std_srvs::srv::Empty): resets IMU data
///     start_race (std_srvs::srv::Empty): starts a race - the car will launch and do laps or a drag race, depending on nodes running

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/srv/save_map.hpp"

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
    declare_parameter("enable_controller", false);
    declare_parameter("slow_mode", true);
    declare_parameter("cmd_max_slow", 1550);
    declare_parameter("cmd_min_slow", 1450);
    declare_parameter("max_accel", 200);
    declare_parameter("map_filename", "maps/map");
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    enable_controller = get_parameter("enable_controller").as_bool();
    slow_mode = get_parameter("slow_mode").as_bool();
    cmd_max_slow = get_parameter("cmd_max_slow").as_int();
    cmd_min_slow = get_parameter("cmd_min_slow").as_int();
    max_accel = get_parameter("max_accel").as_int();
    map_filename = get_parameter("map_filename").as_string();

    // Define other variables
    cmd_neutral = (cmd_min + cmd_max) / 2;
    drive_cmd = cmd_neutral;
    steer_cmd = cmd_neutral;
    drive_cmd_prev = cmd_neutral;
    first_msg = true;
    max_increase = max_accel / loop_rate;

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
    odom_reset_cli = create_client<std_srvs::srv::Empty>("odom_reset");
    start_race_cli = create_client<std_srvs::srv::Empty>("start_race");
    save_map_cli = create_client<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map");

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Controller_Interface::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, cmd_max, cmd_min, cmd_neutral, max_accel, max_increase;
  double drive_cmd, steer_cmd, drive_cmd_prev, cmd_max_slow, cmd_min_slow;
  bool enable_controller;
  sensor_msgs::msg::Joy msg_prev;
  bool first_msg, slow_mode;
  std::string map_filename;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_drive_cli;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr odom_reset_cli;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_race_cli;
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_cli;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_controller_srv;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes servo commands for the drive and steering
  void timer_callback()
  {
    std_msgs::msg::Int32 drive_msg, steer_msg;

    // If slow mode is enabled, limit commands
    if (slow_mode) {
      slow_mode_cmd();
    }

    // Adding servo commands to message
    drive_msg.data = drive_cmd;
    steer_msg.data = steer_cmd;
    
    // Publish command messages if enable_controller is set to true
    if (enable_controller==true){
      drive_cmd_pub->publish(drive_msg);
      steering_cmd_pub->publish(steer_msg);
    }
  }

  /// \brief The main timer callback, publishes servo commands for the drive and steering
  void slow_mode_cmd()
  {
    // Limit commands to the slow cmd range
    if (drive_cmd > cmd_max_slow) {
      drive_cmd = cmd_max_slow;
    } else if (drive_cmd < cmd_min_slow) {
      drive_cmd = cmd_min_slow;
    }

    // Limit acceleration rate 
    if ((drive_cmd > cmd_neutral) && (drive_cmd > (drive_cmd_prev + max_increase))) {
      drive_cmd = drive_cmd_prev + max_increase; 
    } else if ((drive_cmd < cmd_neutral) && (drive_cmd < (drive_cmd_prev - max_increase))) {
      drive_cmd = drive_cmd_prev - max_increase; 
    }

    // Store drive_cmd as drive_cmd_prev
    drive_cmd_prev = drive_cmd;
  }  

  /// \brief The joystick callback function, calculates servo commands from the controller inputs
  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {
    double fwd_input, rev_input, steer_input, drive_input;
    bool enable_drive_cmd;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto request_empty = std::make_shared<std_srvs::srv::Empty::Request>();
    auto request_map = std::make_shared<slam_toolbox::srv::SaveMap::Request>();

    if (first_msg) {
      msg_prev = msg;
      first_msg = false;
    }

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
    if (msg.buttons[0]==1 && msg_prev.buttons[0]!=1) {
      request->data = true;
      enable_drive_cmd = true;
    } else if (msg.buttons[1]==1 && msg_prev.buttons[1]!=1) {
      request->data = false;
      enable_drive_cmd = true;
    }

    // If enable driver button pressed, enable drive
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
    if (msg.buttons[5]==1 && msg_prev.buttons[5]!=1) {
      RCLCPP_INFO(this->get_logger(), "Controller enabled.");
      enable_controller = true;
    } else if (msg.buttons[4]==1 && msg_prev.buttons[4]!=1) {
      RCLCPP_INFO(this->get_logger(), "Controller disabled.");
      enable_controller = false;
    }

    // If reset_imu button is pressed, call reset IMU service
    if (msg.buttons[3]==1 && msg_prev.buttons[3]!=1) {
      int count = 0;
      while ((!odom_reset_cli->wait_for_service(1s)) && (count < 5)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for reset_imu service...");
        count++;
      }
      auto result = odom_reset_cli->async_send_request(request_empty);
    }

    // If save_map button is pressed, call reset IMU service
    if (msg.buttons[7]==1 && msg_prev.buttons[7]!=1) {
      int count = 0;
      // Add filename to string message and send
      std_msgs::msg::String string_msg;
      string_msg.data = map_filename;
      request_map->name = string_msg;
      while ((!save_map_cli->wait_for_service(1s)) && (count < 5)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for save_map service...");
        count++;
      }
      auto result = save_map_cli->async_send_request(request_map);
    }

    // If start_race button is pressed, call start_race service
    if (msg.buttons[2]==1 && msg_prev.buttons[2]!=1) {
      int count = 0;
      while ((!start_race_cli->wait_for_service(1s)) && (count < 5)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for start_race service...");
        count++;
      }
      auto result = start_race_cli->async_send_request(request_empty);
    }
    // Save msg as msg_prev
    msg_prev = msg;
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

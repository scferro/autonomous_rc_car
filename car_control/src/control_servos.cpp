/// \file control_servos.cpp
/// \brief Controls the steering servo and ESC via a PCA 9685 over I2C
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages
///     cmd_max (int): the maximum command from servo driver
///     cmd_min (int): the minimum command from servo driver
///     timeout (double): minimum time required between receiving commands
///     steer_cmd_center (int): steering command for centered steering
///     steer_cmd_range (int): steering command range. lock to lock
///     enable_drive (bool): allow publishing drive commands
///     use_traction_control (bool): enables traction control
///     simulate (bool): publish commands for isaacsim simulation
///     drive_pin (int): the pin used for the drive motor on the servo driver
///     steer_pin (int): the pin used for the steering servo on the servo driver
///     wheel_diameter (double): the diameter of the robot wheels
///     gear_ratio (double): the gear ratio of the robot
///     max_rpm (double): max motor RPM - KV * V
///     steer_angle_range (double): steering angle range, used for simulation
///     max_decel_multiplier (double): multiplier for simulated deceleration
///     max_decel_offset (double): offset for simulated deceleration
///     target_speed_multiplier (double): multiplier for traction control target speed
///     target_speed_offset (double): offset for traction control target speed
///     Kp (double): Kp for traction control
///     Ki (double): Ki for traction control
///     Kd (double): Kd for traction control
/// SUBSCRIBES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC
///     odom (nav_msgs::msg::Odometry): the odometry from the IMU
///     odom_encoder (nav_msgs::msg::Odometry): the odometry from the encoder
/// PUBLISHES:
///     ackermann_cmd (ackermann_msgs::msg::AckermannDriveStamped): ackermann commands published for simulation in Isaac Sim
/// SERVERS:
///     enable_drive (std_srvs::srv::SetBool): enables/disables drive motor

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// PCA9685 Registers
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

// I2C Address of the PCA9685 module
#define PCA9685_ADDRESS 0x40

// Function to initialize PCA9685
void initPCA9685(int fd) {
    // Reset the PCA9685
    char settings[2] = {MODE1, 0x00};
    write(fd, settings, 2);

    // Set the PWM frequency to 50 Hz
    float prescale_val = 25000000.0; // 25MHz
    prescale_val /= 4096.0;          // 12-bit
    prescale_val /= 50.0;            // 50Hz
    prescale_val -= 1.0;
    unsigned char prescale = static_cast<unsigned char>(prescale_val + 0.5);

    settings[0] = MODE1;
    settings[1] = 0x10; // Sleep
    write(fd, settings, 2);

    settings[0] = PRESCALE;
    settings[1] = prescale;
    write(fd, settings, 2);

    settings[0] = MODE1;
    settings[1] = 0x80; // Restart
    write(fd, settings, 2);

    settings[1] = 0x20; // Auto-Increment enabled
    write(fd, settings, 2);
}

void delayMicroseconds(int microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

// Function to set PWM value for a specific channel
void setPWM(int fd, int channel, int onValue, int offValue) {
    char data[5];
    data[0] = LED0_ON_L + 4 * channel;
    data[1] = onValue & 0xFF;
    data[2] = onValue >> 8;
    data[3] = offValue & 0xFF;
    data[4] = offValue >> 8;
    write(fd, data, 5);
}

int convert_microsec(int microsec) {
    double pwm = microsec / 4.88;
    return static_cast<int>(pwm);
}

class Control_Servos : public rclcpp::Node
{
public:
  Control_Servos()
  : Node("control_servos")
  {
    // Parameters and default values
    declare_parameter("rate", 200.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("timeout", 1.);
    declare_parameter("steer_cmd_center", 1500);
    declare_parameter("steer_cmd_range", 500);
    declare_parameter("enable_drive", true);
    declare_parameter("use_traction_control", false);
    declare_parameter("simulate", true);
    declare_parameter("drive_pin", 0);
    declare_parameter("steer_pin", 1);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("max_rpm", 16095.);
    declare_parameter("steer_angle_range", 0.8);
    declare_parameter("max_decel_multiplier", 1.0);
    declare_parameter("max_decel_offset", 0.1);
    declare_parameter("target_speed_multiplier", 1.1);
    declare_parameter("target_speed_offset", 1.0);
    declare_parameter("Kp", 50.0);
    declare_parameter("Ki", 0.);
    declare_parameter("Kd", 0.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    timeout = get_parameter("timeout").as_double();
    steer_cmd_center = get_parameter("steer_cmd_center").as_int();
    steer_cmd_range = get_parameter("steer_cmd_range").as_int();
    enable_drive = get_parameter("enable_drive").as_bool();
    drive_pin = get_parameter("drive_pin").as_int();
    steer_pin = get_parameter("steer_pin").as_int();
    use_traction_control = get_parameter("use_traction_control").as_bool();
    simulate = get_parameter("simulate").as_bool();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    gear_ratio = get_parameter("gear_ratio").as_double();
    max_rpm = get_parameter("max_rpm").as_double();
    steer_angle_range = get_parameter("steer_angle_range").as_double();
    max_decel_offset = get_parameter("max_decel_offset").as_double();
    max_decel_multiplier = get_parameter("max_decel_multiplier").as_double();
    target_speed_multiplier = get_parameter("target_speed_multiplier").as_double();
    target_speed_offset = get_parameter("target_speed_offset").as_double();
    Kp = get_parameter("Kp").as_double();
    Ki = get_parameter("Ki").as_double();
    Kd = get_parameter("Kd").as_double();

    // Define other variables
    drive_cmd_neutral = (cmd_max + cmd_min) / 2;
    steering_cmd = steer_cmd_center;
    drive_cmd = drive_cmd_neutral;
    now = this->get_clock()->now();
    time_now = now.seconds() + (now.nanoseconds() * 0.000000001);
    time_last_steer = time_now;
    time_last_drive = time_now;
    enable_drive = true;
    speed_last = 0.0;
    speed_encoder = 0.;
    speed = 0.;
    speed_error = 0.;
    speed_error_cum = 0.;
    speed_error_prev = 0.;
    speed_error_der = 0.;
    max_decel = 0.;
    speed_factor = (max_rpm * 2 * 3.1415926 * (wheel_diameter / 2) / (60 * gear_ratio)) / ((cmd_max - cmd_min) / 2);
    steering_factor = steer_angle_range / steer_cmd_range;

    // Subscribers
    steering_cmd_sub = create_subscription<std_msgs::msg::Int32>(
      "steering_cmd",
      10, std::bind(&Control_Servos::steering_cmd_callback, this, std::placeholders::_1));
    drive_cmd_sub = create_subscription<std_msgs::msg::Int32>(
      "drive_cmd",
      10, std::bind(&Control_Servos::drive_cmd_callback, this, std::placeholders::_1));
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Control_Servos::odom_callback, this, std::placeholders::_1));
    odom_encoder_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom_encoder",
      10, std::bind(&Control_Servos::odom_encoder_callback, this, std::placeholders::_1));

    // Publishers
    ackermann_cmd_pub = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);

    // Servers
    enable_drive_srv = create_service<std_srvs::srv::SetBool>(
      "enable_drive",
      std::bind(&Control_Servos::enable_drive_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Control_Servos::timer_callback, this));

    // Motor startup
    device = "/dev/i2c-7"; // Change to your I2C bus
    fd = open(device, O_RDWR);

    if (fd < 0) {
        RCLCPP_INFO(this->get_logger(), "Failed to open the i2c bus");
    }
    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        close(fd);
    }

    initPCA9685(fd);

    // Run startup then set motor commands to neutral
    if (simulate==false) {
      setPWM(fd, 0, 0, convert_microsec(drive_cmd_neutral)); // 307 corresponds to approximately 1.5ms pulse width at 50Hz
      setPWM(fd, 1, 0, convert_microsec(steer_cmd_center)); // 307 corresponds to approximately 1.5ms pulse width at 50H
    }
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, timeout;
  int cmd_max, cmd_min, steering_cmd, drive_cmd, drive_cmd_neutral;
  double time_now, time_last_steer, time_last_drive;
  int steer_cmd_center, steer_cmd_range;
  rclcpp::Time now;
  const char *device;
  int fd, pwm;
  bool enable_drive, use_traction_control, simulate;
  int drive_pin, steer_pin;
  double Kp, Ki, Kd;
  double wheel_diameter, gear_ratio, max_rpm;
  double speed_last, steer_angle_range, max_decel, max_decel_multiplier, max_decel_offset;
  double speed_encoder, speed;
  double speed_error, speed_error_cum, speed_error_prev, speed_error_der;
  double target_speed_offset, target_speed_multiplier;
  double speed_factor, steering_factor;

  // Initialize subscriptions and timer
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_cmd_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_encoder_sub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_drive_srv;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_cmd_pub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    if (use_traction_control==true) {
      traction_control();
    }

    // Check if steering and drive commands received within before timeout
    // now = this->get_clock()->now();
    // time_now = now.seconds() + (now.nanoseconds() * 0.000000001);
    // if (((time_now - time_last_steer) > timeout) || ((time_now - time_last_drive) > timeout)) {
    //   RCLCPP_DEBUG(this->get_logger(), "Either no steering or no drive command received within last %f seconds. Timeout.", timeout);
    //   steering_cmd = steer_cmd_center;
    //   drive_cmd = drive_cmd_neutral;
    // }

    // If in enable_drive is set to false, reset drive command to neutral
    if (enable_drive==false) {
      drive_cmd = drive_cmd_neutral;
    }

    // Write servo commands
    if (simulate==false) {
      setPWM(fd, 0, 0, convert_microsec(drive_cmd)); // 307 corresponds to approximately 1.5ms pulse width at 50Hz
      setPWM(fd, 1, 0, convert_microsec(steering_cmd)); // 307 corresponds to approximately 1.5ms pulse width at 50H
    } else {
      publish_ackermann();
    }
  }

  /// \brief Apply traction control to current steering command
  void traction_control()
  {
    double target_speed;

    // If car is being commanded forward
    if (drive_cmd >= drive_cmd_neutral) {
      // Calculate target speed
      target_speed = (speed * target_speed_multiplier) + target_speed_offset;

      // If wheel speed is below target speed, use wheel speed as target
      if (target_speed > speed_encoder) {
        target_speed = speed_encoder;
      }

      // Calculate speed error, integral, and derivative
      speed_error = speed_encoder - target_speed;
      speed_error_cum += (speed_error / loop_rate);
      speed_error_der = (speed_error - speed_error_prev) * loop_rate;

      // Use PID to calculate correction for drive command if wheels spinning
      if (speed_encoder > target_speed){
        drive_cmd += -((Kp * speed_error) + (Ki * speed_error_cum) + (Kd * speed_error_der));
        if (drive_cmd < drive_cmd_neutral) {drive_cmd = drive_cmd_neutral;}
        RCLCPP_INFO(this->get_logger(), "TCS: %i", drive_cmd);
      }
    }
  }

  /// \brief Create and publish an ackermann_msg for simulation in Isaac Sim
  void publish_ackermann()
  {
    ackermann_msgs::msg::AckermannDriveStamped ackermann_cmd;
    double speed_sim, steering_angle;
    
    // Convert command messages to robot commands using robot data
    speed_sim = speed_factor * (drive_cmd - drive_cmd_neutral);
    steering_angle = steering_factor * (steering_cmd - steer_cmd_center);

    max_decel = pow(speed_last, 2) * max_decel_multiplier + max_decel_offset;

    // RCLCPP_INFO(this->get_logger(), "max_decel: %f", max_decel);
    // RCLCPP_INFO(this->get_logger(), "speed1: %f", speed_sim);
    // RCLCPP_INFO(this->get_logger(), "speed_last_prev: %f", speed_last);

    // Adjust speed if slowing down 
    if ((speed_sim >= 0.) && (speed_last >= 0.)){
        if (speed_sim < (speed_last - (max_decel / loop_rate))) {
            speed_sim = speed_last - (max_decel / loop_rate);
        }
    } else if ((speed_sim <= 0.) and (speed_last <= 0.)) {
        if (speed_sim > (speed_last + (max_decel / loop_rate))) {
            speed_sim = speed_last + (max_decel / loop_rate);
        }
    }

    // Add values to message
    ackermann_cmd.drive.speed = speed_sim;
    ackermann_cmd.drive.steering_angle = steering_angle;

    // Add time and publish message
    ackermann_cmd.header.stamp = this->get_clock()->now();
    ackermann_cmd_pub->publish(ackermann_cmd);

    // Store speed as speed_last
    speed_last = speed_sim;
  }

  /// \brief The steering_cmd callback function, updates the steering command
  void steering_cmd_callback(const std_msgs::msg::Int32 & msg)
  {
    rclcpp::Time time;

    // Constrain command to valid range
    if (msg.data > (steer_cmd_center + (steer_cmd_range / 2))) {
      steering_cmd = (steer_cmd_center + (steer_cmd_range / 2));
    } else if (msg.data < (steer_cmd_center - (steer_cmd_range / 2))) {
      steering_cmd = (steer_cmd_center - (steer_cmd_range / 2));
    } else {
      steering_cmd = msg.data;
    }

    time = this->get_clock()->now();
    time_last_steer = time.seconds() + (time.nanoseconds() * 0.000000001);
  }

  /// \brief The drive_cmd callback function, updates the drive motor command
  void drive_cmd_callback(const std_msgs::msg::Int32 & msg)
  {
    rclcpp::Time time;

    // Constrain command to valid range
    if (msg.data > cmd_max) {
      drive_cmd = cmd_max;
    } else if (msg.data < cmd_min) {
      drive_cmd = cmd_min;
    } else {
      drive_cmd = msg.data;
    }

    // Get time
    time = this->get_clock()->now();
    time_last_drive = time.seconds() + (time.nanoseconds() * 0.000000001);
  }

  /// \brief Callback for the enable drive server, enables/disables motors
  void enable_drive_callback(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr)
  {
    if (request->data==true) {
      RCLCPP_INFO(this->get_logger(), "Enabling drive motor.");
      enable_drive = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Disabling drive motor.");
      enable_drive = false;
    }
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    speed = msg.twist.twist.linear.x;
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_encoder_callback(const nav_msgs::msg::Odometry & msg)
  {
    speed_encoder = msg.twist.twist.linear.x;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Control_Servos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

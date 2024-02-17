/// \file odometry.cpp
/// \brief Takes in data from the RealSense IMU and wheel encoders to provide the current robot twist
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
/// SUBSCRIBES:
///     camera/camera/gyro/sample (sensor_msgs::msg::Imu): the gyro data
///     camera/camera/accel/sample (sensor_msgs::msg::Imu): the accelerometer data
/// PUBLISHES:
///     steering_cmd (geometry_msgs::msg::Twist): the command for the steering servo

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class AS5048A {
public:
    AS5048A(const std::string& spiDevice, uint8_t spiMode, uint32_t spiSpeed, uint8_t spiBitsPerWord)
        : fd(-1), mode(spiMode), speed(spiSpeed), bits(spiBitsPerWord) {
        // Open SPI device
        fd = open(spiDevice.c_str(), O_RDWR);
        if (fd < 0) {
            //RCLCPP_INFO(this->get_logger(), "Can't open device.");
            return;
        }

        // SPI mode
        ioctl(fd, SPI_IOC_WR_MODE, &mode);
        ioctl(fd, SPI_IOC_RD_MODE, &mode);

        // Bits per word
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);

        // Max speed Hz
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    }

    ~AS5048A() {
        if (fd >= 0) {
            close(fd);
        }
    }

    uint16_t readAngle() {
        uint8_t tx[] = {0xFF, 0xFF}; // Dummy bytes to trigger clock
        uint8_t rx[2] = {0, 0};
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx,
            .rx_buf = (unsigned long)rx,
            .len = 2,
            .speed_hz = speed,
            .delay_usecs = 0,
            .bits_per_word = bits,
        };

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
            //RCLCPP_INFO(this->get_logger(), "Can't send spi message.");
            return 0;
        }

        // Combine the two bytes received into one 16-bit value and mask out the first two bits
        uint16_t angle = ((rx[0] << 8) | rx[1]) & 0x1FFF;

        return angle;
    }

private:
    int fd;
    uint8_t mode;
    uint32_t speed;
    uint8_t bits;
};

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 250.);
    declare_parameter("encoder_ticks", 8192);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("x_accel_offset", 0.029420);
    declare_parameter("y_accel_offset", -9.404577);
    declare_parameter("z_accel_offset", 0.264780);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    encoder_ticks = get_parameter("encoder_ticks").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    x_accel_offset = get_parameter("x_accel_offset").as_double();
    y_accel_offset = get_parameter("y_accel_offset").as_double();
    z_accel_offset = get_parameter("z_accel_offset").as_double();

    // Define other variables
    linear_vel = 0.0;
    linear_accel = 0.0;
    angular_vel = 0.0;
    angle_prev = 0.;
    raw_accel[0] = 0.0;
    raw_accel[1] = 0.0;
    raw_accel[2] = 0.0;
    raw_gyro[0] = 0.0;
    raw_gyro[1] = 0.0;
    raw_gyro[2] = 0.0;
    angles[0] = 0.0;
    angles[1] = 0.0;
    angles[2] = 0.0;
    accel_thresh = 0.05;

    // Define custom QoS profile
    rclcpp::QoS custom_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos_profile.best_effort();
    custom_qos_profile.durability_volatile();

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_encoder_pub = create_publisher<nav_msgs::msg::Odometry>("odom_encoder", 10);

    // Subscribers
    accel_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/accel/sample",
      custom_qos_profile, std::bind(&Odometry::accel_callback, this, std::placeholders::_1));
    gyro_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/gyro/sample",
      custom_qos_profile, std::bind(&Odometry::gyro_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    encoder_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Odometry::encoder_timer_callback, this));
    imu_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Odometry::imu_timer_callback, this));
  }

private:
  // Initialize parameter variables
  int _rate;
  double raw_accel[3], accel[3], raw_gyro[3];
  double angles[3];
  double linear_vel, linear_accel, angular_vel;
  double accel_thresh;
  int rate;
  double loop_rate, gear_ratio;
  int encoder_ticks;
  uint16_t angle, angle_prev;
  double delta, wheel_diameter;
  double x_accel_offset, y_accel_offset, z_accel_offset;

  // Initialize encoder object 
  AS5048A encoder = AS5048A("/dev/spidev0.0", SPI_MODE_1, 1000000, 8); // Adjust as necessary

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_encoder_pub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub;
  rclcpp::TimerBase::SharedPtr encoder_timer;
  rclcpp::TimerBase::SharedPtr imu_timer;

  /// \brief The main timer callback, loop for both wheels
  void imu_timer_callback()
  {
    nav_msgs::msg::Odometry odom_msg;

    // Calculate current gyro angles
    angles[0] += raw_gyro[0] / loop_rate;
    angles[1] += raw_gyro[1] / loop_rate;
    angles[2] += raw_gyro[2] / loop_rate;
    angular_vel = raw_gyro[1];

    // Correct accel values. If less than accel_thresh, set to 0
    accel[0] = raw_accel[0] - x_accel_offset;
    accel[1] = raw_accel[1] - y_accel_offset;
    accel[2] = raw_accel[2] - z_accel_offset;
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_thresh) {
        accel[i] = 0.0;
      }
    }

    RCLCPP_INFO(this->get_logger(), "angular_vel[0]: %f", raw_gyro[0]);
    RCLCPP_INFO(this->get_logger(), "angular_vel[1]: %f", raw_gyro[1]);
    RCLCPP_INFO(this->get_logger(), "angular_vel[2]: %f", raw_gyro[2]);

    RCLCPP_INFO(this->get_logger(), "accel[0]: %f", accel[0]);
    RCLCPP_INFO(this->get_logger(), "accel[1]: %f", accel[1]);
    RCLCPP_INFO(this->get_logger(), "accel[2]: %f", accel[2]);


    // Calculate current forward velocity based on gyro angles and accel
    linear_accel = ((-accel[0] * cos(angles[0]) * sin(angles[1])) + (accel[1] * sin(angles[0]) * cos(angles[1])) + (accel[2] * cos(angles[0]) * cos(angles[1])));

    // Calculate current linear velocity
    linear_vel += linear_accel / loop_rate;

    // Add velocities to twist message
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;
    odom_msg.pose.pose.orientation.x = angles[0];
    odom_msg.pose.pose.orientation.y = angles[1];
    odom_msg.pose.pose.orientation.z = angles[2];

    // Publish odom_msg
    odom_pub->publish(odom_msg);
  }

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void encoder_timer_callback()
  {
    nav_msgs::msg::Odometry odom_msg;
    double chassis_speed;

    // Read encoder anglker
    angle = encoder.readAngle();

    // Calculate change since last reading
    delta = angle - angle_prev;
    if (abs(delta) > encoder_ticks/2) {
      if (abs(delta)==delta) {
        delta = -(encoder_ticks - abs(delta));
      } else {
        delta = (encoder_ticks - abs(delta));
      }
    }

    // Calculate wheel speed
    chassis_speed = (delta / (encoder_ticks * gear_ratio)) * loop_rate * 6.283185307 * wheel_diameter / 2;

    // Publish wheel speed messages
    odom_msg.twist.twist.linear.x = chassis_speed;
    odom_encoder_pub->publish(odom_msg);

    // Set previous angle
    angle_prev = angle;
  }

  /// \brief The cmd_vel callback function, publishes motor speed commands based on received twist command
  void accel_callback(const sensor_msgs::msg::Imu & msg)
  {
    raw_accel[0] = msg.linear_acceleration.x;
    raw_accel[1] = msg.linear_acceleration.y;
    raw_accel[2] = msg.linear_acceleration.z;
    // RCLCPP_INFO(this->get_logger(), "linear_accel[0]: %f", raw_accel[0]);
    // RCLCPP_INFO(this->get_logger(), "linear_accel[1]: %f", raw_accel[1]);
    // RCLCPP_INFO(this->get_logger(), "linear_accel[2]: %f", raw_accel[2]);
  }

  /// \brief The wheel_speed callback function, calculates vehicle speed based on wheel speed if enabled
  void gyro_callback(const sensor_msgs::msg::Imu & msg)
  {
    raw_gyro[0] = msg.angular_velocity.x;
    raw_gyro[1] = msg.angular_velocity.y;
    raw_gyro[2] = msg.angular_velocity.z;
    // RCLCPP_INFO(this->get_logger(), "angular_vel[0]: %f", raw_gyro[0]);
    // RCLCPP_INFO(this->get_logger(), "angular_vel[1]: %f", raw_gyro[1]);
    // RCLCPP_INFO(this->get_logger(), "angular_vel[2]: %f", raw_gyro[2]);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Odometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

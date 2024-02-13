/// \file read_encoder.cpp
/// \brief Reads the encoder and publishes the wheel speed
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages (Hz)
///     encoder_ticks (int): the number of encoder ticks per motor revolution
///     rate (gear ratio): the gear ratio between the motor and wheels
/// PUBLISHES:
///     wheel_speed (std_msgs::msg::Float64): the speed of the rear wheels

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
#include <linux/spi/spidev.h>
#include <cstring>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class AS5048A {
public:
    AS5048A(const std::string& spiDevice, uint8_t spiMode, uint32_t spiSpeed, uint8_t spiBitsPerWord)
        : fd(-1), mode(spiMode), speed(spiSpeed), bits(spiBitsPerWord) {
        // Open SPI device
        fd = open(spiDevice.c_str(), O_RDWR);
        if (fd < 0) {
            std::cerr << "Can't open device." << std::endl;
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
            std::cerr << "Can't send spi message" << std::endl;
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

class Read_Encoder : public rclcpp::Node
{
public:
  Read_Encoder()
  : Node("read_encoder")
  {
    // Parameters and default values
    declare_parameter("rate", 200.);
    declare_parameter("encoder_ticks", 8192);
    declare_parameter("gear_ratio", 5.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    encoder_ticks = get_parameter("encoder_ticks").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();

    // Define other variables
    wheel_speed = 0.0;
    angle_prev = 0;

    // Publishers
    wheel_speed_pub = create_publisher<std_msgs::msg::Float64>("wheel_speed", 10);

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Read_Encoder::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, gear_ratio;
  int encoder_ticks;
  double wheel_speed;
  uint16_t angle, angle_prev;
  double delta;

  // Initialize encoder object 
  AS5048A encoder = AS5048A("/dev/spidev0.0", SPI_MODE_1, 1000000, 8); // Adjust as necessary

  // Start publisher and timer
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_speed_pub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    std_msgs::msg::Float64 wheel_speed_msg;

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
    wheel_speed = (delta / (encoder_ticks * gear_ratio)) * loop_rate * 6.283185307;

    // Publish wheel speed messages
    wheel_speed_msg.data = wheel_speed;
    wheel_speed_pub->publish(wheel_speed_msg);

    // Set previous angle
    angle_prev = angle;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Read_Encoder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

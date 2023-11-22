#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdint>
#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define BAUDRATE B115200
#define UART_DEVICE "/dev/ttyUSB0"

union DoubleToByte
{
  float value;
  unsigned char bytes[4];
};

unsigned char Sdata[255];
unsigned char Rdata[255];

class Uart : public rclcpp::Node
{
public:
  Uart() : Node("uart")
  {
    // 打开串口
    fd_ = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      return;
    }

    // 配置串口
    struct termios options;
    tcgetattr(fd_, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    tcsetattr(fd_, TCSANOW, &options);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&Uart::sendTwist, this));

    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_nav", 10, std::bind(&Uart::twist_callback, this, std::placeholders::_1));
  }

private:
  void sendTwist(){
    // int i = 0;
    // int bytes = read(fd_, Rdata, auto pub1 = serial::msg::Twist(); 10);
    // if (bytes == -1)
    // {
    //   RCLCPP_ERROR(get_logger(), "failed to read");
    // }
    // for (int i = 0; i < 10; i++)
    // {
    //   if (Rdata[i] == 0xAA)
    //   {
    //     break;
    //   }
    // }
    // if (Rdata[i] == 0xAA && Rdata[i + 9] == 0xBB)
    // {
    //   auto pub = geometry_msgs::msg::Twist();
    //   DoubleToByte conver;
    //   conver.bytes[0] = Rdata[1];
    //   conver.bytes[1] = Rdata[2];
    //   conver.bytes[2] = Rdata[3];
    //   conver.bytes[3] = Rdata[4];
    //   pub.linear.x = conver.value;
    //   conver.bytes[0] = Rdata[5];
    //   conver.bytes[1] = Rdata[6];
    //   conver.bytes[2] = Rdata[7];
    //   conver.bytes[3] = Rdata[8];
    //   pub.angular.z = conver.value;
    //   twist_publisher_->publish(pub);
    // }
  };
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist) const
  {
    DoubleToByte conver;
    // twist->linear.x = 1.1943;
    // twist->angular.z = 1.4567;
    Sdata[0] = 0x5a;
    conver.value = twist->linear.x;
    std::cout << "linear_x: " << conver.value << std::endl;
    Sdata[1] = conver.bytes[0];
    Sdata[2] = conver.bytes[1];
    Sdata[3] = conver.bytes[2];
    Sdata[4] = conver.bytes[3];
    conver.value = twist->angular.z;
    std::cout << "angular_z: " << conver.value << std::endl;
    Sdata[5] = conver.bytes[0];
    Sdata[6] = conver.bytes[1];
    Sdata[7] = conver.bytes[2];
    Sdata[8] = conver.bytes[3];
    Sdata[9] = 0xa5;

    // RCLCPP_INFO(get_logger(), "%x", Sdata[1]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[2]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[3]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[4]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[5]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[6]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[7]);
    // RCLCPP_INFO(get_logger(), "%x", Sdata[8]);
    RCLCPP_INFO(get_logger(), "twist is get");
    // 发送数据
    write(fd_, Sdata, 10);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  int fd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Uart>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

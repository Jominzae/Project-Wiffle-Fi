#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 

using namespace std::chrono_literals;

// [확인] 드라이버 장치 파일 경로
#define DEVICE_PATH "/dev/rssi_driver_table_test" 
#define BUFF_SIZE 4096 

class DriverBridgeNode : public rclcpp::Node
{
public:
  DriverBridgeNode()
  : Node("driver_bridge_node"), fd_(-1)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);
    
    // 장치 열기
    fd_ = open(DEVICE_PATH, O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "장치 열기 실패! (sudo chmod 666 %s 했나요?)", DEVICE_PATH);
    } else {
      RCLCPP_INFO(this->get_logger(), "장치 연결 성공: %s", DEVICE_PATH);
    }

    // 0.5초마다 데이터 읽기
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DriverBridgeNode::read_and_publish, this));
  }

  ~DriverBridgeNode()
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  void read_and_publish()
  {
    if (fd_ < 0) return;

    // [핵심] 파일 읽기 위치를 맨 처음으로 초기화 (이게 없으면 한 번 읽고 멈춤)
    lseek(fd_, 0, SEEK_SET);

    char buffer[BUFF_SIZE];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';
      auto message = std_msgs::msg::String();
      message.data = std::string(buffer);
      
      // 로그 출력 (잘 가고 있는지 확인용)
      RCLCPP_INFO(this->get_logger(), "데이터 송신 중... (%ld bytes)", bytes_read);
      publisher_->publish(message);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int fd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

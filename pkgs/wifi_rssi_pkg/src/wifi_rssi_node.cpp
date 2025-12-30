#include <chrono>
#include <string>
#include <array>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/wifi_raw.hpp"

using namespace std::chrono_literals;

static constexpr const char* SERIAL_PORT = "/dev/serial0";
static constexpr int SERIAL_BAUD = 115200;

static constexpr const char* TOPIC_WIFI_RAW = "/wifi/raw";

static constexpr const char* DRIVER_PATH = "/dev/rssi_driver_table_test";

static constexpr int TIMER_PERIOD_MS = 20;
static constexpr size_t READ_CHUNK = 4096;
static constexpr size_t MAX_BUFFER_BYTES = 300000;

static speed_t to_speed_t(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

class WifiRssiNode : public rclcpp::Node
{
public:
  WifiRssiNode()
  : Node("wifi_rssi_node"), serial_fd_(-1), drv_fd_(-1)
  {
    pub_ = create_publisher<custom_interfaces::msg::WifiRaw>(TOPIC_WIFI_RAW, 10);

    open_serial();
    open_driver();  // 실패해도 토픽 publish는 계속 됨

    timer_ = create_wall_timer(
      std::chrono::milliseconds(TIMER_PERIOD_MS),
      std::bind(&WifiRssiNode::tick, this)
    );

    RCLCPP_INFO(get_logger(),
      "wifi_rssi_node started. port=%s baud=%d topic=%s driver=%s (tee ON)",
      SERIAL_PORT, SERIAL_BAUD, TOPIC_WIFI_RAW, DRIVER_PATH);
  }

  ~WifiRssiNode() override
  {
    if (serial_fd_ >= 0) ::close(serial_fd_);
    if (drv_fd_ >= 0) ::close(drv_fd_);
  }

private:
  void open_serial()
  {
    if (serial_fd_ >= 0) return;

    serial_fd_ = ::open(SERIAL_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "serial open failed: %s (%s)", SERIAL_PORT, std::strerror(errno));
      return;
    }

    struct termios tio{};
    if (tcgetattr(serial_fd_, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    cfmakeraw(&tio);

    speed_t spd = to_speed_t(SERIAL_BAUD);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;   // no hw flow control
    tio.c_cflag &= ~PARENB;    // no parity
    tio.c_cflag &= ~CSTOPB;    // 1 stop bit
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;        // 8 data bits

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    RCLCPP_INFO(get_logger(), "serial configured OK: %s @ %d", SERIAL_PORT, SERIAL_BAUD);
  }

  void open_driver()
  {
    if (drv_fd_ >= 0) return;

    drv_fd_ = ::open(DRIVER_PATH, O_WRONLY | O_NONBLOCK);
    if (drv_fd_ < 0) {
      // 드라이버가 없어도 ROS 토픽 publish는 계속 한다
      RCLCPP_WARN(get_logger(), "driver open failed (tee disabled until available): %s (%s)",
                  DRIVER_PATH, std::strerror(errno));
      return;
    }
    RCLCPP_INFO(get_logger(), "driver opened OK: %s", DRIVER_PATH);
  }

  void write_all_to_driver(const std::string &s)
  {
    if (drv_fd_ < 0) {
      open_driver();
      if (drv_fd_ < 0) return;
    }

    const char *p = s.data();
    size_t left = s.size();

    while (left > 0) {
      ssize_t w = ::write(drv_fd_, p, left);
      if (w > 0) {
        p += w;
        left -= (size_t)w;
        continue;
      }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // 드라이버가 바쁘면 이번 프레임은 드롭
        break;
      }
      if (w < 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "write to driver failed: %s -> reopen", std::strerror(errno));
        ::close(drv_fd_);
        drv_fd_ = -1;
        break;
      }
      break;
    }
  }

  void tick()
  {
    if (serial_fd_ < 0) {
      open_serial();
      return;
    }

    bool got_any = false;

    for (int k = 0; k < 8; ++k) {
      std::array<char, READ_CHUNK> buf{};
      ssize_t n = ::read(serial_fd_, buf.data(), buf.size() - 1);

      if (n > 0) {
        buf[(size_t)n] = '\0';
        rx_acc_ += buf.data();
        got_any = true;
      } else if (n == 0) {
        break;
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;

        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "serial read failed: %s -> reopen", std::strerror(errno));
        ::close(serial_fd_);
        serial_fd_ = -1;
        return;
      }
    }

    if (!got_any) return;

    if (rx_acc_.size() > MAX_BUFFER_BYTES) {
      rx_acc_.erase(0, rx_acc_.size() - MAX_BUFFER_BYTES);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "rx buffer truncated (MAX_BUFFER_BYTES=%zu)", MAX_BUFFER_BYTES);
    }

    publish_complete_frames();
  }

  void publish_complete_frames()
  {
    while (true) {
      size_t b = rx_acc_.find("BEGIN\n");
      if (b == std::string::npos) {
        // BEGIN이 없으면 찌꺼기 정리
        if (rx_acc_.size() > 8192) rx_acc_.erase(0, rx_acc_.size() - 2048);
        return;
      }

      if (b > 0) rx_acc_.erase(0, b);

      size_t e = rx_acc_.find("END\n");
      if (e == std::string::npos) return;

      size_t end_pos = e + std::string("END\n").size();
      std::string frame = rx_acc_.substr(0, end_pos);
      rx_acc_.erase(0, end_pos);

      custom_interfaces::msg::WifiRaw msg;
      msg.stamp = now();
      msg.frame = frame;
      pub_->publish(msg);

      // tee: 동일 프레임을 드라이버에도 write
      write_all_to_driver(frame);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Published /wifi/raw frame (bytes=%zu)", frame.size());
    }
  }

private:
  int serial_fd_;
  int drv_fd_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::WifiRaw>::SharedPtr pub_;

  std::string rx_acc_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiRssiNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "roka_imu_driver/ebimu_v5_protocol.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{

speed_t baud_to_speed(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B0;
  }
}

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

std::string resolve_ebimu_port(const std::string & configured_port)
{
  if (lowercase(configured_port) != "auto") {
    return configured_port;
  }

  const std::filesystem::path by_id{"/dev/serial/by-id"};
  std::vector<std::string> candidates;
  std::error_code error;
  if (!std::filesystem::is_directory(by_id, error)) {
    return {};
  }
  for (const auto & entry : std::filesystem::directory_iterator(by_id, error)) {
    if (error) {
      break;
    }
    const std::string name = lowercase(entry.path().filename().string());
    const bool supported_uart =
      name.find("cp210") != std::string::npos ||
      name.find("ch340") != std::string::npos ||
      name.find("ch341") != std::string::npos ||
      name.find("wch") != std::string::npos;
    // The two FTDI RS-422/485 adapters belong to the MD200T controllers and
    // must never be mistaken for the TTL EBIMU adapter.
    const bool motor_adapter =
      name.find("ftdi") != std::string::npos ||
      name.find("rs-422") != std::string::npos ||
      name.find("rs_422") != std::string::npos ||
      name.find("rs-485") != std::string::npos ||
      name.find("rs_485") != std::string::npos;
    if (supported_uart && !motor_adapter) {
      candidates.push_back(entry.path().string());
    }
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.size() == 1U ? candidates.front() : std::string{};
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class EbimuV5DriverNode : public rclcpp::Node
{
public:
  EbimuV5DriverNode()
  : Node("ebimu_v5_driver")
  {
    serial_port_ = declare_parameter<std::string>(
      "serial_port",
      "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    baud_rate_ = declare_parameter<int>("baud_rate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    expected_rate_hz_ = declare_parameter<double>("expected_rate_hz", 100.0);
    reconnect_period_sec_ = declare_parameter<double>("reconnect_period_sec", 2.0);
    data_timeout_sec_ = declare_parameter<double>("data_timeout_sec", 0.25);
    allow_sensor_configuration_ =
      declare_parameter<bool>("allow_sensor_configuration", false);
    orientation_validated_ = declare_parameter<bool>("orientation_validated", false);
    orientation_world_to_sensor_ = declare_parameter<bool>(
      "orientation_world_to_sensor", true);
    use_sensor_timestamp_ = declare_parameter<bool>("use_sensor_timestamp", true);
    polling_mode_ = declare_parameter<bool>("polling_mode", false);
    send_start_on_open_ = declare_parameter<bool>("send_start_on_open", true);
    auto_poll_fallback_ = declare_parameter<bool>("auto_poll_fallback", true);
    auto_poll_delay_sec_ = declare_parameter<double>("auto_poll_delay_sec", 0.50);
    sensor_to_imu_rpy_ = declare_parameter<std::vector<double>>(
      "sensor_to_imu_rpy", std::vector<double>{0.0, 0.0, 0.0});
    orientation_stddev_rpy_ = declare_parameter<std::vector<double>>(
      "orientation_stddev_rpy", std::vector<double>{0.035, 0.035, 0.175});
    angular_velocity_stddev_ = declare_parameter<std::vector<double>>(
      "angular_velocity_stddev", std::vector<double>{0.02, 0.02, 0.02});
    linear_acceleration_stddev_ = declare_parameter<std::vector<double>>(
      "linear_acceleration_stddev", std::vector<double>{0.20, 0.20, 0.20});

    const std::string output_code = lowercase(
      declare_parameter<std::string>("output_code", "binary"));
    const std::string orientation_format = lowercase(
      declare_parameter<std::string>("orientation_format", "quaternion"));
    layout_.output_code = output_code == "ascii" ?
      roka_imu_driver::EbimuOutputCode::kAscii :
      roka_imu_driver::EbimuOutputCode::kBinary;
    layout_.orientation_format = orientation_format == "euler" ?
      roka_imu_driver::EbimuOrientationFormat::kEuler :
      roka_imu_driver::EbimuOrientationFormat::kQuaternion;
    layout_.gyro = declare_parameter<bool>("output_gyro", true);
    layout_.accelerometer = declare_parameter<bool>("output_accelerometer", true);
    layout_.magnetometer = declare_parameter<bool>("output_magnetometer", false);
    layout_.temperature = declare_parameter<bool>("output_temperature", false);
    layout_.timestamp = declare_parameter<bool>("output_timestamp", true);

    validate_parameters();
    parser_ = std::make_unique<roka_imu_driver::EbimuV5Parser>(layout_);
    sensor_to_imu_.setRPY(
      sensor_to_imu_rpy_[0], sensor_to_imu_rpy_[1], sensor_to_imu_rpy_[2]);
    sensor_to_imu_.normalize();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "imu/rpy", rclcpp::SensorDataQoS());
    yaw_pub_ = create_publisher<std_msgs::msg::Float64>("imu/yaw", 10);
    magnetic_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "imu/mag", rclcpp::SensorDataQoS());
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      "imu/temperature", rclcpp::SensorDataQoS());
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("imu/connected", 1);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    configure_service_ = create_service<std_srvs::srv::Trigger>(
      "imu/configure",
      std::bind(
        &EbimuV5DriverNode::handle_configure, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "imu/reset",
      std::bind(
        &EbimuV5DriverNode::handle_reset, this,
        std::placeholders::_1, std::placeholders::_2));

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&EbimuV5DriverNode::poll_serial, this));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&EbimuV5DriverNode::publish_diagnostics, this));

    open_serial();
    RCLCPP_INFO(
      get_logger(),
      "EBIMU-9DOFV5 driver: port=%s baud=%d format=%s orientation=%s convention=%s "
      "mode=%s expected_rate=%.1fHz",
      serial_port_.c_str(), baud_rate_, output_code.c_str(),
      orientation_format.c_str(),
      orientation_world_to_sensor_ ? "world_to_sensor" : "sensor_to_world",
      polling_mode_ ? "polling" : "continuous",
      expected_rate_hz_);
    if (!orientation_validated_) {
      RCLCPP_WARN(
        get_logger(),
        "Orientation is marked unvalidated; orientation_covariance[0] will be -1 until "
        "mounting axes and ENU convention are verified on hardware");
    }
  }

  ~EbimuV5DriverNode() override
  {
    close_serial();
  }

private:
  void validate_parameters()
  {
    if (baud_to_speed(baud_rate_) == B0) {
      throw std::invalid_argument("Unsupported baud_rate");
    }
    if (expected_rate_hz_ < 1.0 || expected_rate_hz_ > 1000.0) {
      throw std::invalid_argument("expected_rate_hz must be between 1 and 1000");
    }
    if (data_timeout_sec_ <= 0.0 || reconnect_period_sec_ <= 0.0) {
      throw std::invalid_argument("timeout and reconnect period must be positive");
    }
    if (auto_poll_delay_sec_ <= data_timeout_sec_) {
      throw std::invalid_argument("auto_poll_delay_sec must exceed data_timeout_sec");
    }
    if (sensor_to_imu_rpy_.size() != 3U) {
      throw std::invalid_argument("sensor_to_imu_rpy must contain roll, pitch, yaw radians");
    }
    if (orientation_stddev_rpy_.size() != 3U ||
      angular_velocity_stddev_.size() != 3U ||
      linear_acceleration_stddev_.size() != 3U)
    {
      throw std::invalid_argument("all standard-deviation parameters must contain x, y, z");
    }
    if (use_sensor_timestamp_ && !layout_.timestamp) {
      throw std::invalid_argument("use_sensor_timestamp requires output_timestamp=true");
    }
  }

  void open_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      return;
    }
    active_serial_port_ = resolve_ebimu_port(serial_port_);
    if (active_serial_port_.empty()) {
      if (!open_warning_active_) {
        RCLCPP_WARN(
          get_logger(),
          "Unable to resolve EBIMU serial_port='%s': connect exactly one CP210x or CH340/CH341 TTL adapter",
          serial_port_.c_str());
        open_warning_active_ = true;
      }
      return;
    }
    serial_fd_ = ::open(active_serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      if (!open_warning_active_) {
        RCLCPP_WARN(
          get_logger(), "Unable to open EBIMU port '%s': %s",
          active_serial_port_.c_str(), std::strerror(errno));
        open_warning_active_ = true;
      }
      return;
    }
    if (::flock(serial_fd_, LOCK_EX | LOCK_NB) != 0) {
      const int lock_error = errno;
      if (!open_warning_active_) {
        RCLCPP_WARN(
          get_logger(),
          "EBIMU port '%s' is already owned by another ROKA IMU process: %s",
          active_serial_port_.c_str(), std::strerror(lock_error));
        open_warning_active_ = true;
      }
      ::close(serial_fd_);
      serial_fd_ = -1;
      return;
    }
#ifdef TIOCEXCL
    // Also reject unrelated late opens (stty/dd/another driver) while this
    // process owns the physical tty. The advisory flock above detects another
    // ROKA driver that opened the device first.
    if (::ioctl(serial_fd_, TIOCEXCL) != 0 && errno != ENOTTY) {
      RCLCPP_WARN(
        get_logger(), "Unable to make EBIMU tty exclusive: %s", std::strerror(errno));
    }
#endif

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close_serial_locked();
      return;
    }
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    const speed_t speed = baud_to_speed(baud_rate_);
    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
      tcsetattr(serial_fd_, TCSANOW, &options) != 0)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure EBIMU serial port");
      close_serial_locked();
      return;
    }
    tcflush(serial_fd_, TCIOFLUSH);
    parser_->reset();
    timestamp_unwrapper_.reset();
    sensor_time_initialized_ = false;
    connected_ = false;
    fallback_polling_active_ = polling_mode_;
    serial_open_time_ = std::chrono::steady_clock::now();
    last_valid_sample_time_ = serial_open_time_;
    last_reconnect_attempt_ = std::chrono::steady_clock::now();
    last_poll_request_time_ = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / expected_rate_hz_));
    // <start> only resumes operation. Unlike <soc>/<sof>/<sor>, it does not
    // rewrite the sensor's nonvolatile output layout on every boot.
    if (send_start_on_open_ && !write_command_locked("<start>")) {
      RCLCPP_WARN(get_logger(), "Unable to send EBIMU <start> command");
    }
    RCLCPP_INFO(
      get_logger(), "Opened EBIMU-9DOFV5 port '%s'", active_serial_port_.c_str());
    open_warning_active_ = false;
  }

  void close_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    close_serial_locked();
  }

  void close_serial_locked()
  {
    if (serial_fd_ >= 0) {
#ifdef TIOCNXCL
      (void)::ioctl(serial_fd_, TIOCNXCL);
#endif
      (void)::flock(serial_fd_, LOCK_UN);
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
    set_connected(false);
    sensor_time_initialized_ = false;
  }

  bool write_command_locked(const std::string & command)
  {
    if (serial_fd_ < 0) {
      return false;
    }
    std::size_t offset = 0U;
    while (offset < command.size()) {
      const ssize_t count = ::write(
        serial_fd_, command.data() + offset, command.size() - offset);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      offset += static_cast<std::size_t>(count);
    }
    tcdrain(serial_fd_);
    return true;
  }

  void poll_serial()
  {
    if (serial_fd_ < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_reconnect_attempt_ >
        std::chrono::duration<double>(reconnect_period_sec_))
      {
        last_reconnect_attempt_ = now;
        open_serial();
      }
      return;
    }

    // EBIMU-9DOFV5 uses one '*' byte to request the current sample when
    // <sor0> polling mode is selected. This also provides a
    // non-destructive commissioning path before continuous output is saved.
    const auto now = std::chrono::steady_clock::now();
    if (!polling_mode_ && auto_poll_fallback_ && !connected_ &&
      now - last_valid_sample_time_ >= std::chrono::duration<double>(auto_poll_delay_sec_))
    {
      fallback_polling_active_ = true;
    }
    if (fallback_polling_active_) {
      const auto request_period = std::chrono::duration<double>(1.0 / expected_rate_hz_);
      if (now - last_poll_request_time_ >= request_period) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ >= 0 && write_command_locked("*")) {
          last_poll_request_time_ = now;
        }
      }
    }

    std::array<std::uint8_t, 1024> buffer{};
    while (true) {
      ssize_t count = 0;
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ < 0) {
          return;
        }
        count = ::read(serial_fd_, buffer.data(), buffer.size());
      }
      if (count > 0) {
        const auto samples = parser_->feed(buffer.data(), static_cast<std::size_t>(count));
        for (const auto & sample : samples) {
          publish_sample(sample);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "EBIMU serial read failed: %s", std::strerror(errno));
      close_serial();
      break;
    }
  }

  rclcpp::Time sample_stamp(const roka_imu_driver::EbimuV5Sample & sample)
  {
    const rclcpp::Time now = get_clock()->now();
    if (!use_sensor_timestamp_ || !sample.has_timestamp) {
      return now;
    }
    const std::uint64_t unwrapped_ms = timestamp_unwrapper_.unwrap(sample.timestamp_ms);
    if (!sensor_time_initialized_) {
      sensor_time_initialized_ = true;
      sensor_time_base_ms_ = unwrapped_ms;
      ros_time_base_ = now;
      return now;
    }
    // A sensor reset can move its one-minute counter backwards without looking
    // like the normal 60 s wrap. Re-anchor instead of allowing unsigned time
    // subtraction to create a far-future ROS timestamp.
    if (unwrapped_ms < sensor_time_base_ms_) {
      sensor_time_base_ms_ = unwrapped_ms;
      ros_time_base_ = now;
      return now;
    }
    const double elapsed_sec = static_cast<double>(unwrapped_ms - sensor_time_base_ms_) / 1000.0;
    rclcpp::Time stamp = ros_time_base_ + rclcpp::Duration::from_seconds(elapsed_sec);
    if (std::abs((now - stamp).seconds()) > 1.0) {
      sensor_time_base_ms_ = unwrapped_ms;
      ros_time_base_ = now;
      stamp = now;
    }
    return stamp;
  }

  std::array<double, 3> rotate_vector(const std::array<double, 3> & input) const
  {
    const tf2::Vector3 transformed = tf2::quatRotate(
      sensor_to_imu_, tf2::Vector3(input[0], input[1], input[2]));
    return {{transformed.x(), transformed.y(), transformed.z()}};
  }

  tf2::Quaternion output_orientation(
    const roka_imu_driver::EbimuV5Sample & sample) const
  {
    tf2::Quaternion sensor_orientation;
    if (sample.orientation_is_quaternion) {
      const auto quaternion = roka_imu_driver::convert_orientation_convention(
        sample.quaternion_xyzw, orientation_world_to_sensor_);
      sensor_orientation = tf2::Quaternion(
        quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    } else {
      sensor_orientation.setRPY(
        sample.euler_rpy_rad[0], sample.euler_rpy_rad[1], sample.euler_rpy_rad[2]);
      if (orientation_world_to_sensor_) {
        sensor_orientation = sensor_orientation.inverse();
      }
    }
    sensor_orientation.normalize();
    tf2::Quaternion result = sensor_orientation * sensor_to_imu_.inverse();
    result.normalize();
    return result;
  }

  void publish_sample(const roka_imu_driver::EbimuV5Sample & sample)
  {
    const rclcpp::Time stamp = sample_stamp(sample);
    const tf2::Quaternion orientation = output_orientation(sample);
    const auto gyro = rotate_vector(sample.angular_velocity_rad_s);
    const auto acceleration = rotate_vector(sample.linear_acceleration_m_s2);

    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.orientation.x = orientation.x();
    message.orientation.y = orientation.y();
    message.orientation.z = orientation.z();
    message.orientation.w = orientation.w();
    if (!orientation_validated_) {
      message.orientation_covariance[0] = -1.0;
    } else {
      for (std::size_t i = 0U; i < 3U; ++i) {
        message.orientation_covariance[i * 3U + i] =
          orientation_stddev_rpy_[i] * orientation_stddev_rpy_[i];
      }
    }
    if (sample.has_gyro) {
      message.angular_velocity.x = gyro[0];
      message.angular_velocity.y = gyro[1];
      message.angular_velocity.z = gyro[2];
      for (std::size_t i = 0U; i < 3U; ++i) {
        message.angular_velocity_covariance[i * 3U + i] =
          angular_velocity_stddev_[i] * angular_velocity_stddev_[i];
      }
    } else {
      message.angular_velocity_covariance[0] = -1.0;
    }
    if (sample.has_accelerometer) {
      message.linear_acceleration.x = acceleration[0];
      message.linear_acceleration.y = acceleration[1];
      message.linear_acceleration.z = acceleration[2];
      for (std::size_t i = 0U; i < 3U; ++i) {
        message.linear_acceleration_covariance[i * 3U + i] =
          linear_acceleration_stddev_[i] * linear_acceleration_stddev_[i];
      }
    } else {
      message.linear_acceleration_covariance[0] = -1.0;
    }
    imu_pub_->publish(message);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.header = message.header;
    rpy.vector.x = roll;
    rpy.vector.y = pitch;
    rpy.vector.z = yaw;
    rpy_pub_->publish(rpy);
    std_msgs::msg::Float64 yaw_message;
    yaw_message.data = yaw;
    yaw_pub_->publish(yaw_message);

    if (sample.has_magnetometer) {
      const auto magnetic = rotate_vector(sample.magnetic_field_t);
      sensor_msgs::msg::MagneticField magnetic_message;
      magnetic_message.header = message.header;
      magnetic_message.magnetic_field.x = magnetic[0];
      magnetic_message.magnetic_field.y = magnetic[1];
      magnetic_message.magnetic_field.z = magnetic[2];
      magnetic_pub_->publish(magnetic_message);
    }
    if (sample.has_temperature) {
      sensor_msgs::msg::Temperature temperature_message;
      temperature_message.header = message.header;
      temperature_message.temperature = sample.temperature_c;
      temperature_pub_->publish(temperature_message);
    }

    last_valid_sample_time_ = std::chrono::steady_clock::now();
    ++frames_since_diagnostic_;
    set_connected(true);
  }

  void set_connected(bool value)
  {
    if (connected_ == value && connected_state_published_) {
      return;
    }
    connected_ = value;
    connected_state_published_ = true;
    std_msgs::msg::Bool message;
    message.data = value;
    connected_pub_->publish(message);
  }

  void publish_diagnostics()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const double diagnostic_period = std::chrono::duration<double>(
      now_steady - last_diagnostic_time_).count();
    const double measured_rate = diagnostic_period > 0.0 ?
      static_cast<double>(frames_since_diagnostic_) / diagnostic_period : 0.0;
    frames_since_diagnostic_ = 0U;
    last_diagnostic_time_ = now_steady;

    const bool stale = connected_ &&
      std::chrono::duration<double>(now_steady - last_valid_sample_time_).count() >
      data_timeout_sec_;
    if (stale) {
      set_connected(false);
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "EBIMU-9DOFV5";
    status.hardware_id = active_serial_port_.empty() ? serial_port_ : active_serial_port_;
    if (serial_fd_ < 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial port unavailable";
    } else if (!connected_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "no valid EBIMU frame";
    } else if (measured_rate < expected_rate_hz_ * 0.8) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "IMU frame rate below target";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "streaming";
    }
    const auto & statistics = parser_->statistics();
    status.values.push_back(diagnostic_value("configured_port", serial_port_));
    status.values.push_back(diagnostic_value("active_port", active_serial_port_));
    status.values.push_back(diagnostic_value("baud_rate", std::to_string(baud_rate_)));
    status.values.push_back(diagnostic_value("sensor_model", "EBIMU-9DOFV5 R3.0a"));
    status.values.push_back(
      diagnostic_value(
        "output_mode",
        polling_mode_ ? "polling" :
        (fallback_polling_active_ ? "automatic_polling_fallback" : "continuous")));
    status.values.push_back(diagnostic_value("measured_rate_hz", std::to_string(measured_rate)));
    status.values.push_back(
      diagnostic_value(
        "valid_frames", std::to_string(
          statistics.valid_frames)));
    status.values.push_back(
      diagnostic_value(
        "checksum_errors",
        std::to_string(statistics.checksum_errors)));
    status.values.push_back(
      diagnostic_value(
        "format_errors",
        std::to_string(statistics.format_errors)));
    status.values.push_back(
      diagnostic_value(
        "discarded_bytes",
        std::to_string(statistics.discarded_bytes)));
    status.values.push_back(
      diagnostic_value(
        "orientation_validated", orientation_validated_ ? "true" : "false"));
    status.values.push_back(
      diagnostic_value(
        "orientation_convention",
        orientation_world_to_sensor_ ? "world_to_sensor_inverted" : "sensor_to_world"));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);

    std_msgs::msg::Bool connected_message;
    connected_message.data = connected_;
    connected_pub_->publish(connected_message);
  }

  void handle_configure(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!allow_sensor_configuration_) {
      response->success = false;
      response->message =
        "configuration is locked; set allow_sensor_configuration=true for commissioning";
      return;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ < 0) {
      response->success = false;
      response->message = "serial port is not open";
      return;
    }

    const int output_period_ms = polling_mode_ ? 0 : std::max(
      1, std::min(1000, static_cast<int>(std::lround(1000.0 / expected_rate_hz_))));
    const std::vector<std::string> commands{
      "<stop>",
      layout_.output_code == roka_imu_driver::EbimuOutputCode::kBinary ? "<soc2>" : "<soc1>",
      layout_.orientation_format == roka_imu_driver::EbimuOrientationFormat::kQuaternion ?
      "<sof2>" : "<sof1>",
      layout_.gyro ? "<sog1>" : "<sog0>",
      layout_.accelerometer ? "<soa1>" : "<soa0>",
      layout_.magnetometer ? "<som1>" : "<som0>",
      layout_.temperature ? "<sot1>" : "<sot0>",
      layout_.timestamp ? "<sots1>" : "<sots0>",
      "<sor" + std::to_string(output_period_ms) + ">",
      "<pons1>",
      "<start>",
    };
    tcflush(serial_fd_, TCIOFLUSH);
    bool success = true;
    for (const auto & command : commands) {
      success = write_command_locked(command) && success;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    parser_->reset();
    timestamp_unwrapper_.reset();
    sensor_time_initialized_ = false;
    response->success = success;
    response->message = success ?
      "configuration commands written; verify /diagnostics and then relock configuration" :
      "failed to write one or more configuration commands";
  }

  void handle_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    response->success = write_command_locked("<reset>");
    response->message = response->success ? "reset command written" : "reset command failed";
  }

  std::string serial_port_;
  std::string active_serial_port_;
  int baud_rate_{921600};
  std::string frame_id_{"imu_link"};
  double expected_rate_hz_{100.0};
  double reconnect_period_sec_{2.0};
  double data_timeout_sec_{0.25};
  bool allow_sensor_configuration_{false};
  bool orientation_validated_{false};
  bool orientation_world_to_sensor_{false};
  bool use_sensor_timestamp_{true};
  bool polling_mode_{false};
  bool send_start_on_open_{true};
  bool auto_poll_fallback_{true};
  bool fallback_polling_active_{false};
  double auto_poll_delay_sec_{0.50};
  bool open_warning_active_{false};
  std::vector<double> sensor_to_imu_rpy_;
  std::vector<double> orientation_stddev_rpy_;
  std::vector<double> angular_velocity_stddev_;
  std::vector<double> linear_acceleration_stddev_;
  roka_imu_driver::EbimuV5Layout layout_;
  std::unique_ptr<roka_imu_driver::EbimuV5Parser> parser_;
  roka_imu_driver::EbimuTimestampUnwrapper timestamp_unwrapper_;
  tf2::Quaternion sensor_to_imu_;

  int serial_fd_{-1};
  std::mutex serial_mutex_;
  bool connected_{false};
  bool connected_state_published_{false};
  bool sensor_time_initialized_{false};
  std::uint64_t sensor_time_base_ms_{0U};
  rclcpp::Time ros_time_base_{0, 0, RCL_SYSTEM_TIME};
  std::uint64_t frames_since_diagnostic_{0U};
  std::chrono::steady_clock::time_point last_valid_sample_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_diagnostic_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_reconnect_attempt_{
    std::chrono::steady_clock::now() - std::chrono::seconds(10)};
  std::chrono::steady_clock::time_point last_poll_request_time_{
    std::chrono::steady_clock::now() - std::chrono::seconds(1)};
  std::chrono::steady_clock::time_point serial_open_time_{
    std::chrono::steady_clock::now()};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr configure_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EbimuV5DriverNode>());
  rclcpp::shutdown();
  return 0;
}

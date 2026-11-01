#ifndef ROKA_IMU_DRIVER__EBIMU_V5_PROTOCOL_HPP_
#define ROKA_IMU_DRIVER__EBIMU_V5_PROTOCOL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace roka_imu_driver
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

enum class EbimuOutputCode
{
  kAscii,
  kBinary,
};

enum class EbimuOrientationFormat
{
  kEuler,
  kQuaternion,
};

struct EbimuV5Layout
{
  EbimuOutputCode output_code{EbimuOutputCode::kBinary};
  EbimuOrientationFormat orientation_format{EbimuOrientationFormat::kQuaternion};
  bool gyro{true};
  bool accelerometer{true};
  bool magnetometer{false};
  bool temperature{false};
  bool timestamp{true};

  std::size_t data_word_count() const
  {
    std::size_t count = orientation_format == EbimuOrientationFormat::kQuaternion ? 4U : 3U;
    count += gyro ? 3U : 0U;
    count += accelerometer ? 3U : 0U;
    count += magnetometer ? 3U : 0U;
    count += temperature ? 1U : 0U;
    count += timestamp ? 1U : 0U;
    return count;
  }
};

struct EbimuV5Sample
{
  bool orientation_is_quaternion{true};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> euler_rpy_rad{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity_rad_s{{0.0, 0.0, 0.0}};
  std::array<double, 3> linear_acceleration_m_s2{{0.0, 0.0, 0.0}};
  std::array<double, 3> magnetic_field_t{{0.0, 0.0, 0.0}};
  double temperature_c{0.0};
  std::uint32_t timestamp_ms{0U};
  bool has_gyro{false};
  bool has_accelerometer{false};
  bool has_magnetometer{false};
  bool has_temperature{false};
  bool has_timestamp{false};
};

inline std::array<double, 4> convert_orientation_convention(
  const std::array<double, 4> & quaternion_xyzw,
  bool world_to_sensor)
{
  if (!world_to_sensor) {
    return quaternion_xyzw;
  }
  // A unit quaternion inverse is its conjugate. EBIMU-9DOFV5 R3.0a was
  // measured to report world->sensor, while sensor_msgs/Imu requires the
  // sensor orientation in the world frame (sensor->world).
  return {{
      -quaternion_xyzw[0],
      -quaternion_xyzw[1],
      -quaternion_xyzw[2],
      quaternion_xyzw[3]}};
}

struct EbimuParserStatistics
{
  std::uint64_t valid_frames{0U};
  std::uint64_t checksum_errors{0U};
  std::uint64_t format_errors{0U};
  std::uint64_t discarded_bytes{0U};
};

class EbimuTimestampUnwrapper
{
public:
  std::uint64_t unwrap(std::uint32_t timestamp_ms)
  {
    constexpr std::uint32_t kWrapMs = 60000U;
    constexpr std::uint32_t kWrapDetectionThresholdMs = 30000U;

    timestamp_ms %= (kWrapMs + 1U);
    if (!initialized_) {
      initialized_ = true;
      previous_raw_ms_ = timestamp_ms;
      return timestamp_ms;
    }

    if (previous_raw_ms_ > timestamp_ms &&
      previous_raw_ms_ - timestamp_ms > kWrapDetectionThresholdMs)
    {
      epoch_ms_ += kWrapMs;
    }
    previous_raw_ms_ = timestamp_ms;
    return epoch_ms_ + timestamp_ms;
  }

  void reset()
  {
    initialized_ = false;
    previous_raw_ms_ = 0U;
    epoch_ms_ = 0U;
  }

private:
  bool initialized_{false};
  std::uint32_t previous_raw_ms_{0U};
  std::uint64_t epoch_ms_{0U};
};

class EbimuV5Parser
{
public:
  explicit EbimuV5Parser(EbimuV5Layout layout)
  : layout_(layout)
  {
  }

  std::vector<EbimuV5Sample> feed(const std::uint8_t * data, std::size_t size)
  {
    buffer_.insert(buffer_.end(), data, data + size);
    if (buffer_.size() > kMaximumBufferSize) {
      const auto excess = buffer_.size() - kMaximumBufferSize;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
      statistics_.discarded_bytes += excess;
    }
    return layout_.output_code == EbimuOutputCode::kBinary ? parse_binary() : parse_ascii();
  }

  std::vector<EbimuV5Sample> feed(const std::vector<std::uint8_t> & data)
  {
    return feed(data.data(), data.size());
  }

  const EbimuParserStatistics & statistics() const
  {
    return statistics_;
  }

  void reset()
  {
    buffer_.clear();
    statistics_ = EbimuParserStatistics{};
  }

private:
  static constexpr std::size_t kMaximumBufferSize = 65536U;

  static std::int16_t read_i16_be(const std::uint8_t * data)
  {
    const auto value = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[0]) << 8U) |
      static_cast<std::uint16_t>(data[1]));
    return static_cast<std::int16_t>(value);
  }

  static std::uint16_t read_u16_be(const std::uint8_t * data)
  {
    return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[0]) << 8U) |
      static_cast<std::uint16_t>(data[1]));
  }

  static void normalize_quaternion(std::array<double, 4> * quaternion)
  {
    const double norm = std::sqrt(
      (*quaternion)[0] * (*quaternion)[0] +
      (*quaternion)[1] * (*quaternion)[1] +
      (*quaternion)[2] * (*quaternion)[2] +
      (*quaternion)[3] * (*quaternion)[3]);
    if (norm > 1.0e-9) {
      for (double & value : *quaternion) {
        value /= norm;
      }
    } else {
      *quaternion = {{0.0, 0.0, 0.0, 1.0}};
    }
  }

  EbimuV5Sample decode_binary_words(const std::vector<std::uint16_t> & words) const
  {
    EbimuV5Sample sample;
    std::size_t index = 0U;
    const auto signed_word = [&words](std::size_t i) {
        return static_cast<std::int16_t>(words.at(i));
      };

    if (layout_.orientation_format == EbimuOrientationFormat::kQuaternion) {
      // The validated EBIMU-9DOFV5 R3.0a binary order is z, y, x, w.
      const double z = static_cast<double>(signed_word(index++)) / 10000.0;
      const double y = static_cast<double>(signed_word(index++)) / 10000.0;
      const double x = static_cast<double>(signed_word(index++)) / 10000.0;
      const double w = static_cast<double>(signed_word(index++)) / 10000.0;
      sample.quaternion_xyzw = {{x, y, z, w}};
      normalize_quaternion(&sample.quaternion_xyzw);
      sample.orientation_is_quaternion = true;
    } else {
      for (double & angle : sample.euler_rpy_rad) {
        angle = static_cast<double>(signed_word(index++)) / 100.0 * kPi / 180.0;
      }
      sample.orientation_is_quaternion = false;
    }

    if (layout_.gyro) {
      for (double & value : sample.angular_velocity_rad_s) {
        value = static_cast<double>(signed_word(index++)) / 10.0 * kPi / 180.0;
      }
      sample.has_gyro = true;
    }
    if (layout_.accelerometer) {
      for (double & value : sample.linear_acceleration_m_s2) {
        value = static_cast<double>(signed_word(index++)) / 1000.0 * kGravity;
      }
      sample.has_accelerometer = true;
    }
    if (layout_.magnetometer) {
      for (double & value : sample.magnetic_field_t) {
        value = static_cast<double>(signed_word(index++)) / 10.0 * 1.0e-6;
      }
      sample.has_magnetometer = true;
    }
    if (layout_.temperature) {
      sample.temperature_c = static_cast<double>(signed_word(index++)) / 10.0;
      sample.has_temperature = true;
    }
    if (layout_.timestamp) {
      sample.timestamp_ms = words.at(index++);
      sample.has_timestamp = true;
    }
    return sample;
  }

  std::vector<EbimuV5Sample> parse_binary()
  {
    std::vector<EbimuV5Sample> samples;
    const std::size_t data_words = layout_.data_word_count();
    const std::size_t packet_size = 2U + data_words * 2U + 2U;

    while (buffer_.size() >= 2U) {
      auto header = std::search(
        buffer_.begin(), buffer_.end(), kHeader.begin(), kHeader.end());
      if (header == buffer_.end()) {
        const bool keep_last = buffer_.back() == 0x55U;
        const auto discarded = buffer_.size() - (keep_last ? 1U : 0U);
        statistics_.discarded_bytes += discarded;
        if (keep_last) {
          buffer_.erase(buffer_.begin(), buffer_.end() - 1);
        } else {
          buffer_.clear();
        }
        break;
      }
      if (header != buffer_.begin()) {
        const auto discarded = static_cast<std::size_t>(std::distance(buffer_.begin(), header));
        statistics_.discarded_bytes += discarded;
        buffer_.erase(buffer_.begin(), header);
      }
      if (buffer_.size() < packet_size) {
        break;
      }

      std::uint16_t checksum = 0U;
      for (std::size_t i = 0U; i < packet_size - 2U; ++i) {
        checksum = static_cast<std::uint16_t>(checksum + buffer_[i]);
      }
      const auto received_checksum = read_u16_be(&buffer_[packet_size - 2U]);
      if (checksum != received_checksum) {
        ++statistics_.checksum_errors;
        ++statistics_.discarded_bytes;
        buffer_.erase(buffer_.begin());
        continue;
      }

      std::vector<std::uint16_t> words;
      words.reserve(data_words);
      for (std::size_t i = 0U; i < data_words; ++i) {
        words.push_back(read_u16_be(&buffer_[2U + i * 2U]));
      }
      try {
        samples.push_back(decode_binary_words(words));
        ++statistics_.valid_frames;
      } catch (const std::exception &) {
        ++statistics_.format_errors;
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
    }
    return samples;
  }

  static bool parse_ascii_values(const std::string & line, std::vector<double> * values)
  {
    values->clear();
    if (line.empty() || line.front() != '*') {
      return false;
    }
    std::stringstream stream(line.substr(1U));
    std::string token;
    try {
      while (std::getline(stream, token, ',')) {
        if (!token.empty() && token.back() == '\r') {
          token.pop_back();
        }
        values->push_back(std::stod(token));
      }
    } catch (const std::exception &) {
      return false;
    }
    return true;
  }

  EbimuV5Sample decode_ascii_values(const std::vector<double> & values) const
  {
    EbimuV5Sample sample;
    std::size_t index = 0U;
    if (layout_.orientation_format == EbimuOrientationFormat::kQuaternion) {
      const double z = values.at(index++);
      const double y = values.at(index++);
      const double x = values.at(index++);
      const double w = values.at(index++);
      sample.quaternion_xyzw = {{x, y, z, w}};
      normalize_quaternion(&sample.quaternion_xyzw);
      sample.orientation_is_quaternion = true;
    } else {
      for (double & angle : sample.euler_rpy_rad) {
        angle = values.at(index++) * kPi / 180.0;
      }
      sample.orientation_is_quaternion = false;
    }
    if (layout_.gyro) {
      for (double & value : sample.angular_velocity_rad_s) {
        value = values.at(index++) * kPi / 180.0;
      }
      sample.has_gyro = true;
    }
    if (layout_.accelerometer) {
      for (double & value : sample.linear_acceleration_m_s2) {
        value = values.at(index++) * kGravity;
      }
      sample.has_accelerometer = true;
    }
    if (layout_.magnetometer) {
      for (double & value : sample.magnetic_field_t) {
        value = values.at(index++) * 1.0e-6;
      }
      sample.has_magnetometer = true;
    }
    if (layout_.temperature) {
      sample.temperature_c = values.at(index++);
      sample.has_temperature = true;
    }
    if (layout_.timestamp) {
      sample.timestamp_ms = static_cast<std::uint32_t>(std::llround(values.at(index++)));
      sample.has_timestamp = true;
    }
    return sample;
  }

  std::vector<EbimuV5Sample> parse_ascii()
  {
    std::vector<EbimuV5Sample> samples;
    while (true) {
      const auto newline = std::find(buffer_.begin(), buffer_.end(), static_cast<std::uint8_t>('\n'));
      if (newline == buffer_.end()) {
        break;
      }
      const std::string line(buffer_.begin(), newline);
      buffer_.erase(buffer_.begin(), newline + 1);
      std::vector<double> values;
      if (!parse_ascii_values(line, &values) || values.size() != layout_.data_word_count()) {
        ++statistics_.format_errors;
        continue;
      }
      try {
        samples.push_back(decode_ascii_values(values));
        ++statistics_.valid_frames;
      } catch (const std::exception &) {
        ++statistics_.format_errors;
      }
    }
    return samples;
  }

  EbimuV5Layout layout_;
  std::vector<std::uint8_t> buffer_;
  EbimuParserStatistics statistics_;
  const std::array<std::uint8_t, 2> kHeader{{0x55U, 0x55U}};
};

}  // namespace roka_imu_driver

#endif  // ROKA_IMU_DRIVER__EBIMU_V5_PROTOCOL_HPP_

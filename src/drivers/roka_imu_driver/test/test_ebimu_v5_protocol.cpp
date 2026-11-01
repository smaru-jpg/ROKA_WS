#include <cmath>
#include <cstdint>
#include <vector>

#include "roka_imu_driver/ebimu_v5_protocol.hpp"
#include "gtest/gtest.h"

namespace
{

std::vector<std::uint8_t> make_binary_packet(const std::vector<std::uint16_t> & words)
{
  std::vector<std::uint8_t> packet{0x55U, 0x55U};
  for (const auto word : words) {
    packet.push_back(static_cast<std::uint8_t>((word >> 8U) & 0xffU));
    packet.push_back(static_cast<std::uint8_t>(word & 0xffU));
  }
  std::uint16_t checksum = 0U;
  for (const auto byte : packet) {
    checksum = static_cast<std::uint16_t>(checksum + byte);
  }
  packet.push_back(static_cast<std::uint8_t>((checksum >> 8U) & 0xffU));
  packet.push_back(static_cast<std::uint8_t>(checksum & 0xffU));
  return packet;
}

std::uint16_t signed_word(std::int16_t value)
{
  return static_cast<std::uint16_t>(value);
}

TEST(EbimuV5Protocol, DecodesBinaryQuaternionAndPhysicalUnits)
{
  roka_imu_driver::EbimuV5Layout layout;
  roka_imu_driver::EbimuV5Parser parser(layout);

  // V5 binary order: quaternion z,y,x,w; gyro xyz; acceleration xyz; timestamp.
  const auto packet = make_binary_packet({
    signed_word(0), signed_word(0), signed_word(0), signed_word(10000),
    signed_word(100), signed_word(-200), signed_word(300),
    signed_word(1000), signed_word(0), signed_word(-1000),
    1234U,
  });
  const auto samples = parser.feed(packet);

  ASSERT_EQ(samples.size(), 1U);
  const auto & sample = samples.front();
  EXPECT_DOUBLE_EQ(sample.quaternion_xyzw[0], 0.0);
  EXPECT_DOUBLE_EQ(sample.quaternion_xyzw[1], 0.0);
  EXPECT_DOUBLE_EQ(sample.quaternion_xyzw[2], 0.0);
  EXPECT_DOUBLE_EQ(sample.quaternion_xyzw[3], 1.0);
  EXPECT_NEAR(
    sample.angular_velocity_rad_s[0], 10.0 * roka_imu_driver::kPi / 180.0, 1.0e-12);
  EXPECT_NEAR(
    sample.angular_velocity_rad_s[1], -20.0 * roka_imu_driver::kPi / 180.0, 1.0e-12);
  EXPECT_NEAR(
    sample.angular_velocity_rad_s[2], 30.0 * roka_imu_driver::kPi / 180.0, 1.0e-12);
  EXPECT_NEAR(sample.linear_acceleration_m_s2[0], 9.80665, 1.0e-12);
  EXPECT_NEAR(sample.linear_acceleration_m_s2[1], 0.0, 1.0e-12);
  EXPECT_NEAR(sample.linear_acceleration_m_s2[2], -9.80665, 1.0e-12);
  EXPECT_EQ(sample.timestamp_ms, 1234U);
  EXPECT_EQ(parser.statistics().valid_frames, 1U);
}

TEST(EbimuV5Protocol, RejectsBadChecksumAndResynchronizes)
{
  roka_imu_driver::EbimuV5Layout layout;
  roka_imu_driver::EbimuV5Parser parser(layout);
  const std::vector<std::uint16_t> words{
    0U, 0U, 0U, 10000U, 0U, 0U, 0U, 0U, 0U, 1000U, 42U,
  };
  auto bad = make_binary_packet(words);
  bad.back() ^= 0x01U;
  const auto good = make_binary_packet(words);
  std::vector<std::uint8_t> stream{0x12U, 0x34U};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());

  const auto samples = parser.feed(stream);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples.front().timestamp_ms, 42U);
  EXPECT_EQ(parser.statistics().checksum_errors, 1U);
  EXPECT_GT(parser.statistics().discarded_bytes, 0U);
}

TEST(EbimuV5Protocol, DecodesAsciiStreamAcrossChunks)
{
  roka_imu_driver::EbimuV5Layout layout;
  layout.output_code = roka_imu_driver::EbimuOutputCode::kAscii;
  roka_imu_driver::EbimuV5Parser parser(layout);
  const std::string first{"*0,0,0,1,10,-20,"};
  const std::string second{"30,1,0,-1,1234\r\n"};

  EXPECT_TRUE(parser.feed(
      reinterpret_cast<const std::uint8_t *>(first.data()), first.size()).empty());
  const auto samples = parser.feed(
    reinterpret_cast<const std::uint8_t *>(second.data()), second.size());

  ASSERT_EQ(samples.size(), 1U);
  EXPECT_NEAR(
    samples.front().angular_velocity_rad_s[2],
    30.0 * roka_imu_driver::kPi / 180.0, 1.0e-12);
  EXPECT_NEAR(samples.front().linear_acceleration_m_s2[2], -9.80665, 1.0e-12);
  EXPECT_EQ(samples.front().timestamp_ms, 1234U);
}

TEST(EbimuV5Protocol, UnwrapsSixtySecondTimestamp)
{
  roka_imu_driver::EbimuTimestampUnwrapper unwrapper;
  EXPECT_EQ(unwrapper.unwrap(59990U), 59990U);
  EXPECT_EQ(unwrapper.unwrap(0U), 60000U);
  EXPECT_EQ(unwrapper.unwrap(10U), 60010U);
}

TEST(EbimuV5Protocol, InvertsMeasuredV5WorldToSensorQuaternion)
{
  const double half_sqrt_two = std::sqrt(0.5);
  const std::array<double, 4> clockwise_yaw{{
      0.0, 0.0, -half_sqrt_two, half_sqrt_two}};
  const auto converted = roka_imu_driver::convert_orientation_convention(
    clockwise_yaw, true);

  EXPECT_DOUBLE_EQ(converted[0], 0.0);
  EXPECT_DOUBLE_EQ(converted[1], 0.0);
  EXPECT_DOUBLE_EQ(converted[2], half_sqrt_two);
  EXPECT_DOUBLE_EQ(converted[3], half_sqrt_two);
}

TEST(EbimuV5Protocol, PreservesSensorToWorldQuaternion)
{
  const std::array<double, 4> quaternion{{0.1, -0.2, 0.3, 0.9}};
  EXPECT_EQ(
    roka_imu_driver::convert_orientation_convention(quaternion, false),
    quaternion);
}

}  // namespace

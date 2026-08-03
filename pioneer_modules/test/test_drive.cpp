// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "pioneer_modules/drive.hpp"

namespace pioneer_modules
{

// Exposes the protected pure conversion methods of Drive for testing, without requiring a full
// rclcpp_lifecycle node or a live ArRobot connection.
class DriveTestable : public Drive
{
public:
  using Drive::ariaToRosOdometry;
  using Drive::ariaToRosTf;
  using Drive::parseFrontBumperBits;
  using Drive::parseRearBumperBits;
  using Drive::clock_;
  using Drive::odom_frame_;
  using Drive::robot_base_frame_;
};

class DriveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    drive_.clock_ = std::make_shared<rclcpp::Clock>();
    drive_.odom_frame_ = "odom";
    drive_.robot_base_frame_ = "base_link";
  }

  DriveTestable drive_;
};

TEST_F(DriveTest, OdometryWritesAngularVelocityOnZAxis)
{
  ArPose pose(1000.0, 2000.0, 90.0);
  auto odom = drive_.ariaToRosOdometry(pose, 100.0, 0.0, 45.0);

  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, 45.0 * M_PI / 180.0);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.y, 0.0);
}

TEST_F(DriveTest, OdometryConvertsPositionAndFrames)
{
  ArPose pose(1000.0, 2000.0, 0.0);
  auto odom = drive_.ariaToRosOdometry(pose, 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, 2.0);
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "base_link");
}

TEST_F(DriveTest, OdometryConvertsLinearVelocityFromMillimetersToMeters)
{
  ArPose pose(0.0, 0.0, 0.0);
  auto odom = drive_.ariaToRosOdometry(pose, 500.0, -250.0, 0.0);

  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.y, -0.25);
}

TEST_F(DriveTest, TfUsesConfiguredFrames)
{
  ArPose pose(1000.0, 0.0, 0.0);
  auto tf_msg = drive_.ariaToRosTf(pose);

  EXPECT_EQ(tf_msg.header.frame_id, "odom");
  EXPECT_EQ(tf_msg.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.x, 1.0);
}

TEST(DriveBumperParsingTest, FrontBumperBitsSkipStallBit)
{
  // Bit 0 is the stall bit; bumper 0 is bit 1, bumper 1 is bit 2.
  unsigned char front_bumpers = 0b00000110;
  auto bits = DriveTestable::parseFrontBumperBits(front_bumpers, 5);

  ASSERT_EQ(bits.size(), 5u);
  EXPECT_TRUE(bits[0]);
  EXPECT_TRUE(bits[1]);
  EXPECT_FALSE(bits[2]);
  EXPECT_FALSE(bits[3]);
  EXPECT_FALSE(bits[4]);
}

TEST(DriveBumperParsingTest, FrontBumperBitsAllClear)
{
  auto bits = DriveTestable::parseFrontBumperBits(0b00000001, 5);
  for (bool bit : bits) {
    EXPECT_FALSE(bit);
  }
}

TEST(DriveBumperParsingTest, RearBumperBitsAreReversed)
{
  // With count = 5, bumper 0 is bit 5 (the leftmost, highest-order bit used).
  unsigned char rear_bumpers = 0b00100000;
  auto bits = DriveTestable::parseRearBumperBits(rear_bumpers, 5);

  ASSERT_EQ(bits.size(), 5u);
  EXPECT_TRUE(bits[0]);
  EXPECT_FALSE(bits[1]);
  EXPECT_FALSE(bits[2]);
  EXPECT_FALSE(bits[3]);
  EXPECT_FALSE(bits[4]);
}

}  // namespace pioneer_modules

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

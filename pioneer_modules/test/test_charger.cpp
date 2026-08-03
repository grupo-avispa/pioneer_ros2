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

#include "gtest/gtest.h"
#include "pioneer_modules/charger.hpp"

namespace pioneer_modules
{

// Exposes the protected pure mapping method of Charger for testing, without requiring a full
// rclcpp_lifecycle node or a live ArRobot connection.
class ChargerTestable : public Charger
{
public:
  using Charger::mapChargeStateToPowerSupplyStatus;
};

TEST(ChargerTest, ChargingWhenChargerPowerIsGood)
{
  auto status = ChargerTestable::mapChargeStateToPowerSupplyStatus(
    true, 0.5f, ArRobot::ChargeState::CHARGING_NOT);
  EXPECT_EQ(status, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

TEST(ChargerTest, FullWhenPercentageIsOneAndNotCharging)
{
  auto status = ChargerTestable::mapChargeStateToPowerSupplyStatus(
    false, 1.0f, ArRobot::ChargeState::CHARGING_NOT);
  EXPECT_EQ(status, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL);
}

TEST(ChargerTest, NotChargingWhenChargeStateIsNot)
{
  auto status = ChargerTestable::mapChargeStateToPowerSupplyStatus(
    false, 0.5f, ArRobot::ChargeState::CHARGING_NOT);
  EXPECT_EQ(status, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST(ChargerTest, UnknownWhenChargeStateIsUnknown)
{
  auto status = ChargerTestable::mapChargeStateToPowerSupplyStatus(
    false, 0.5f, ArRobot::ChargeState::CHARGING_UNKNOWN);
  EXPECT_EQ(status, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
}

}  // namespace pioneer_modules

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

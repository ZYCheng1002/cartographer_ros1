//
// Created by czy on 23-12-29.
//

#pragma once
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

struct WheelSpeedData {
  common::Time time;
  double wheelspeed_left;   /// 左轮速
  double wheelspeed_right;  /// 右轮速
  double wheelbase;         /// 轴距
};

// Converts 'wheel_speed data' to a proto::WheelSpeedData.
proto::WheelSpeedData ToProto(const WheelSpeedData& odometry_data);

// Converts 'proto' to an OdometryData.
WheelSpeedData FromProto(const proto::WheelSpeedData& proto);

}  // namespace sensor
}  // namespace cartographer

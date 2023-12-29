//
// Created by czy on 23-12-29.
//

#include "cartographer/sensor/wheel_data.h"

namespace cartographer {
namespace sensor {

proto::WheelSpeedData ToProto(const WheelSpeedData& odometry_data) {
  proto::WheelSpeedData proto;
  proto.set_timestamp(common::ToUniversal(odometry_data.time));
  proto.set_wheelspeed_l(odometry_data.wheelspeed_left);
  proto.set_wheelspeed_r(odometry_data.wheelspeed_right);
  proto.set_wheelbase(odometry_data.wheelbase);
  return proto;
}

// Converts 'proto' to an OdometryData.
WheelSpeedData FromProto(const proto::WheelSpeedData& proto) {
  return WheelSpeedData{common::FromUniversal(proto.timestamp()), proto.wheelspeed_l(), proto.wheelspeed_r(),
                        proto.wheelbase()};
}

}  // namespace sensor
}  // namespace cartographer
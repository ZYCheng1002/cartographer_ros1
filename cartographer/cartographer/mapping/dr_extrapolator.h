//
// Created by czy on 23-12-30.
//

#pragma once

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class DrExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit DrExtrapolator(common::Duration pose_queue_duration, double imu_gravity_time_constant,
                          bool static_init = false);

  DrExtrapolator(const DrExtrapolator&) = delete;
  DrExtrapolator& operator=(const DrExtrapolator&) = delete;

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  bool GetInitStatus() const override;

  ///@brief 增加CSM姿态结果
  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  void AddWheelData(const sensor::WheelSpeedData& wheelspeed_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  ///@brief 系统初始化
  void SystemStaticInit(const sensor::ImuData& imu_data);

  ///@brief 静态检查
  inline bool StaticCheck(double x, double y) { return fabs(x) <= 0.1 && fabs(y) <= 0.1; }

  void UpdateVelocitiesFromPoses();

  ///@brief imu列队处理
  void TrimImuData();

  ///@brief wheel列队处理
  void TrimWheelSpeedData();

  ///@brief odom列队处理
  void TrimOdometryData();

  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;

  ///@struct 带有time的pose🤪
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };

  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();   /// match计算的线速度
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();  /// match计算的角速度

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;                /// 跟踪CSM获取的姿态
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;       /// 跟踪odom获取的姿态
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;  /// 跟踪姿态递推器获取的姿态
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();   /// 轮速获得的线速度
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();  /// match获得的角速度

  std::deque<sensor::WheelSpeedData> wheelspeed_data_;  /// 轮速数据
  sensor::WheelSpeedData current_wheelspeed_;           /// 此时的轮速
  std::vector<sensor::ImuData> imu_init_vec_;           /// 用于imu初始化
  std::atomic<bool> init_success_{false};               /// 递推系统初始化成功标志位
  bool static_init_;                                    /// 是否进行静止初始化

  Eigen::Vector3d bias_gyro_ = Eigen::Vector3d::Zero();  /// 陀螺仪零偏
  Eigen::Vector3d acce_avr_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

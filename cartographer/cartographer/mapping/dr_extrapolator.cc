//
// Created by czy on 23-12-30.
//

#include "cartographer/mapping/dr_extrapolator.h"

#include <algorithm>
#include <fstream>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

DrExtrapolator::DrExtrapolator(const common::Duration pose_queue_duration, double imu_gravity_time_constant,
                                   bool static_init)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(), transform::Rigid3d::Identity()} {
  use_static_init_ = static_init;
   static_imu_init_ = StaticIMUInit();
   eskf_ = ESKFD();
   // timed_eskf_pose_buffer.SetSizeLimit(300);
}

DrExtrapolator::~DrExtrapolator() {
  std::string  path = "/home/idriver/data/carto_data/dr_result.txt";
  if (timed_eskf_pose_buffer.empty()) {
    return ;
  }
  std::ofstream file;
  file.open(path);
  if (!file) {
    return ;
  }
  file << std::fixed;
  auto save_result = [&file](const Vec3d& position) {
    file << position.x() << " " << position.y();
  };
  for (const auto& pose : eskf_state_deque_) {
    save_result(pose.p_);
    file << std::endl;
  }
  file.close();
}


common::Time DrExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

common::Time DrExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

bool DrExtrapolator::GetInitStatus() const { return imu_inited_; }

void DrExtrapolator::AddPose(const common::Time time, const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ = absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 && timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

void DrExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  if (!static_imu_init_.InitSuccess()) {
    static_imu_init_.AddImu(imu_data);
    return ;
  }
  /// 需要对imu进行初始化
  if (!imu_inited_) {
    ESKFD ::Options eskf_options;
    eskf_options.gyro_var_ = sqrt(static_imu_init_.GetCovGyro()[0]);
    eskf_options.acce_var_ = sqrt(static_imu_init_.GetCovAcce()[0]);
    eskf_.SetInitialConditions(eskf_options, static_imu_init_.GetInitBg(), static_imu_init_.GetInitBa(), static_imu_init_.GetGravity());
    imu_inited_ = true;
    return;
  }
  /// 预测
  eskf_.Predict(imu_data);
  auto state = eskf_.GetNominalState();
  eskf_state_deque_.push_back(state);
  transform::Rigid3d pose(state.p_, state.R_.unit_quaternion());
  common::Time time_current = common::NormalToTime(state.timestamp_);
  timed_eskf_pose_buffer.Push(imu_data.time, pose);
}

void DrExtrapolator::AddOdometryData(const sensor::OdometryData& odometry_data) {
  /// 暂时用不到
  }

void DrExtrapolator::AddWheelData(const sensor::WheelSpeedData& wheelspeed_data) {
  if (!imu_inited_) {
    static_imu_init_.AddWheelSpeed(wheelspeed_data);
    return ;
  }
  eskf_.ObserveWheelSpeed(wheelspeed_data);
}

transform::Rigid3d DrExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();  /// 获取最新的pose
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    std::unique_lock<std::mutex> lock(eskf_buffer_mutex);
    const Eigen::Vector3d translation = ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() * ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ = TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond DrExtrapolator::EstimateGravityOrientation(const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

void DrExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: " << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ = (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void DrExtrapolator::AdvanceImuTracker(const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                                                            : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it =
      std::lower_bound(imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
                       [](const sensor::ImuData& imu_data, const common::Time& time) { return imu_data.time < time; });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond DrExtrapolator::ExtrapolateRotation(const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d DrExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta = common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

DrExtrapolator::ExtrapolationResult DrExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity =
      odometry_data_.size() < 2 ? linear_velocity_from_poses_ : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()), current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer

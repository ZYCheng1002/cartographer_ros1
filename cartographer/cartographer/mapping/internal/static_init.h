//
// Created by czy on 24-1-5.
//

#pragma once
// #include "common/eigen_types.h"
// #include "common/imu.h"
// #include "common/odom.h"
#include <deque>
#include <numeric>

#include "cartographer/common/eigen_helper.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/wheel_data.h"

namespace cartographer {
namespace mapping {

class StaticIMUInit {
 public:
  struct Options {
    Options() {}
    double init_time_seconds_ = 10.0;            // 静止时间
    int init_imu_queue_max_size_ = 2000;         // 初始化IMU队列最大长度
    int static_odom_pulse_ = 5;                  // 静止时轮速计输出噪声
    double max_static_v_ = 0.05;                 // 静止时的轮速计速度噪声
    double max_static_gyro_var = 0.5;            // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;           // 静态下加计测量方差
    double gravity_norm_ = 9.81;                 // 重力大小
    bool use_speed_for_static_checking_ = true;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
  };

  /// 构造函数
  StaticIMUInit(Options options = Options()) : options_(options) {}

  /// 添加IMU数据
  bool AddImu(const sensor::ImuData& imu);
  /// 添加轮速数据
  bool AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed);

  /// 判定初始化是否成功
  bool InitSuccess() const { return init_success_; }

  /// 获取各Cov, bias, gravity
  Vec3d GetCovGyro() const { return cov_gyro_; }
  Vec3d GetCovAcce() const { return cov_acce_; }
  Vec3d GetInitBg() const { return init_bg_; }
  Vec3d GetInitBa() const { return init_ba_; }
  Vec3d GetGravity() const { return gravity_; }

 private:
  /// 尝试对系统初始化
  bool TryInit();

  template <typename C, typename D, typename Getter>
  void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
        mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&getter](const D &sum, const sensor::ImuData &data) -> D { return sum + getter(data); }) / len;
        cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                                   [&mean, &getter](const D &sum, const sensor::ImuData &data) -> D {
                                       return sum + (getter(data) - mean).cwiseAbs2().eval();
                                   }) / (len - 1);
    // clang-format on
  }

  Options options_;                             // 选项信息
  bool init_success_ = false;                   // 初始化是否成功
  Vec3d cov_gyro_ = Vec3d::Zero();              // 陀螺测量噪声协方差（初始化时评估）
  Vec3d cov_acce_ = Vec3d::Zero();              // 加计测量噪声协方差（初始化时评估）
  Vec3d init_bg_ = Vec3d::Zero();               // 陀螺初始零偏
  Vec3d init_ba_ = Vec3d::Zero();               // 加计初始零偏
  Vec3d gravity_ = Vec3d::Zero();               // 重力
  bool is_static_ = false;                      // 标志车辆是否静止
  std::deque<sensor::ImuData> init_imu_deque_;  // 初始化用的数据
  double current_time_ = 0.0;                   // 当前时间
  double init_start_time_ = 0.0;                // 静止的初始时间
};

}  // namespace mapping
}  // namespace cartographer
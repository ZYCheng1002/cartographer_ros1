//
// Created by czy on 24-1-5.
//

#pragma once
#include "cartographer/common/eigen_helper.h"
#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {

template <typename S=double>
class ESKF{
 public:
  struct Options {
    Options() = default;

    /// IMU 测量与零偏参数
    double imu_dt_ = 0.01;  // IMU测量间隔
    // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
    double gyro_var_ = 1e-5;       // 陀螺测量标准差
    double acce_var_ = 1e-2;       // 加计测量标准差
    double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
    double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

    /// 里程计参数
    double odom_var_ = 0.5;
    double odom_span_ = 0.1;        // 里程计测量间隔
    double wheel_radius_ = 0.155;   // 轮子半径
    double circle_pulse_ = 1024.0;  // 编码器每圈脉冲数

    /// 其他配置
    bool update_bias_gyro_ = true;  // 是否更新陀螺bias
    bool update_bias_acce_ = true;  // 是否更新加计bias
  };
};

}  // namespace mapping
}  // namespace cartographer

/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <limits>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace transform {

constexpr size_t kUnlimitedBufferSize = std::numeric_limits<size_t>::max();

// A time-ordered buffer of transforms that supports interpolated lookups.
// Unless explicitly set, the buffer size is unlimited.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  explicit TransformInterpolationBuffer(
      const mapping::proto::Trajectory& trajectory);

  // Sets the transform buffer size limit and removes old transforms
  // if it is exceeded.
  ///@brief 设置存储数据的大小
  void SetSizeLimit(size_t buffer_size_limit);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Clears the transform buffer.
  void Clear();

  // Returns true if an interpolated transform can be computed at 'time'.
  ///@brief 判断时间是否在范围内
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  ///@brief 根据时间查找T并进行插值(有必要的话)
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

  // Returns the maximum allowed size of the transform buffer.
  size_t size_limit() const;

  // Returns the current size of the transform buffer.
  size_t size() const;

  std::deque<TimestampedTransform> GetTimedPoseDeque() {
    return timestamped_transforms_;
  }

 private:
  void RemoveOldTransformsIfNeeded();

  std::deque<TimestampedTransform> timestamped_transforms_;  /// 存储T
  size_t buffer_size_limit_ = kUnlimitedBufferSize;  /// 存储的空间,不指定就是最大
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

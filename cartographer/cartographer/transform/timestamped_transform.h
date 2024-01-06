/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "cartographer/common/time.h"
#include "cartographer/transform/proto/timestamped_transform.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

///@struct 带有时间辍的T
struct TimestampedTransform {
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform FromProto(const proto::TimestampedTransform& proto);
proto::TimestampedTransform ToProto(const TimestampedTransform& transform);

///@brief T的插值
TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

///@brief 插值(找到对应时间点的pose之后进行插值)

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

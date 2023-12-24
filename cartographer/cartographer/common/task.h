/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_COMMON_TASK_H_
#define CARTOGRAPHER_COMMON_TASK_H_

#include <set>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace cartographer {
namespace common {

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;

  using WorkItem = std::function<void()>;
  ///@brief 热任务的状态
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  Task() = default;
  ~Task();

  ///@brief 返回任务状态
  State GetState() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'.
  ///@brief 设置任务
  void SetWorkItem(const WorkItem& work_item) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed.
  ///@brief 依赖任务
  void AddDependency(std::weak_ptr<Task> dependency) LOCKS_EXCLUDED(mutex_);

 private:
  // Allowed in all states.
  void AddDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  void Execute() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  void SetThreadPool(ThreadPoolInterface* thread_pool) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  WorkItem work_item_ GUARDED_BY(mutex_);                                    /// function
  ThreadPoolInterface* thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;  /// 依赖完成,线程触发标志位
  State state_ GUARDED_BY(mutex_) = NEW;                                     /// 任务状态
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;             /// 依赖的任务数
  std::set<Task*> dependent_tasks_ GUARDED_BY(mutex_);                       /// 其他的任务

  absl::Mutex mutex_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_

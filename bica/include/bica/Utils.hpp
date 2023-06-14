// Copyright 2019 Intelligent Robotics Lab
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

#ifndef BICA__UTILS_HPP_
#define BICA__UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

namespace bica {

/*
  这段代码看着就和 cyber_dog/rclcpp_lifecycle__demos 这个项目中的特别相似了
  [](D:\Document\Hirain\Project\rolling\ros2\demos\lifecycle\src\lifecycle_service_client.md)
  [](D:\Document\Hirain\cyberdog_ros2\cyberdog_common\cyberdog_utils\include\cyberdog_utils\simple_action_server.hpp)
  [](D:\Document\Hirain\Project\rolling\ros2\demos\lifecycle\src\lifecycle_service_client.cpp)
*/

/*
  此代码段是一个用于等待异步操作返回结果的函数。该函数接受两个参数：一个模板类型的 future
  对象和最长等待时间。如果在指定时间内 future 返回结果，则返回
  std::future_status::ready；如果超时，则返回
  std::future_status::timeout。在等待过程中，每隔一段时间检查一次 future 是否已经返回结果。如果
  rclcpp::ok() 返回 false，则退出循环并返回相应状态。
*/
/**
 * @brief 等待 future 返回结果的函数
 * @param future 一个模板类型的 future 对象，表示需要等待的异步操作
 * @param time_to_wait 表示最长等待时间
 * @details 如果 future 在指定时间内返回结果，则返回 std::future_status::ready。
 *          如果超时，则返回 std::future_status::timeout。
 *          如果 rclcpp::ok() 返回 false，则退出循环并返回相应状态。
 */
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;

  // 循环等待 future 返回结果
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    // 等待一段时间或者直到 future 返回结果
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

}  // namespace bica

#endif  // BICA__UTILS_HPP_

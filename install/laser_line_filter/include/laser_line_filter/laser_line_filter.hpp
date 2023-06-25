// Copyright 2019 Zhushi Tech, Inc.
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

#ifndef LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_
#define LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_

#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace laser_line_filter
{

using sensor_msgs::msg::PointCloud2;

/**
 * @brief List of parameter names.
 *
 */
const std::vector<std::string> KEYS = {"enable", "window_size", "gap", "deviate", "step", "length"};

/**
 * @brief Group parameters together.
 *
 */
struct Params
{
  bool enable = false;
  int window_size = 10;
  int gap = 5;
  double deviate = 5.;
  double step = 2.;
  int length = 30;
};

/**
 * @brief Moving average algorithm to filter out noise points.
 *
 */
class LaserLineFilter : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Laser Line Filter object.
   *
   * Initialize publisher.
   * Declare parameters before usage.
   * Create a thread for each worker.
   * Create a thread for manager.
   * Initialize subscription.
   * Initialize ROS parameter callback.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit LaserLineFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Laser Line Filter object.
   *
   * Release subscription.
   * Wake up all workers.
   * Wake up the manager.
   * Synchronize with all threads, wait for its return.
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~LaserLineFilter();

private:
  /**
   * @brief Declare parameters with defaults before usage.
   *
   */
  void _declare_parameters();

  /**
   * @brief Update parameters from ROS.
   *
   * @return Params Zipped parameters
   */
  Params _update_parameters();

  /**
   * @brief The worker works in seperate thread to process incoming date parallelly.
   *
   * Enter infinite loop.
   * Wait for incoming data.
   * Wake up to get a possible data, make a promise and notify the manager.
   * Continue to work on the data and return to sleep if no further data to process.
   */
  void _worker();

  /**
   * @brief The manager works in seperate thread to gather worker's results in order.
   *
   * Spin infinitely until rclcpp:ok() return false.
   * Whenever a future is ready, the manager wake up, get the result from the future and publish.
   */
  void _manager();

  /**
   * @brief Push a point cloud and notity workers.
   *
   * @param ptr Reference to a unique pointer to point clout to be moved.
   */
  void _push_back_point(PointCloud2::UniquePtr ptr);

  /**
   * @brief Promise a future so its future can be sychronized and notify the manager.
   *
   * @param f A future to point cloud msg.
   */
  void _push_back_future(std::future<PointCloud2::UniquePtr> fut);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/line_filtered";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/line";

  /**
   * @brief Shared pointer to subscription.
   *
   */
  rclcpp::Subscription<PointCloud2>::SharedPtr _sub;

  /**
   * @brief Number of co-workers.
   *
   */
  int _workers;

  /**
   * @brief Mutex to protect point queue.
   *
   */
  std::mutex _points_mut;

  /**
   * @brief Condition variable for point queue.
   *
   */
  std::condition_variable _points_con;

  /**
   * @brief Double end queue for points.
   *
   */
  std::deque<PointCloud2::UniquePtr> _points;

  /**
   * @brief Mutex to protect result queue.
   *
   */
  std::mutex _futures_mut;

  /**
   * @brief Condition variable for result queue.
   *
   */
  std::condition_variable _futures_con;

  /**
   * @brief Double end queue for results.
   *
   */
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;

  /**
   * @brief Callback handle for parameters.
   *
   */
  OnSetParametersCallbackHandle::SharedPtr _handle;
};

}  // namespace laser_line_filter

#endif  // LASER_LINE_FILTER__LASER_LINE_FILTER_HPP_

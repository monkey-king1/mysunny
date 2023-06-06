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

#ifndef LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_
#define LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_

#include <deque>
#include <string>
#include <vector>
#include <modbus.h>
#include <unistd.h>
#include "fileout/E2centerData.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tutorial_interfaces/msg/if_algorhmitmsg.hpp"
#include "tutorial_interfaces/msg/if_algorhmitcloud.hpp"

#define PIC_IMAGE_HEIGHT 2064
#define PIC_IMAGE_WIDTH  1544

namespace line_center_reconstruction
{

using sensor_msgs::msg::PointCloud2;
using tutorial_interfaces::msg::IfAlgorhmitmsg;
using tutorial_interfaces::msg::IfAlgorhmitcloud;

/**
 * @brief List of parameter names.
 *
 */
const std::vector<std::string> KEYS = {"homography_matrix"};
// const std::vector<std::string> KEYS = {"camera_matrix", "distort_coeffs", "homography_matrix"};

/**
 * @brief Group parameters together.
 *
 */
struct Params
{
  // std::vector<double> camera_matrix;
  // std::vector<double> distort_coeffs;
  std::vector<double> homography_matrix;
};

/**
 * @brief Homography transformation between two plane: image plane and laser plane.
 *
 */
class LineCenterReconstruction : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Line Center Reconstruction object.
   *
   * Initialize publisher.
   * Declare parameters before usage.
   * Get parameters.
   * Create a thread for each worker.
   * Create a thread for manager.
   * Initialize subscription.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit LineCenterReconstruction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Line Center Reconstruction object.
   *
   * Release subscription.
   * Wake up all workers.
   * Wake up the manager.
   * Synchronize with all threads, wait for its return.
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~LineCenterReconstruction();

private:
  /**
   * @brief Declare parameters with defaults before usage.
   *
   */
  void _declare_parameters();

  /**
   * @brief Get the parameters.
   *
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

  void _task100_199_worker();

  void _task100_199_manager();

  /**
   * @brief Push a point cloud and notity workers.
   *
   * @param ptr Reference to a unique pointer to point clout to be moved.
   */
  void _push_back_point(PointCloud2::UniquePtr ptr);

  void _push_back_point_task100_199(IfAlgorhmitmsg::UniquePtr ptr);

  /**
   * @brief Promise a future so its future can be sychronized and notify the manager.
   *
   * @param f A future to point cloud msg.
   */
  void _push_back_future(std::future<PointCloud2::UniquePtr> fut);

  void _push_back_future_task100_199(std::future<IfAlgorhmitcloud::UniquePtr> fut);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/pnts";

  const char * _pub_task100_199_name = "~/cloud_task100_199";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub;

  rclcpp::Publisher<IfAlgorhmitcloud>::SharedPtr _pub_task100_199;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/line";

  const char * _sub_task100_199_name = "~/image_task100_199";

  /**
   * @brief Shared pointer to subscription.
   *
   */
  rclcpp::Subscription<PointCloud2>::SharedPtr _sub;

  rclcpp::Subscription<IfAlgorhmitmsg>::SharedPtr _sub_task100_199;

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


  std::mutex _task100_199_mut;

  std::mutex _task100_199_futures_mut;

  std::condition_variable _task100_199_con;

  std::condition_variable _task100_199_futures_con;

  std::deque<IfAlgorhmitmsg::UniquePtr> _task100_199;

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

  std::deque<std::future<IfAlgorhmitcloud::UniquePtr>> _task100_199_futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;

  std::vector<std::thread> _task100_199_threads;

  PointCloud2::UniquePtr execute(PointCloud2::UniquePtr ptr, const Params & pm);

  IfAlgorhmitcloud::UniquePtr _task100_199_execute(IfAlgorhmitmsg::UniquePtr ptr, const Params & pm);

  //modbus sock
  modbus_t * ctx;
  std::thread _thread;
  void _modbus(int port);
  bool b_modbusconnect;

  std::shared_ptr<rclcpp::AsyncParametersClient> _param_camera_get;
  void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future);
  int picwidth;
  int picheight;

  E2proomData centerData;

  OnSetParametersCallbackHandle::SharedPtr _handle;

//OnSetParametersCallbackHandle::SharedPtr _handle;
//int _set_homography_matrix(std::vector<double> homography_matrix);

};

}  // namespace line_center_reconstruction

#endif  // LINE_CENTER_RECONSTRUCTION__LINE_CENTER_RECONSTRUCTION_HPP_

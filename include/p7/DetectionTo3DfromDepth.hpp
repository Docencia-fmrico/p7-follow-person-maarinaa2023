// Copyright 2023 Intelligent Robotics Lab
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

#ifndef DETECTIONTO3DFROMDEPTH_HPP_
#define DETECTIONTO3DFROMDEPTH_HPP_

#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/depth_traits.hpp"

namespace p7
{

class DetectionTo3DfromDepth : public rclcpp::Node
{
public:
  DetectionTo3DfromDepth();

private:
  void callback_sync(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg);

  void callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray> MySyncPolicy;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  std::shared_ptr<image_geometry::PinholeCameraModel> model_;
  
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attractive_vector_pub_;  // Publicador de Vector3Stamped
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  
  geometry_msgs::msg::Vector3Stamped attractive_vector_;  // Almacenará el vector atractivo

  bool camera_info_received_;
};

}  // namespace p7

#endif  // DETECTIONTO3DFROMDEPTH_HPP_
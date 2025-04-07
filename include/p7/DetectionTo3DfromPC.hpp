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

#ifndef DETECTIONTO3DFROMPC_HPP_
#define DETECTIONTO3DFROMPC_HPP_

#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

namespace p7
{

class DetectionTo3DfromPC : public rclcpp::Node
{
public:
  DetectionTo3DfromPC();

private:
  void callback_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray> MySyncPolicy;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc2_sub_;
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attractive_vector_pub_; 
  
  geometry_msgs::msg::Vector3Stamped attractive_vector_;
};

}  // namespace p7

#endif  // DETECTIONTO3DFROMPC_HPP_

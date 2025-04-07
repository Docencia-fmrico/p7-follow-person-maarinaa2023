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

#include <memory>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "p7/DetectionTo3DfromDepth.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/depth_traits.hpp"

#include "rclcpp/rclcpp.hpp"

namespace p7
{

using std::placeholders::_1;
using std::placeholders::_2;

DetectionTo3DfromDepth::DetectionTo3DfromDepth()
: Node("detection_to_3d_from_depth_node"), camera_info_received_(false)
{
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "input_depth", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  detection_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(
    this, "input_detection_2d", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(5), *depth_sub_, *detection_sub_);
  sync_->registerCallback(std::bind(&DetectionTo3DfromDepth::callback_sync, this, _1, _2));

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", 1, std::bind(&DetectionTo3DfromDepth::callback_info, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS().reliable());

  attractive_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "attractive_vector", rclcpp::SensorDataQoS().reliable());
}

void
DetectionTo3DfromDepth::callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  if (camera_info_received_) return;
  camera_info_received_ = true;

  RCLCPP_INFO(get_logger(), "Camera info received");
  model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  model_->fromCameraInfo(*msg);
}

void DetectionTo3DfromDepth::callback_sync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN(get_logger(), "Camera Model not yet available");
    return;
  }

  if (image_msg->encoding != "16UC1" && image_msg->encoding != "32FC1") {
    RCLCPP_ERROR(get_logger(), "The image type has no depth info");
    return;
  }

  vision_msgs::msg::Detection3DArray detections_3d_msg;
  detections_3d_msg.header = detection_msg->header;

  try {
    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(*image_msg, image_msg->encoding);
    for (const auto & detection : detection_msg->detections) {
      vision_msgs::msg::Detection3D detection_3d_msg;
      detection_3d_msg.header = detection_msg->header;
      detection_3d_msg.results = detection.results;

      float depth = (image_msg->encoding == "16UC1") ?
        depth_image_proc::DepthTraits<uint16_t>::toMeters(
          cv_depth_ptr->image.at<uint16_t>(cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y))) :
        cv_depth_ptr->image.at<float>(cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y));

      if (std::isnan(depth)) continue;

      cv::Point3d ray = model_->projectPixelTo3dRay(
        model_->rectifyPoint(cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y)));
      ray = ray / ray.z;
      cv::Point3d point = ray * depth;

      detection_3d_msg.bbox.center.position.x = point.x;
      detection_3d_msg.bbox.center.position.y = point.y;
      detection_3d_msg.bbox.center.position.z = point.z;

      detections_3d_msg.detections.push_back(detection_3d_msg);

      attractive_vector_.header.stamp = detection_msg->header.stamp;
      attractive_vector_.header.frame_id = detection_msg->header.frame_id;
      
      attractive_vector_.vector.x = point.x;
      attractive_vector_.vector.y = point.y;
      attractive_vector_.vector.z = point.z;

      RCLCPP_INFO(get_logger(), "Publicando attractive_vector: [%.2f, %.2f, %.2f]",
            point.x, point.y, point.z);

      attractive_vector_pub_->publish(attractive_vector_);
    }
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (!detections_3d_msg.detections.empty()) {
    RCLCPP_INFO(get_logger(), "Publishing 3D detections: %ld objects", detections_3d_msg.detections.size());
    detection_pub_->publish(detections_3d_msg);
  }
}

}  // namespace p7
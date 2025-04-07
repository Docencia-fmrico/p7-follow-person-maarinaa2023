// Copyright 2024 Intelligent Robotics Lab
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

#include "p7/ObstacleDetector.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace p7
{

using std::placeholders::_1;

ObstacleDetector::ObstacleDetector()
: Node("obstacle_detector_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetector set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetector::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle_detection", 100);

  repulsive_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("repulsive_vector", 100);
}

void
ObstacleDetector::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  if (distance_min < min_distance_) {
    float angle = scan->angle_min + scan->angle_increment * min_idx;
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    RCLCPP_INFO(get_logger(), "Obstacle in (%f, %f)", distance_min, angle);

    repulsive_vector_.header.stamp = scan->header.stamp;
    repulsive_vector_.header.frame_id = scan->header.frame_id;

    // Calculando el vector repulsivo
    repulsive_vector_.vector.x = - std::cos(angle) * (1.0 / distance_min);  // Fuerza repulsiva en X
    repulsive_vector_.vector.y = - std::sin(angle) * (1.0 / distance_min);  // Fuerza repulsiva en Y
    repulsive_vector_.vector.z = 0.0;  // En 2D (sin movimiento en Z)

    RCLCPP_INFO(
      get_logger(), "Vector repulsivo publicado: [%f, %f, %f]", 
      repulsive_vector_.vector.x, repulsive_vector_.vector.y, repulsive_vector_.vector.z);
    
      repulsive_vector_pub_->publish(repulsive_vector_);

  } else {
    RCLCPP_INFO(get_logger(), "No se detectaron obstáculos cercanos.");
  }

}

}  // namespace p7

// Copyright 2021 Intelligent Robotics Lab
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

#ifndef CONTROLNODE_HPP_
#define CONTROLNODE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "p7/PIDController.hpp"
#include "rclcpp/rclcpp.hpp"

namespace p7
{

enum State {IDLE, TRACKING};


class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  
  void attractive_callback(const geometry_msgs::msg::Vector3Stamped msg);
  void repulsive_callback(const geometry_msgs::msg::Vector3Stamped msg);

  void control_cycle();

private:
  State state_;
  p7::PIDController pid_lin_;  // Para la velocidad lineal
  p7::PIDController pid_ang_;  // Para la velocidad angular

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attractive_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr repulsive_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Vector3Stamped attractive_vector_;
  geometry_msgs::msg::Vector3Stamped repulsive_vector_;

  rclcpp::Time last_attractive_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transformadas iniciales (solo se obtienen una vez)
  tf2::Stamped<tf2::Transform> bf2camera_initial;
  tf2::Stamped<tf2::Transform> camera2target_initial;

  bool target_saved_;
  bool initial_saved_;
};

}  // namespace p7

#endif  // CONTROLNODE_HPP_

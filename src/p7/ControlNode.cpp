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

#include <memory>
#include <utility>
#include <algorithm>
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

#include "p7/ControlNode.hpp"
#include "p7/PIDController.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace p7
{

ControlNode::ControlNode()
: Node("control_node"), state_(IDLE),
  pid_lin_(0.05, 1.0, 0.05, 0.3),
  pid_ang_(0.05, 1.0, 0.05, 0.2),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  target_saved_ (false),
  initial_saved_(false)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);

  attractive_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "attractive_vector", 10, std::bind(&ControlNode::attractive_callback, this, _1));

  repulsive_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "repulsive_vector", 10, std::bind(&ControlNode::repulsive_callback, this, _1));

  timer_ = create_wall_timer(50ms, std::bind(&ControlNode::control_cycle, this));

  last_attractive_time_ = this->now();

  // Velocidad lineal: entre 0.05 y 1.0, salida entre 0.05 y 0.3 m/s
  // Velocidad angular: salida entre 0.05 y 0.2 rad/s

}

void ControlNode::attractive_callback(const geometry_msgs::msg::Vector3Stamped msg)
{
  if (std::isnan(msg.vector.x) || std::isnan(msg.vector.y)) {
    RCLCPP_WARN(get_logger(), "Vector recibido contiene valores inválidos.");
    return;
  } 

  attractive_vector_ = msg;
  last_attractive_time_ = this->now();

  RCLCPP_INFO(get_logger(), "Vector atractivo recibido: [%f, %f, %f]", msg.vector.x, msg.vector.y, msg.vector.z);
}

void ControlNode::repulsive_callback(const geometry_msgs::msg::Vector3Stamped msg)
{
  if (std::isnan(msg.vector.x) || std::isnan(msg.vector.y)) {
    RCLCPP_WARN(get_logger(), "Vector recibido contiene valores inválidos.");
    return;
  }

  repulsive_vector_ = msg;
  RCLCPP_INFO(get_logger(), "Vector repulsivo recibido: [%f, %f, %f]", msg.vector.x, msg.vector.y, msg.vector.z);
}

void
ControlNode::control_cycle()
{
  geometry_msgs::msg::Twist vel;

  double obstacle_distance = sqrt(
    repulsive_vector_.vector.x * repulsive_vector_.vector.x + 
    repulsive_vector_.vector.y * repulsive_vector_.vector.y
  );
  
  bool danger_close = obstacle_distance > 1.45; // Si la fuerza es muy grande, el obstáculo está muy cerca
  
  tf2::Stamped<tf2::Transform> bf2camera_actual;
  tf2::Stamped<tf2::Transform> camera2target_actual;
  std::string error;

  tf2::Transform bf2camera_relative;

  // Obtener la transformación de "base_footprint" a "camera"
  if (tf_buffer_.canTransform("base_footprint", "camera", tf2::TimePointZero, &error)) {
    auto bf2camera_msg = tf_buffer_.lookupTransform(
      "base_footprint", "camera", tf2::TimePointZero);
    tf2::fromMsg(bf2camera_msg, bf2camera_actual);

    // Guardar la transformación inicial solo una vez
    if (!initial_saved_) {
      bf2camera_initial = bf2camera_actual;
      initial_saved_ = true;
    }

    // Obtener la transformación relativa (base_footprint -> camera)
    tf2::Transform bf2camera_relative = bf2camera_initial.inverse() * bf2camera_actual;
  }

  // Obtener la transformación de "camera" a "target" (humano)
  if (tf_buffer_.canTransform("camera", "target", tf2::TimePointZero, &error)) {
    auto camera2target_msg = tf_buffer_.lookupTransform(
      "camera", "target", tf2::TimePointZero);
    tf2::fromMsg(camera2target_msg, camera2target_actual);

    // Guardar la transformación inicial solo una vez
    if (!target_saved_) {
      camera2target_initial = camera2target_actual;
      target_saved_ = true;
    }

    // Obtener la transformación relativa (camera -> target)
    tf2::Transform camera2target_relative = camera2target_initial.inverse() * camera2target_actual;

    // Componer las transformaciones para obtener "base_footprint -> target"
    tf2::Transform basefootprint2target = bf2camera_relative * camera2target_relative;
  }

  switch (state_) {
    case IDLE: {
      RCLCPP_INFO(get_logger(), "Estado: IDLE. Buscando objetivo...");

      if (danger_close) {
        RCLCPP_WARN(get_logger(), "¡Obstáculo demasiado cerca! Retrocediendo.");
        vel.linear.x = -0.2; // Retrocede lento
        vel.angular.z = 0.1;  
        vel_pub_->publish(vel);
        return;
      }

      // Configura el movimiento de búsqueda (gira sobre sí mismo)
      vel.linear.x = 0.0;
      vel.angular.z = 0.4;  // Gira a velocidad angular constante
      vel_pub_->publish(vel);

      // Condición para cambiar a TRACKING: vector atractivo detectado
      if ((attractive_vector_.vector.x != 0.0) || (attractive_vector_.vector.y != 0.0)) {
        state_ = TRACKING;
        RCLCPP_INFO(get_logger(), "Cambio de estado a TRACKING. Objetivo detectado.");
      }
      break;
    }
    
    case TRACKING: {
      RCLCPP_INFO(get_logger(), "Estado: TRACKING. Siguiendo objetivo...");

      if (danger_close) {
        RCLCPP_WARN(get_logger(), "¡Obstáculo demasiado cerca! Retrocediendo.");
        vel.linear.x = -0.2; // Retrocede lento
        vel.angular.z = 0.0;  // No giramos para no desestabilizar
        vel_pub_->publish(vel);
        return;
      }

      // Use result vector to calculate output speed
      double x = attractive_vector_.vector.x + repulsive_vector_.vector.x;
      double y = attractive_vector_.vector.y + repulsive_vector_.vector.y;

      double angle = atan2(x, y);
      double dist = sqrt(x * x + y * y);

      // ZONA MUERTA para evitar que avance al llegar
      if (std::abs(dist - 1.0) < 0.1) {  // por ejemplo ±10 cm
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
      } else {
        vel.linear.x = std::clamp(pid_lin_.get_output(dist - 1.0), -0.2, 0.3); // truncate linear vel to [0.0, 0.3] m/s
        vel.angular.z = std::clamp(pid_ang_.get_output(angle), -0.2, 0.2);  // truncate rotation vel to [-0.5, 0.5] rad/s
      }

      vel_pub_->publish(vel);
      
      // Condición para regresar a IDLE: 
      rclcpp::Duration time_since_last = this->now() - last_attractive_time_;
      if (time_since_last.seconds() > 3.0) {
        state_ = IDLE;

        // Invalidamos el vector atractivo anteriormente recibido, hasta obtener uno nuevo
        attractive_vector_.vector.x = 0.0;
        attractive_vector_.vector.y = 0.0;
        attractive_vector_.vector.z = 0.0;

        RCLCPP_WARN(get_logger(), "Cambio de estado a IDLE. Sin vector atractivo por más de 5 segundos.");
      }      
      break;
    }   
  }
}
  
}  // namespace p7

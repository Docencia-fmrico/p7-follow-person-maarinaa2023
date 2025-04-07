cmake_minimum_required(VERSION 3.8)
project(p7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  vision_msgs
  image_transport
  visualization_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  image_geometry
  depth_image_proc
  message_filters
  pcl_conversions
  pcl_ros
  yolo_msgs
  tf2
  tf2_ros
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/p7/YoloDetection.cpp
  src/p7/DetectionTo3DfromPC.cpp
  src/p7/DetectionTo3DfromDepth.cpp
  src/p7/ObstacleDetector.cpp
  src/p7/ControlNode.cpp
  src/p7/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Compilar el nodo de detección del target
add_executable(yolo_detection_main src/yolo_detection_main.cpp)
ament_target_dependencies(yolo_detection_main ${dependencies})
target_link_libraries(yolo_detection_main ${PROJECT_NAME})

# Compilar el nodo de detección del target, nos ayuda a obtener la coord. z, mediante la profundidad
add_executable(detection_2d_to_3d_pc2_main src/detection_2d_to_3d_pc2_main.cpp)
ament_target_dependencies(detection_2d_to_3d_pc2_main ${dependencies})
target_link_libraries(detection_2d_to_3d_pc2_main ${PROJECT_NAME})

add_executable(detection_2d_to_3d_depth_main src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(detection_2d_to_3d_depth_main ${dependencies})
target_link_libraries(detection_2d_to_3d_depth_main ${PROJECT_NAME})

# Compilar el nodo de detección del obstáculo
add_executable(obstacle_detector_main src/obstacle_detector_main.cpp)
ament_target_dependencies(obstacle_detector_main ${dependencies})
target_link_libraries(obstacle_detector_main ${PROJECT_NAME})

# Compilar el nodo de control
add_executable(control_node_main src/control_node_main.cpp)
ament_target_dependencies(control_node_main ${dependencies})
target_link_libraries(control_node_main ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  yolo_detection_main
  detection_2d_to_3d_pc2_main
  obstacle_detector_main
  control_node_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

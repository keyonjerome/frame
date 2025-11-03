cmake_minimum_required(VERSION 3.8)
project(ir_to_rgb_remap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(ir_to_rgb_remap_component SHARED src/ir_to_rgb_remap_component.cpp)
target_link_libraries(ir_to_rgb_remap_component ${OpenCV_LIBS})
ament_target_dependencies(ir_to_rgb_remap_component rclcpp rclcpp_components sensor_msgs cv_bridge)

rclcpp_components_register_nodes(ir_to_rgb_remap_component "ir_to_rgb_remap::IrToRgbRemapNode")

install(TARGETS ir_to_rgb_remap_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ir_to_rgb_remap_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()


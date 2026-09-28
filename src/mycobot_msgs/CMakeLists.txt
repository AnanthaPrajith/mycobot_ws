cmake_minimum_required(VERSION 3.5)
project(mycobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add your new message here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointVelocity.msg"
  "msg/DetectedObject.msg"   # <--- added
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

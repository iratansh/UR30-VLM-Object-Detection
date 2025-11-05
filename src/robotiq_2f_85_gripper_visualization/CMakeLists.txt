cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_85_gripper_visualization)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

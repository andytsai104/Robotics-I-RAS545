# cmake_minimum_required(VERSION 3.16)
# cmake_policy(SET CMP0057 NEW)

# project(dobot_magician_lite)

# find_package(ament_cmake REQUIRED COMPONENTS
# 	message_generation
# 	roscpp
# 	rospy
# 	std_msgs
# 	geometry_msgs
# 	urdf
# 	xacro
# 	message_generation
# )

# ament_package(
# 	CATKIN_DEPENDS
# 	geometry_msgs
# 	roscpp
# 	rospy
# 	std_msgs
# )


# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)


cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0057 NEW)

project(dobot_magician_lite)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Export dependencies (must be before ament_package())
ament_export_dependencies(rclcpp rclpy std_msgs geometry_msgs urdf xacro)

# Define the package
ament_package()

# Install directories (corrected for ROS 2)
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)
cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
# 仅保留消息定义（FAST-LIO 编译依赖 CustomMsg），不包含 Livox 雷达驱动本体
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)
add_message_files(FILES CustomPoint.msg CustomMsg.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

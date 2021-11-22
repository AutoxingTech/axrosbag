cmake_minimum_required(VERSION 3.0)
project(axrosbag_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation rosgraph_msgs)

add_service_files(FILES TriggerRecord.srv)

generate_messages(DEPENDENCIES rosgraph_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime rosgraph_msgs
)
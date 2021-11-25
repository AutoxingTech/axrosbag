cmake_minimum_required(VERSION 3.0)
project(axrosbag)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -W -Wno-maybe-uninitialized")

find_package(catkin REQUIRED COMPONENTS rosbag roscpp std_srvs topic_tools message_generation)

add_service_files(FILES TriggerRecord.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS include
    CATKIN_DEPENDS rosbag roscpp std_srvs topic_tools message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main_record.cpp src/recorder.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

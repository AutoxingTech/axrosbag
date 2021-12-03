cmake_minimum_required(VERSION 3.0)
project(axrosbag)

set (CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall)
add_compile_options(-Werror=unused)
add_compile_options(-Werror=address)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=uninitialized)
add_compile_options(-Werror=unused-parameter)
add_compile_options(-Werror=dangling-else)
add_compile_options(-Wsign-compare)
add_compile_options(-Wno-reorder)
add_compile_options(-fno-operator-names)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Werror=unused-private-field)
  add_compile_options(-Wthread-safety)
endif()

find_package(catkin REQUIRED COMPONENTS rosbag roscpp std_srvs topic_tools message_generation)

add_service_files(FILES TriggerRecord.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS rosbag roscpp std_srvs topic_tools message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} 
  src/main_record.cpp
  src/nc_argparse.cpp
  src/write_command.cpp
  src/daemon_command.cpp
  src/command_base.cpp
  src/bag_writer.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

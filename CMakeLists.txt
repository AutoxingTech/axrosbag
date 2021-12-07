cmake_minimum_required(VERSION 3.0)
project(axrosbag_ros)

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

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")

find_package(catkin REQUIRED COMPONENTS rosbag roscpp std_srvs topic_tools message_generation)

add_service_files(FILES TriggerRecord.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS "."
  CATKIN_DEPENDS rosbag roscpp std_srvs topic_tools message_runtime
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h")
file(GLOB_RECURSE ALL_EXECUTABLES "src/main_*.cpp")
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
add_subdirectory("src")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

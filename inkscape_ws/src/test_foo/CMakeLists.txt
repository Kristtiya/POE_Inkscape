cmake_minimum_required(VERSION 2.8.3)
project(test_foo)
add_compile_options(-std=c++11)

set(CATKIN_DEPS rospy roscpp std_msgs)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${CATKIN_DEPS})

add_executable(speed_pub src/dummy/speed_pub.cpp)
target_link_libraries(speed_pub ${catkin_LIBRARIES})
add_dependencies(speed_pub ${test_foo_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS speed_pub
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

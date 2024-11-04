cmake_minimum_required(VERSION 3.8)
project(TODO_PACKAGE_NAME)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Messages TODO_EXTRA
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# OpenCV TODO_EXTRA
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

# Add source for node executable (link non-ros dependencies here)
add_executable(TODO_PACKAGE_NAME src/TODO_NODE_NAME.cpp src/TODO_NODE_NAME_node.cpp)
target_include_directories(TODO_PACKAGE_NAME PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(TODO_PACKAGE_NAME PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make ros deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        # Messages TODO_EXTRA
        ackermann_msgs
        sensor_msgs
        std_msgs
        # OpenCv TODO_EXTRA
        cv_bridge
        OpenCV
        )

# Link ros dependencies
ament_target_dependencies(
        TODO_PACKAGE_NAME
        ${dependencies}
)

install(TARGETS TODO_PACKAGE_NAME
        DESTINATION lib/${PROJECT_NAME})

# Uncomment below to make launch files available if created
#install(
#        DIRECTORY launch config
#        DESTINATION share/${PROJECT_NAME}/
#)

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            # Remember to add node source files
            src/TODO_NODE_NAME_node.cpp
            )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
endif ()

ament_package()

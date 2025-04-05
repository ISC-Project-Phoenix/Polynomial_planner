cmake_minimum_required(VERSION 3.8)
project(polynomial_planner)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
# Messages TODO_EXTRA
# find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# OpenCV TODO_EXTRA
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)


# Node 2
add_executable(polynomial_planner_ai src/PolynomialPlannerAi.cpp src/PolynomialPlannerAi_node.cpp)
target_include_directories(polynomial_planner_ai PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(polynomial_planner_ai PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make ros deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        # Messages TODO_EXTRA
        # ackermann_msgs
        sensor_msgs
        std_msgs
        nav_msgs
        geometry_msgs
        tf2_geometry_msgs
        # OpenCv TODO_EXTRA
        cv_bridge
        OpenCV
        # for camera space transformation
        image_geometry
)

# Link ros dependencies


# Link ros dependencies
ament_target_dependencies(
        polynomial_planner_ai
        ${dependencies}
)


install(TARGETS polynomial_planner_ai
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

            src/PolynomialPlannerAi_node.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
endif ()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(skid_steer_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/encoder_emulator.py
  scripts/wheel_odometry.py
  scripts/path_publisher.py
  scripts/calibrate.py
  scripts/umbmark_test.py
  scripts/plot_results.py
  scripts/part6_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
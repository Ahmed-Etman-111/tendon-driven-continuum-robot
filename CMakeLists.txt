cmake_minimum_required(VERSION 3.8)
project(tendon_continuum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(scripts)

# Install all simulation directories
install(DIRECTORY
  launch
  urdf
  worlds
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executable nodes
install(PROGRAMS
  scripts/kinematics_solver.py
  scripts/tendon_controller.py
  scripts/dashboard_gui.py
  scripts/rod_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
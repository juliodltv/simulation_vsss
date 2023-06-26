cmake_minimum_required(VERSION 3.0.2)
project(simulation_vsss)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
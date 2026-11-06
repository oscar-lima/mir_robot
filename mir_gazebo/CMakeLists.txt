cmake_minimum_required(VERSION 3.5.1)
project(mir_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/fake_mir_robot_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  maps
  sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)

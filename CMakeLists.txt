cmake_minimum_required(VERSION 3.0.2)
project(senseless_robot_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    nodes/rendezvous_sm_node
    nodes/true_pose_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch nodes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

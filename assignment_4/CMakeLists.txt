cmake_minimum_required(VERSION 2.8.3)
project(assignment_4)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
  
cs_add_executable(action_server_node src/action_server.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(action_client_node src/action_client.cpp)
cs_add_executable(lidar_node src/lidar_alarm.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(ros_library_test_main example_ros_library)

cs_install()
cs_export()

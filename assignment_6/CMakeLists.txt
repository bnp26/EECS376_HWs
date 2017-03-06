cmake_minimum_required(VERSION 2.8.3)
project(assignment_6)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(traj_builder src/traj_builder.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(traj_builder_example_main src/traj_builder_example_main.cpp)

cs_add_executable(mobot_pub_des_state src/pub_des_state_main.cpp src/pub_des_state.cpp)
cs_add_executable(mobot_pub_des_state_startup src/pub_des_state_startup_main.cpp src/pub_des_state.cpp)

cs_add_executable(open_loop_controller src/open_loop_controller.cpp)

cs_add_executable(pub_des_state_path_client src/pub_des_state_path_client.cpp)
cs_add_executable(pub_des_state_path_client_3x3 src/pub_des_state_path_client_3mx3m_sqr.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(traj_builder_example_main traj_builder)

cs_install()
cs_export()
    
cmake_minimum_required(VERSION 2.8.3)
project(assignment_8)

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
#find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(${PROJECT_NAME}_w_amcl_headers
    include/${PROJECT_NAME}/odom_tf.h
    include/${PROJECT_NAME}/steering_algorithm.h
)

set(${PROJECT_NAME}_w_amcl_srcs
    src/odom_tf.cc
    src/lin_steering_wrt_odom.cpp
)

cs_add_executable(linear_steering_w_amcl_controller
    ${${PROJECT_NAME}_w_amcl_headers}
    ${${PROJECT_NAME}_w_amcl_srcs}
)


# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(traj_builder_lib_8 src/traj_builder.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(traj_builder_example_main_node_8 src/traj_builder_example_main.cpp)

cs_add_executable(mobot_pub_des_state_node_8 src/pub_des_state_main.cpp src/pub_des_state.cpp)
cs_add_executable(mobot_pub_des_state_startup_node_8 src/pub_des_state_startup_main.cpp src/pub_des_state.cpp)

cs_add_executable(open_loop_controller_node_8 src/open_loop_controller.cpp)
cs_add_executable(lin_steering_node_8 src/lin_steering_wrt_odom.cpp)

cs_add_executable(pub_des_state_path_client_node_8 src/pub_des_state_path_client.cpp)
cs_add_executable(pub_des_state_path_client_3x3_node_8 src/pub_des_state_path_client_3mx3m_sqr.cpp)


target_link_libraries(mobot_pub_des_state_node_8 traj_builder_lib_8)
target_link_libraries(mobot_pub_des_state_startup_node_8 traj_builder_lib_8)
target_link_libraries(open_loop_controller_node_8 traj_builder_lib_8)
target_link_libraries(lin_steering_node_8 traj_builder_lib_8)
target_link_libraries(pub_des_state_path_client_node_8 traj_builder_lib_8)
target_link_libraries(pub_des_state_path_client_3x3_node_8 traj_builder_lib_8)

target_link_libraries(traj_builder_example_main_node_8 traj_builder_lib_8)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package

cs_install()
cs_export()
    

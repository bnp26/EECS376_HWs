cmake_minimum_required(VERSION 2.8.3)
project(assignment_1)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
#cs_add_library(bnp26_stdr_library src/minimal_controller.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(minimal_controller_node src/minimal_controller.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(bnp26_stdr_control_node)

cs_install()
cs_export()


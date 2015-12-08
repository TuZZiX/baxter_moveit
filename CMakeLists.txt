cmake_minimum_required(VERSION 2.8.3)
project(baxter_moveit)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
 find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
cs_add_executable(move_group_interface_tutorial_baxter src/move_group_interface_tutorial_baxter.cpp)
target_link_libraries(move_group_interface_tutorial_baxter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial_baxter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_add_executable(baxter_moveit_test src/baxter_moveit_test.cpp)
target_link_libraries(baxter_moveit_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS baxter_moveit_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_install()
cs_export()
    

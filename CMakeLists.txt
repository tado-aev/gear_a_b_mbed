cmake_minimum_required(VERSION 2.8.3)
project(gear_a_b_mbed)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    rosserial_mbed
    coms_msgs
    )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gear_a_b_mbed
    CATKIN_DEPENDS rosserial_mbed coms_msgs
    )

add_custom_target(gear_a_b_mbed
    COMMAND make deploy
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

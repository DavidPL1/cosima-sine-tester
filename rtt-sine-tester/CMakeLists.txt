cmake_minimum_required(VERSION 2.8.3)
project(sine-component)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(gazebo REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(srdfdom_advr REQUIRED)
find_package(XBotCoreModel REQUIRED)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

option(DISABLE_INTROSPECTION "Disable introspection features" TRUE)

if (DISABLE_INTROSPECTION)
    message(STATUS "###################################################")
    message(STATUS "### RTT-CORE-EXTENSIONS Introspection disabled! ###")
    message(STATUS "###################################################")
else()
    find_package(RTT-CORE-EXTENSIONS 0.2)
    if (RTT-CORE-EXTENSIONS_FOUND)
        message(STATUS "RTT-CORE-EXTENSIONS_LIBRARIES: ${RTT-CORE-EXTENSIONS_LIBRARIES}")
        message(STATUS "RTT-CORE-EXTENSIONS_LIBRARY_DIRS: ${RTT-CORE-EXTENSIONS_LIBRARY_DIRS}")
        message(STATUS "RTT-CORE-EXTENSIONS_INCLUDE_DIRS: ${RTT-CORE-EXTENSIONS_INCLUDE_DIRS}")
        message(STATUS "##################################################")
        message(STATUS "### RTT-CORE-EXTENSIONS Introspection enabled! ###")
        message(STATUS "##################################################")
        set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS USE_INTROSPECTION)
    else()
        message(STATUS "###################################################")
        message(STATUS "### RTT-CORE-EXTENSIONS Introspection disabled! ###")
        message(STATUS "###################################################")
    endif()
endif()

link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${RST-RT_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
    ${srdfdom_advr_INCLUDE_DIRS}
    ${XBotCoreModel_INCLUDE_DIRS}
    ${RTT-CORE-EXTENSIONS_INCLUDE_DIRS}
)

file(GLOB_RECURSE rtt_gazebo_robot_sim_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

#orocos_component(senderComponent src/senderComponent.cpp ${rtt_gazebo_robot_sim_INCLUDES})
orocos_component(${PROJECT_NAME} src/SineTester.cpp ${rtt_gazebo_robot_sim_INCLUDES})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
   ${RTT-CORE-EXTENSIONS_LIBRARIES}
)

orocos_generate_package(INCLUDE_DIRS include)
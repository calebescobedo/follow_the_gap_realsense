# Install script for directory: /home/odroid/ar_workspace/src/ros_pololu_servo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/odroid/ar_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/msg" TYPE FILE FILES
    "/home/odroid/ar_workspace/src/ros_pololu_servo/msg/MotorCalibration.msg"
    "/home/odroid/ar_workspace/src/ros_pololu_servo/msg/MotorCommand.msg"
    "/home/odroid/ar_workspace/src/ros_pololu_servo/msg/MotorState.msg"
    "/home/odroid/ar_workspace/src/ros_pololu_servo/msg/MotorStateList.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/srv" TYPE FILE FILES "/home/odroid/ar_workspace/src/ros_pololu_servo/srv/MotorRange.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/action" TYPE FILE FILES "/home/odroid/ar_workspace/src/ros_pololu_servo/action/pololu_trajectory.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/msg" TYPE FILE FILES
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryAction.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryActionGoal.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryActionResult.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryActionFeedback.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryGoal.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryResult.msg"
    "/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg/pololu_trajectoryFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/cmake" TYPE FILE FILES "/home/odroid/ar_workspace/build/ros_pololu_servo/catkin_generated/installspace/ros_pololu_servo-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/odroid/ar_workspace/devel/include/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/odroid/ar_workspace/devel/share/roseus/ros/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/odroid/ar_workspace/devel/share/common-lisp/ros/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/odroid/ar_workspace/devel/share/gennodejs/ros/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/odroid/ar_workspace/devel/lib/python2.7/dist-packages/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/odroid/ar_workspace/devel/lib/python2.7/dist-packages/ros_pololu_servo")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/odroid/ar_workspace/build/ros_pololu_servo/catkin_generated/installspace/ros_pololu_servo.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/cmake" TYPE FILE FILES "/home/odroid/ar_workspace/build/ros_pololu_servo/catkin_generated/installspace/ros_pololu_servo-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo/cmake" TYPE FILE FILES
    "/home/odroid/ar_workspace/build/ros_pololu_servo/catkin_generated/installspace/ros_pololu_servoConfig.cmake"
    "/home/odroid/ar_workspace/build/ros_pololu_servo/catkin_generated/installspace/ros_pololu_servoConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_pololu_servo" TYPE FILE FILES "/home/odroid/ar_workspace/src/ros_pololu_servo/package.xml")
endif()


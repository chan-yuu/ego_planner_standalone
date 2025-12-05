# Install script for directory: /home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/src/planner/plan_manage

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ego_planner/msg" TYPE FILE FILES
    "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/src/planner/plan_manage/msg/Bspline.msg"
    "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/src/planner/plan_manage/msg/DataDisp.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ego_planner/cmake" TYPE FILE FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/catkin_generated/installspace/ego_planner-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/include/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/share/roseus/ros/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/share/common-lisp/ros/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/share/gennodejs/ros/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/lib/python3/dist-packages/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/devel/lib/python3/dist-packages/ego_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/catkin_generated/installspace/ego_planner.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ego_planner/cmake" TYPE FILE FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/catkin_generated/installspace/ego_planner-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ego_planner/cmake" TYPE FILE FILES
    "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/catkin_generated/installspace/ego_plannerConfig.cmake"
    "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/catkin_generated/installspace/ego_plannerConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ego_planner" TYPE FILE FILES "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/src/planner/plan_manage/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/cyun/Documents/ego-planner/2.0/12.4-ego-planner/ego-planner/build/planner/plan_manage/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

project(test)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")


find_package(OpenCV REQUIRED)
option(ENABLE_IMAGE "enable camera image" OFF)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../HCServer ./HCServer)
#add_subdirectory(RTIMULib)


#set(USE_VREP_SIMULATOR CACHE BOOL FORCE)
#IF($ENV{VREP_DIR})
#	set(USE_VREP_SIMULATOR True)
#ENDIF($ENV{VREP_DIR})

#IF(USE_VREP_SIMULATOR)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../SimDriver ./SimDriver)
#add_definitions(-DVREP_SIMULATOR)
#ENDIF(USE_VREP_SIMULATOR)


if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
endif()

set (WEBOTS_LIBRARIES Controller CppController)

message(${WEBOTS_LIBRARIES})

add_definitions(-D_AFXDLL -DSIM -D_CRT_SECURE_NO_WARNINGS)

include_directories(
	${CMAKE_BINARY_DIR}/gen
	${CMAKE_CURRENT_SOURCE_DIR}
	${WEBOTS_HOME}/include/controller/c 
	${WEBOTS_HOME}/include/controller/cpp
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  
	${WEBOTS_HOME}/lib/controller
${OpenCV_LIBRARIES}

)



add_executable(test
    main.cpp
)

target_link_libraries(test ${OpenCV_LIBRARIES}
	${WEBOTS_LIBRARIES}
)

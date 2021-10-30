project(hr46_b3m)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF(NOT WIN32)
	set(CMAKE_CXX_FLAGS "-fPIC")
	set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")
ENDIF(NOT WIN32)

IF(WIN32)
	set(Boost_USE_STATIC_LIBS        ON)
ELSE(WIN32)
	set(Boost_USE_STATIC_LIBS        OFF)
ENDIF(WIN32)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

find_package(Boost REQUIRED COMPONENTS system thread filesystem date_time)
find_package(Protobuf REQUIRED)

option(ENABLE_IMAGE "enable camera image" OFF)

add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../HCServer ./HCServer)
#add_subdirectory(RTIMULib)
add_subdirectory(HCServer)
add_subdirectory(protobuf)

set(ICE_DIR $ENV{ICE_HOME} CACHE PATH "ZeroC ICE installation dir(Default: $ICE_HOME)")

#set(USE_VREP_SIMULATOR CACHE BOOL FORCE)
#IF($ENV{VREP_DIR})
#	set(USE_VREP_SIMULATOR True)
#ENDIF($ENV{VREP_DIR})

#IF(USE_VREP_SIMULATOR)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../SimDriver ./SimDriver)
#add_definitions(-DVREP_SIMULATOR)
#ENDIF(USE_VREP_SIMULATOR)

option(GANKENKUN_MODE "Change to GankenKun mode" ON)
IF(GANKENKUN_MODE)
	add_definitions(-DGANKENKUN_MODE)
ENDIF(GANKENKUN_MODE)

if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
endif()

set (WEBOTS_LIBRARIES Controller CppController)

message(${WEBOTS_LIBRARIES})

add_definitions(-D_AFXDLL -DSIM -D_CRT_SECURE_NO_WARNINGS)

include_directories(
	${ICE_DIR}/include
	${CMAKE_BINARY_DIR}/gen
	${CMAKE_CURRENT_SOURCE_DIR}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${WEBOTS_HOME}/include/controller/c 
	${WEBOTS_HOME}/include/controller/cpp
	${Protobuf_INCLUDE_DIRS}
	protobuf
	#../SimDriver
	HCServer
	#RTIMULib
)

link_directories(
	${ICE_DIR}/lib
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
	${WEBOTS_HOME}/lib/controller
	#RTIMULib
)

set(HR46_B3M_SRCS
acc.c                 calc_deg.c            calc_mv.c             
cntr.c                func.c                gyro.c
joy.c                 kine.c                main.cpp
motion.c              mvtbl.c               
serv.c                servo_rs.c            serv_init.c
sq_motion.c           sq_ready.c            sq_start.c
sq_straight.c         sq_walk.c             var_init.c
pc_motion.cpp         OrientationEstimator.cpp

acc.h                 calc_deg.h            calc_mv.h
cntr.h                func.h                gyro.h
joy.h                 kine.h                motion.h
mvtbl.h                                     serv.h
servo_rs.h            serv_init.h           sq_motion.h
sq_ready.h            sq_start.h            sq_straight.h
sq_walk.h             var.h                 pc_motion.h
OrientationEstimator.h						
)


add_executable(hr46_b3m
	${HR46_B3M_SRCS}
)

if(ENABLE_IMAGE)
	target_compile_definitions(hr46_b3m PRIVATE "ENABLE_IMAGE=1")
endif()

SET(hr46_b3m_LIBRARIES
	Ice
	##IceUtil
	HCLib
)

#IF(USE_VREP_SIMULATOR)
#MESSAGE("!!!!! hr42 uses vrep simulator !!!!!")
#SET(hr46_b3m_LIBRARIES ${hr46_b3m_LIBRARIES} SimIPCLib)
#ENDIF(USE_VREP_SIMULATOR)

add_dependencies(hr46_b3m
	HCLib
#	SimIPCLib
)


target_link_libraries(hr46_b3m
	${Boost_LIBRARIES}
	${hr46_b3m_LIBRARIES}
	${WEBOTS_LIBRARIES}
	-lrt 
	protobuf::libprotobuf 
	libpictureproto
	#RTIMULib
)


SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../)

INSTALL(TARGETS hr46_b3m 
  RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  LIBRARY DESTINATION ../bin
  ARCHIVE DESTINATION ../bin
)

install(FILES servo_para.txt DESTINATION ../bin)
#install(FILES RTIMULib.ini DESTINATION ../bin)


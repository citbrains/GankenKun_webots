project(MotionEngineWebots)
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0003 NEW)

if(WIN32)
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_HAS_EXCEPTIONS=0 ")
    STRING (REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD ON)
endif(WIN32)

if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
    message("WEBOTS_HOME is set to $ENV{WEBOTS_HOME}")
else()
    if(WIN32)
        set(WEBOTS_HOME "C:\Program Files\Webots")
    else(WIN32)
        set(WEBOTS_HOME /usr/local/webots)
    endif(WIN32)
endif()

find_package(Protobuf REQUIRED)
find_package(cppzmq)

option(USE_CAMERA_SENSOR "Use camera sensor. " OFF)
IF(USE_CAMERA_SENSOR)
	add_definitions(-DCAMERA_ENABLE)
ENDIF(USE_CAMERA_SENSOR)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS message.proto)


add_executable(MotionEngine_server main.cpp server.hpp client_webots.hpp ${PROTO_SRCS} ${PROTO_HDRS} )

set (WEBOTS_LIBRARIES Controller CppController)

if(WIN32)
    find_package(WebotsCtrl REQUIRED)  #現状 https://qiita.com/kkoba775/items/d28b30a6e1f13803036fを参考にして、ビルドを行ったライブラリを使う.
    target_link_libraries(MotionEngine_server PRIVATE WebotsCtrl::CppController)
    set(WEBOTS_LIBRARIES "")
else(WIN32)
    link_directories(${WEBOTS_HOME}/lib/controller)
    target_compile_options(MotionEngine_server PUBLIC -O2 -Wall -pedantic -Wall)
    target_link_directories(MotionEngine_server PRIVATE ${WEBOTS_HOME}/lib/controller)
endif(WIN32)


target_compile_features(MotionEngine_server PUBLIC cxx_std_17)
target_include_directories(MotionEngine_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIR}
                                        ${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
target_link_libraries(MotionEngine_server PUBLIC cppzmq ${WEBOTS_LIBRARIES} ${Protobuf_LIBRARIES} protobuf::libprotoc protobuf::libprotobuf protobuf::libprotobuf-lite)


install(TARGETS MotionEngine_server 
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS MotionEngine_server 
    CONFIGURATIONS Release
    RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
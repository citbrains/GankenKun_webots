project(MotionEngineWebots)
cmake_minimum_required(VERSION 3.10)

if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
endif()

find_package(Protobuf REQUIRED)
find_package(cppzmq)

link_directories(${WEBOTS_HOME}/lib/controller)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS message.proto)

add_executable(MotionEngine_server main.cpp server.hpp client_webots.hpp ${PROTO_SRCS} ${PROTO_HDRS} )

set (WEBOTS_LIBRARIES Controller CppController)


target_compile_features(MotionEngine_server PUBLIC cxx_std_17)
target_compile_options(MotionEngine_server PUBLIC -O2 -Wall -pedantic -Wall)
target_include_directories(MotionEngine_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIR}
                                        ${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
target_link_libraries(MotionEngine_server cppzmq ${WEBOTS_LIBRARIES} ${Protobuf_LIBRARIES})

install(TARGETS MotionEngine_server DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

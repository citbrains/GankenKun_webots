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

add_executable(main main.cpp server.hpp client_webots.hpp ${PROTO_SRCS} ${PROTO_HDRS} )

set (WEBOTS_LIBRARIES Controller CppController)


target_compile_features(main PUBLIC cxx_std_17)
target_compile_options(main PUBLIC -O2 -Wall -pedantic -Wall)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIR}
                                        ${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
target_link_libraries(main cppzmq ${WEBOTS_LIBRARIES} ${Protobuf_LIBRARIES})

install(TARGETS main DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

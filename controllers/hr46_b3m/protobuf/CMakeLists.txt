find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
message(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS picture.proto)
#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS DESCRIPTORS PROTO_DESCS pro.proto)
add_library(libpictureproto STATIC ${PROTO_SRCS} ${PROTO_HDRS} )
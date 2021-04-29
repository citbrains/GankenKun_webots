find_package(Boost REQUIRED COMPONENTS thread system date_time )


include_directories(${Boost_INCLUDE_DIRS}/boost)

link_directories(${Boost_LIBRARY_DIR})

add_executable(remove remove_message_queue.cpp)

target_link_libraries(remove ${Boost_LIBRARIES} -lrt )
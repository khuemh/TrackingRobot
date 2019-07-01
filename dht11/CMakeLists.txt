cmake_minimum_required(VERSION 2.8.12)

project (My Projects)

add_executable (output upd_dht11.cpp Socket.cpp)
target_link_libraries(output Socket.h)
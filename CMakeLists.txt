cmake_minimum_required(VERSION 3.16.0)

project(Open-SAE-J1939 C)

FILE(GLOB_RECURSE SOURCE_FILES_LIST "Src/*.c" "Src/*.c")

add_executable(main ${SOURCE_FILES_LIST})
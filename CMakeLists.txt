cmake_minimum_required(VERSION 3.16.0)

project(Open-SAE-J1939 C)

# Default value if not set externally
if(NOT DEFINED PROCESSOR_CHOICE)
    set(PROCESSOR_CHOICE NO_PROCESSOR)
endif()

# Make processor choice a compile definition
add_compile_definitions(PROCESSOR_CHOICE=${PROCESSOR_CHOICE})

# Collect sources
file(GLOB_RECURSE ALL_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c")
set(MAIN_SRC "${CMAKE_CURRENT_SOURCE_DIR}/Src/Main.c")
list(REMOVE_ITEM ALL_SOURCE_FILES ${MAIN_SRC})

# Library
add_library(opensaej1939 STATIC ${ALL_SOURCE_FILES})
target_include_directories(opensaej1939 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Src")

# Executable
add_executable(main ${MAIN_SRC})
target_link_libraries(main PRIVATE opensaej1939)
target_include_directories(main PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Src")

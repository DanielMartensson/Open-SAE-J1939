# Offline regression: cmake -S tests -B build-tests
#                     cmake --build build-tests
#                     ctest --test-dir build-tests --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(OpenSAEJ1939Tests C)

set(OPENSAE_J1939_TARGET_PLATFORM INTERNAL_CALLBACK)
add_subdirectory(.. opensaej1939)

enable_testing()
add_executable(transport_eom transport_eom.c)
target_link_libraries(transport_eom PRIVATE opensaej1939)
add_test(NAME transport_eom COMMAND transport_eom)

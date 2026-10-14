cmake_minimum_required(VERSION 3.16)
project(wmirror_cmocka_first C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMOCKA_ROOT "/Users/admin/Tools/cmocka-2.0.2/build/install")

find_path(CMOCKA_INCLUDE_DIR cmocka.h
  PATHS "${CMOCKA_ROOT}/include"
  REQUIRED
)

find_library(CMOCKA_LIBRARY cmocka
  PATHS "${CMOCKA_ROOT}/lib"
  REQUIRED
)

add_executable(test_basic test_basic.c)
target_include_directories(test_basic PRIVATE "${CMOCKA_INCLUDE_DIR}")
target_link_libraries(test_basic PRIVATE "${CMOCKA_LIBRARY}")

enable_testing()
add_test(NAME test_basic COMMAND test_basic)

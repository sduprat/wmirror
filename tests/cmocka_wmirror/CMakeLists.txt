cmake_minimum_required(VERSION 3.16)
project(wmirror_cmocka_logic C)

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

add_executable(test_wmirror_logic
  test_wmirror_logic.c
  ../../wmirror_logic.c
)

target_include_directories(test_wmirror_logic PRIVATE
  "${CMOCKA_INCLUDE_DIR}"
  "/Users/maison/WORK/wmirror"
)
target_link_libraries(test_wmirror_logic PRIVATE "${CMOCKA_LIBRARY}")

enable_testing()
add_test(NAME test_wmirror_logic COMMAND test_wmirror_logic)

cmake_minimum_required(VERSION 3.5.1)
project(PCESystems)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BIN_ROOT_DIR ${CMAKE_BINARY_DIR})
set(SRC_ROOT_DIR ${CMAKE_SOURCE_DIR})

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

include_directories(include)

find_package(drake CONFIG REQUIRED)
find_package(PythonInterp 3.6 EXACT MODULE REQUIRED)

set(PYTHONPATH
  "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages:$ENV{PYTHONPATH}")

set(
  CORELIBS 
  drake::drake 
  gflags
)

file(
  GLOB 
  SRC_FILES
  "src/*.cc"
)

file(
  GLOB 
  TEST_SRC_FILES
  "test/*.cc"
)

foreach(TEST_SRC ${TEST_SRC_FILES})
  # generate test name from test file name
  get_filename_component(
    TEST_EXE_NAME 
    ${TEST_SRC} 
    NAME_WE
  )

  # create binary for test
  add_executable(
    ${TEST_EXE_NAME} 
    ${TEST_SRC}
    ${SRC_FILES}
  )
  target_link_libraries(
    ${TEST_EXE_NAME} 
    ${CORELIBS}
  )

  # add CTest (run with 'make test')
  add_test(
    NAME 
    ${TEST_EXE_NAME} 
    WORKING_DIRECTORY 
    ${BIN_ROOT_DIR}
    COMMAND 
    ${TEST_EXE_NAME}
  )
endforeach(TEST_SRC)

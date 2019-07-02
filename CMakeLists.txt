cmake_minimum_required(VERSION 3.5.1)
project(DR)

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

##############################################################
# Start code for automatically downloading and building gtest.
# Taken from: https://crascit.com/2015/07/25/cmake-gtest/
##############################################################

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                 "${CMAKE_BINARY_DIR}/googletest-build")

# The gtest/gmock targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if(CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

##############################################################
# End code for automatically downloading and building gtest.
##############################################################

find_package(drake CONFIG REQUIRED)

set(
  CORELIBS 
  drake::drake 
  gflags
  gtest
)

file(
  GLOB 
  SRC_FILES
  "src/*.cc"
  "primitives/*.cc"
)

file(
  GLOB 
  TEST_SRC_FILES
  "primitives/test/*.cc"
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
    gtest_main
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

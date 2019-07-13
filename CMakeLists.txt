cmake_minimum_required(VERSION 3.5.1)
project(DR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Activate C++17. Also turn on all compiler warnings, and make warnings errors, in compilation.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wunused-but-set-parameter -Wno-free-nonheap-object -Werror=all -Werror=deprecated -Werror=deprecated-declarations -Werror=ignored-qualifiers -Werror=old-style-cast -Werror=overloaded-virtual -Werror=shadow -Werror=unused-result -Werror=extra -Werror=logical-op -Werror=non-virtual-dtor -Werror=return-local-addr -Werror=unused-but-set-parameter -Wno-missing-field-initializers -fno-canonical-system-headers -Wno-builtin-macro-redefined") 

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

# Collect source files for core lib.
file(
  GLOB 
  SRC_FILES
  "common/*.cc"
  "primitives/*.cc"
  "simulation/*.cc"
)

# Create project core library.
add_library(
  ${PROJECT_NAME}_core
  ${SRC_FILES}
)

target_link_libraries(
  ${PROJECT_NAME}_core
  drake::drake 
)

# Create and add test binaries to CTest.
file(
  GLOB 
  TEST_SRC_FILES
  "common/test/*.cc"
  "primitives/test/*.cc"
  "simulation/test/*.cc"
)

foreach(TEST_SRC ${TEST_SRC_FILES})
  # Generate test name from test file name.
  get_filename_component(
    TEST_EXE_NAME 
    ${TEST_SRC} 
    NAME_WE
  )

  # Create binary for test.
  add_executable(
    ${PROJECT_NAME}_${TEST_EXE_NAME} 
    ${TEST_SRC}
  )

  target_link_libraries(
    ${PROJECT_NAME}_${TEST_EXE_NAME} 
    ${PROJECT_NAME}_core
    gtest
    gtest_main
  )

  # Add CTest (run with 'make test').
  add_test(
    NAME 
    ${PROJECT_NAME}_${TEST_EXE_NAME} 
    WORKING_DIRECTORY 
    ${BIN_ROOT_DIR}
    COMMAND 
    ${PROJECT_NAME}_${TEST_EXE_NAME}
  )
endforeach(TEST_SRC)

cmake_minimum_required(VERSION 3.16)
project(cloudini_python LANGUAGES C CXX)

# Installers that run CMake themselves may not set a build type: build optimized.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# In a conda/pixi environment, use its pybind11, zstd and lz4.
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

# cloudini_lib: next to this file in a self-contained copy of the package, else the repository's.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cloudini_lib/CMakeLists.txt)
  set(CLOUDINI_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cloudini_lib)
else()
  set(CLOUDINI_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cloudini_lib)
endif()

# cloudini_lib is linked statically into the extension module.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Use zstd and lz4 from the system when found; download them only as a fallback.
set(CLOUDINI_FORCE_VENDORED_DEPS OFF CACHE BOOL "Force vendored dependencies")
add_subdirectory(${CLOUDINI_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/cloudini_lib EXCLUDE_FROM_ALL)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(_cloudini src/cloudini_python.cpp)
target_link_libraries(_cloudini PRIVATE cloudini::cloudini_lib)
# Link the C++ runtime explicitly: some installers (e.g. Bazel toolchains) link with the C driver (gcc), which
# does not add it, and the module then fails to import (undefined std:: symbols).
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(_cloudini PRIVATE stdc++)
endif()
# Shared zstd/lz4 of the environment, <prefix>/lib, from the module in <prefix>/lib/pythonX.Y/site-packages/cloudini
# or in <prefix>/cloudini.
set_target_properties(_cloudini PROPERTIES INSTALL_RPATH "$ORIGIN/../../..:$ORIGIN/../lib" BUILD_WITH_INSTALL_RPATH ON)

if(SKBUILD)
  install(TARGETS _cloudini LIBRARY DESTINATION cloudini)
else()
  # Plain CMake build: put the module in the package directory, which is then importable as is.
  set_target_properties(_cloudini PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cloudini)
endif()

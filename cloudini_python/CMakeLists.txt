cmake_minimum_required(VERSION 3.16)
project(cloudini_python LANGUAGES C CXX)

# In a conda/pixi environment, use its pybind11, zstd and lz4.
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

# cloudini_lib is linked statically into the extension module.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Use zstd and lz4 from the system when found; download them only as a fallback.
set(CLOUDINI_FORCE_VENDORED_DEPS OFF CACHE BOOL "Force vendored dependencies")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cloudini_lib ${CMAKE_CURRENT_BINARY_DIR}/cloudini_lib EXCLUDE_FROM_ALL)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(_cloudini src/cloudini_python.cpp)
target_link_libraries(_cloudini PRIVATE cloudini::cloudini_lib)
# Shared zstd/lz4 of the environment: <prefix>/lib from <prefix>/lib/pythonX.Y/site-packages/cloudini.
set_target_properties(_cloudini PROPERTIES INSTALL_RPATH "$ORIGIN/../../..")

install(TARGETS _cloudini LIBRARY DESTINATION cloudini)

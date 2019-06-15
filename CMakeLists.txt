cmake_minimum_required (VERSION 3.11)
project (ProgParallel)

option(PROF "Reference build" NO)
include(FetchContent)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")

FetchContent_Declare(
  GoogleBenchmark
  URL https://github.com/google/benchmark/archive/v1.4.1.tar.gz
  )

if (NOT GoogleBenchmark_POPULATED)
  FetchContent_Populate(GoogleBenchmark)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "From Gtest")
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "From Gtest")
  add_subdirectory(${googlebenchmark_SOURCE_DIR} ${googlebenchmark_BINARY_DIR})
endif()

find_package(Threads REQUIRED)
find_package(GTest)



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g3 -W -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=thread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

include_directories(lib/include)

add_library(quadtree
  src/IQuadTree.hpp
  src/IQuadTree.cpp
  src/IAsyncQuadTree.hpp
  src/naive_quadtree.hpp
  src/naive_quadtree.cpp
  src/naive_async_quadtree.hpp
  src/naive_async_quadtree.cpp
  src/optimized_octree.hpp
  src/optimized_async_octree.hpp
  src/tools.hpp
  src/tools.cpp
  )


add_executable(tests src/tests.cpp)
target_link_libraries(tests quadtree GTest::GTest GTest::Main Threads::Threads)

add_executable(bench src/bench.cpp)
target_link_libraries(bench quadtree benchmark)

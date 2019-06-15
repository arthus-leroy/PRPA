#include <gtest/gtest.h>
#include <functional>
#include <thread>
#include "tools.hpp"
#include "naive_quadtree.hpp"
#include "naive_async_quadtree.hpp"
# include "optimized_octree.hpp"
# include "optimized_async_octree.hpp"

using namespace std::string_literals;

// TODO
// Adapt/Create new tests tests with your new structures
// naive_quadtree/async_naive_quadtree can be used as references

# define SYNC(N)                                                              \
    TEST(Sync_OcTree_ ## N, SimpleScenario)                                   \
    {                                                                         \
        std::size_t n = 10;                                                   \
        std::size_t nqueries = 20;                                            \
                                                                              \
        Scenario scn(n, nqueries);                                            \
                                                                              \
        OptimizedOcTree<point, N> dic;                                        \
        scn.prepare(dic);                                                     \
        scn.execute_verbose(dic);                                             \
    }

SYNC(1)
SYNC(2)
SYNC(3)
SYNC(4)
SYNC(5)

# define ASYNC(N)                                                             \
    TEST(Async_OcTree_ ## N, AsyncConsistency)                                \
    {                                                                         \
        std::size_t n = 10000; /* 100000 */                                   \
        std::size_t nqueries = 64; /* 512 */                                  \
        Scenario scn(n, nqueries);                                            \
                                                                              \
        OptimizedOcTree<point, N> dic;                                        \
        AsyncOptimizedOcTree<point, N> async_dic;                             \
        scn.prepare(dic);                                                     \
        scn.prepare(async_dic);                                               \
        auto r1 = scn.execute(async_dic);                                     \
        auto r2 = scn.execute(dic);                                           \
        ASSERT_EQ(r1, r2);                                                    \
    }

// FIXME: tests for async fail, fix it
ASYNC(1)
ASYNC(2)
ASYNC(3)
ASYNC(4)
ASYNC(5)

long l2norm2(point p)
{
  return (long)p.x * p.x + (long)p.y * p.y + (long)p.z * p.z;
}


// A basic add/remove/search test
TEST(Quadtree, Basic)
{
  naive_quadtree dic = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0},
                        {0, 2, 0}, {0, 0, 2}, {0, -10, 0}};

  {
    auto r1 = dic.search({0,0,0});
    ASSERT_EQ(r1[0], (point{1,0,0}));
    ASSERT_EQ(l2norm2(r1[1]), 4);
    ASSERT_EQ(l2norm2(r1[2]), 4);
    ASSERT_EQ(l2norm2(r1[3]), 4);
    ASSERT_EQ(r1[4], (point{3,0,0}));
  }

  {
    dic.insert({0,0,-1});
    auto r2 = dic.search({0,0,0});
    ASSERT_EQ(l2norm2(r2[0]), 1);
    ASSERT_EQ(l2norm2(r2[1]), 1);
  }

  {
    dic.erase({1,0,0});
    auto r3 = dic.search({0,0,0});
    ASSERT_EQ(r3[0], (point{0,0,-1}));
  }
}

// Test that executes some operations concurrently
// Does not test anything in itself but can be used to detected bugs
// with the thread sanitizer enabled
TEST(Quadtree, ConcurrentOperations)
{
  using namespace std::placeholders;


  std::size_t n = 250;
  // Cut in 3 parts: A B C
  // Initialize with A B
  // 2 threads Search A
  // 2 threads Remove B
  // 2 threads Insert C

  auto data = generate_points(6 * n);
  naive_quadtree dic(data.begin(), data.begin() + 4 * n);

  std::thread t[6];
  const point* data_ptr = data.data();

  t[0] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 0 * n, data_ptr + 1 * n, std::bind(&IQuadTree::search, &dic, _1)); });
  t[1] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 1 * n, data_ptr + 2 * n, std::bind(&IQuadTree::search, &dic, _1)); });

  t[2] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 2 * n, data_ptr + 3 * n, std::bind(&IQuadTree::erase, &dic, _1)); });
  t[3] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 3 * n, data_ptr + 4 * n, std::bind(&IQuadTree::erase, &dic, _1)); });

  t[4] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 4 * n, data_ptr + 5 * n, std::bind(&IQuadTree::insert, &dic, _1)); });
  t[5] = std::thread([&dic,data_ptr, n]() { std::for_each(data_ptr + 5 * n, data_ptr + 6 * n, std::bind(&IQuadTree::insert, &dic, _1)); });

  for (int i = 0; i < 6; ++i)
    t[i].join();
}


// A simple scenario
TEST(Quadtree, SimpleScenario)
{
  std::size_t n = 10;
  std::size_t nqueries = 20;

  Scenario scn(n, nqueries);

  naive_quadtree dic;
  scn.prepare(dic);
  scn.execute_verbose(dic);
}


// A long scenario, check that the async quadtree as the
// same output as the blocking one
TEST(Quadtree, AsyncConsistency)
{

  std::size_t n = 10000; // 100000
  std::size_t nqueries = 64; // 512
  Scenario scn(n, nqueries);

  naive_quadtree dic;
  naive_async_quadtree async_dic;
  scn.prepare(dic);
  scn.prepare(async_dic);
  auto r1 = scn.execute(async_dic);
  auto r2 = scn.execute(dic);
  ASSERT_EQ(r1, r2);
}



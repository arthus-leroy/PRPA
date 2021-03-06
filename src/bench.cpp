#include <functional>
#include "tools.hpp"
#include "naive_quadtree.hpp"
#include "naive_async_quadtree.hpp"
# include "octree.hpp"

#include <benchmark/benchmark.h>

constexpr int NPOINTS = 1000000;
constexpr int NQUERIES = 10000;

class BMScenario : public ::benchmark::Fixture
{
public:
  void SetUp(benchmark::State&)
    {
      if (!m_scenario)
      {
        m_scenario = std::make_unique<Scenario>(NPOINTS, NQUERIES);
      }
    }

protected:
  static std::unique_ptr<Scenario> m_scenario;
};

std::unique_ptr<Scenario> BMScenario::m_scenario;

# define SYNC(N)                                                              \
    BENCHMARK_DEFINE_F(BMScenario, OcTree_Opti_V1_ ## N)(benchmark::State& st)\
    {                                                                         \
        SyncOctree<N> dic;                                                    \
        m_scenario->prepare(dic);                                             \
                                                                              \
        for (auto _ : st)                                                     \
            m_scenario->execute(dic);                                         \
                                                                              \
        st.SetItemsProcessed(st.iterations() * NQUERIES);                     \
    }                                                                         \
                                                                              \
    BENCHMARK_REGISTER_F(BMScenario, OcTree_Opti_V1_ ## N)->Unit(benchmark::kMillisecond);

SYNC(1)
SYNC(2)
SYNC(3)
SYNC(4)
SYNC(5)
SYNC(6)
SYNC(7)
SYNC(8)
SYNC(9)
SYNC(10)
SYNC(11)
SYNC(12)
SYNC(13)
SYNC(14)
SYNC(15)
SYNC(16)
SYNC(17)
SYNC(18)
SYNC(19)
SYNC(20)

# define ASYNC(N)                                                             \
    BENCHMARK_DEFINE_F(BMScenario, Async_OcTree_Opti_V1_ ## N)(benchmark::State& st)\
    {                                                                         \
        AsyncOctree<N> dic;                                                   \
        m_scenario->prepare(dic);                                             \
                                                                              \
        for (auto _ : st)                                                     \
            m_scenario->execute(dic);                                         \
                                                                              \
        st.SetItemsProcessed(st.iterations() * NQUERIES);                     \
    }                                                                         \
                                                                              \
    BENCHMARK_REGISTER_F(BMScenario, Async_OcTree_Opti_V1_ ## N)->Unit(benchmark::kMillisecond);

ASYNC(1)
ASYNC(2)
ASYNC(3)
ASYNC(4)
ASYNC(5)
ASYNC(6)
ASYNC(7)
ASYNC(8)
ASYNC(9)
ASYNC(10)
ASYNC(11)
ASYNC(12)
ASYNC(13)
ASYNC(14)
ASYNC(15)
ASYNC(16)
ASYNC(17)
ASYNC(18)
ASYNC(19)
ASYNC(20)

BENCHMARK_DEFINE_F(BMScenario, Naive_NoAsync)(benchmark::State& st)
{
  naive_quadtree dic;
  m_scenario->prepare(dic);

  for (auto _ : st)
    m_scenario->execute(dic);

  st.SetItemsProcessed(st.iterations() * NQUERIES);
}

BENCHMARK_DEFINE_F(BMScenario, Naive_Async)(benchmark::State& st)
{
  naive_async_quadtree dic;
  m_scenario->prepare(dic);

  for (auto _ : st)
    m_scenario->execute(dic);

  st.SetItemsProcessed(st.iterations() * NQUERIES);
}

BENCHMARK_REGISTER_F(BMScenario, Naive_NoAsync)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BMScenario, Naive_Async)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();

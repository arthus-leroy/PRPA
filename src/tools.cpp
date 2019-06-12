#include "tools.hpp"
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <fstream>
#include <iostream>
#include <cmath>

///! Load the word list from a file
std::vector<point> generate_points(std::size_t n)
{
  std::random_device rd;
  std::mt19937 g(rd());

  std::uniform_real_distribution<> angle(0.0, 2 * M_PI);
  std::normal_distribution<> noise(0.0, 0.1);

  std::vector<point> pts(n);
  constexpr float a = 0.6f;
  constexpr float r = 0.15f;
  for (std::size_t i = 0; i < n; ++i)
  {
    double u = angle(g);
    double v = angle(g);
    double x = std::clamp((a + r * std::cos(u)) * std::cos(v) + noise(g), -1., 1.);
    double y = std::clamp((a + r * std::cos(u)) * std::sin(v) + noise(g), -1., 1.);
    double z = std::clamp(r * std::sin(u) + noise(g), -1., 1.);
    int16_t x0 = static_cast<short>(x * INT16_MAX);
    int16_t y0 = static_cast<short>(y * INT16_MAX);
    int16_t z0 = static_cast<short>(z * INT16_MAX);
    pts[i] = {x0, y0, z0};
  }
  return pts;
}


point add_noise(point p)
{
  static std::random_device rd;
  static std::mt19937 g(rd());
  static std::normal_distribution<> noise(0.0, 0.05);

  point q = p;
  q.x = (short)std::clamp(q.x + (noise(g) * INT16_MAX), (double)INT16_MIN, (double)INT16_MAX);
  q.y = (short)std::clamp(q.y + (noise(g) * INT16_MAX), (double)INT16_MIN, (double)INT16_MAX);
  q.z = (short)std::clamp(q.z + (noise(g) * INT16_MAX), (double)INT16_MIN, (double)INT16_MAX);
  return q;
}



struct query
{
  enum op_type
  {
    search,
    insert,
    erase
  };

  op_type op;
  point arg;
};


struct Scenario::scenario_impl_t
{
  std::vector<point> init_points;
  std::vector<query> queries;
};


Scenario::Scenario()
  : m_impl(std::make_unique<scenario_impl_t>())
{
}

Scenario::~Scenario()
{
}

Scenario::Scenario(std::size_t n, std::size_t nqueries)
  : m_impl(std::make_unique<scenario_impl_t>())
{

  // Create queries with these distributions
  // 70% search
  // 15% insert
  // 15% erase

  const std::size_t n_searches = nqueries * 70 / 100;
  const std::size_t n_indels = nqueries * 15 / 100;
  nqueries = n_searches + 2 * n_indels; // rounding
  this->m_impl->queries.reserve(nqueries);


  std::vector<point> points = generate_points(n);

  // Init quadtree with 85%
  this->m_impl->init_points.assign(points.begin(),
                                   points.begin() + n * 80 / 100);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> rand(0, n - 1);
  // Generate search queries
  {
    for (std::size_t i = 0; i < n_searches; ++i)
    {
      auto x = points[rand(gen)];
      this->m_impl->queries.push_back({ query::search, add_noise(x) });
    }
  }

  // Generate insert/erase queries
  {
    for (std::size_t i = 0; i < n_indels; ++i)
    {
      auto x = points[rand(gen)];
      this->m_impl->queries.push_back({query::insert, x});
      this->m_impl->queries.push_back({query::erase, x});
    }
  }

  // Shuffle everything
  std::shuffle(this->m_impl->queries.begin(), this->m_impl->queries.end(), gen);
}



void Scenario::prepare(IQuadTreeBase& dic) const
{
  dic.init(m_impl->init_points);
}

std::vector<result_t> Scenario::execute(IQuadTree& qt) const
{
  std::size_t n = this->m_impl->queries.size();
  std::vector<result_t> results;

  results.reserve(n * 7 / 10);
  for (auto&& q : this->m_impl->queries)
  {
    switch (q.op)
    {
    case query::search: results.push_back(qt.search(q.arg)); break;
    case query::insert: qt.insert(q.arg); break;
    case query::erase:  qt.erase(q.arg); break;
    }
  }

  return results;
}

void Scenario::execute_verbose(IQuadTree& dic) const
{
  std::size_t n = this->m_impl->queries.size();
  std::vector<result_t> results;

  results.reserve(n * 7 / 10);

  std::cout << "I: {\n";
  for (const auto& w : m_impl->init_points)
    std::cout << w << ", ";
  std::cout << "}\n";

  for (auto&& q : this->m_impl->queries)
  {
    switch (q.op)
    {
    case query::search:
    {
      auto r = dic.search(q.arg);
      std::cout << "S: " << q.arg << " -> " << "[";
      for (auto p : r)
        std::cout << p << " ";
      std::cout << "]\n";
      break;
    }
    case query::insert:
      std::cout << "A: " << q.arg << "\n";
      dic.insert(q.arg);
      break;
    case query::erase:
      std::cout << "D: " << q.arg << "\n";
      dic.erase(q.arg);
      break;
    }
  }
}


std::vector<result_t> Scenario::execute(IAsyncQuadTree& dic) const
{
  std::size_t n = this->m_impl->queries.size();

  std::vector<result_t> results;
  results.reserve(n * 7 / 10);

  constexpr int SEARCH_BUFFER_SIZE = 50;
  constexpr int INDEL_BUFFER_SIZE = 50;

  std::future<result_t> search_buffer[SEARCH_BUFFER_SIZE];
  std::future<void>     indel_buffer[INDEL_BUFFER_SIZE];

  unsigned search_buffer_pos = 0;
  unsigned indel_buffer_pos = 0;

  for (auto&& q : this->m_impl->queries)
  {
    if (q.op == query::search)
    {
      if (search_buffer[search_buffer_pos].valid())
        results.push_back(search_buffer[search_buffer_pos].get());
    }
    else
    {
      if (indel_buffer[indel_buffer_pos].valid())
        indel_buffer[indel_buffer_pos].get();
    }

    switch (q.op)
    {
    case query::search:
      search_buffer[search_buffer_pos] = dic.search(q.arg); // non-blocking
      search_buffer_pos = (search_buffer_pos + 1) % SEARCH_BUFFER_SIZE;
      break;
    case query::insert:
      indel_buffer[indel_buffer_pos] = dic.insert(q.arg);
      indel_buffer_pos = (indel_buffer_pos + 1) % INDEL_BUFFER_SIZE;
      break;
    case query::erase:
      indel_buffer[indel_buffer_pos] = dic.erase(q.arg);
      indel_buffer_pos = (indel_buffer_pos + 1) % INDEL_BUFFER_SIZE;
      break;
    }
  }

  for (int i = 0; i < SEARCH_BUFFER_SIZE; ++i)
  {
    if (search_buffer[search_buffer_pos].valid())
      results.push_back(search_buffer[search_buffer_pos].get());
    search_buffer_pos = (search_buffer_pos + 1) % SEARCH_BUFFER_SIZE;
  }
  return results;
}

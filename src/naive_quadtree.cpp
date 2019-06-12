#include "naive_quadtree.hpp"
#include <cassert>
#include <algorithm>

naive_quadtree::naive_quadtree(const std::initializer_list<point>& init)
  : m_set(init.begin(), init.end())
{
}


void naive_quadtree::init(const std::vector<point>& l) noexcept
{
  m_set.clear();
  m_set.insert(l.begin(), l.end());
}


namespace
{

  long l2distsqr(point a, point b)
  {
    long x = a.x - b.x;
    long y = a.y - b.y;
    long z = a.z - b.z;
    return x*x + y*y + z*z;
  }

}

result_t naive_quadtree::search(point query) const noexcept
{
  std::lock_guard l(m);

  assert(m_set.size() >= K);

  std::vector<std::pair<long, point>> distances(m_set.size());

  // Compute the distance to the query
  std::transform(m_set.begin(), m_set.end(), distances.begin(),
                 [b = query](point a) {
                   long d = l2distsqr(a, b);
                   return std::make_pair(d, a);
                 });

  // Sort
  std::partial_sort(
      distances.begin(), distances.begin() + K, distances.end(),
      [](const auto &a, const auto &b) { return a.first < b.first; });

  // Copy to output

  result_t res;
  for (int i = 0; i < K; ++i)
    res[i] = distances[i].second;

  return res;
}


void naive_quadtree::insert(point p) noexcept
{
  std::lock_guard l(m);
  m_set.insert(p);
}

void naive_quadtree::erase(point p) noexcept
{
  std::lock_guard l(m);
  m_set.erase(p);
}

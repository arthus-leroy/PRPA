#pragma once

#include "IQuadTree.hpp"
#include <mutex>
#include <set>

class naive_quadtree : public IQuadTree
{
public:
  naive_quadtree() = default;
  naive_quadtree(const std::initializer_list<point>& init);

  template <class Iterator>
  naive_quadtree(Iterator begin, Iterator end);

  void init(const std::vector<point>& l) noexcept final;


  result_t      search(point p) const noexcept final;
  void          insert(point p) noexcept final;
  void          erase(point p) noexcept final;

private:
  std::set<point> m_set;
  mutable std::mutex m;
};



template <class Iterator>
naive_quadtree::naive_quadtree(Iterator begin, Iterator end)
  : m_set(begin, end)
{
}

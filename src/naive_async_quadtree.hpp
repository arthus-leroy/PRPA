#pragma once

#include "IAsyncQuadTree.hpp"
#include "naive_quadtree.hpp"


/// The naive implementation is blocking to ensure Sequential Consistency
class naive_async_quadtree : public IAsyncQuadTree
{
public:
  naive_async_quadtree() = default;

  template <class Iterator>
  naive_async_quadtree(Iterator begin, Iterator end);
  naive_async_quadtree(const std::initializer_list<point>& init);
  void init(const std::vector<point>& word_list) noexcept final;

  std::future<result_t> search(point p) const noexcept final;
  std::future<void> insert(point p) noexcept final;
  std::future<void> erase(point p) noexcept final;

private:
  naive_quadtree m_impl;
};

template <class Iterator>
naive_async_quadtree::naive_async_quadtree(Iterator begin, Iterator end)
  : m_impl(begin, end)
{
}

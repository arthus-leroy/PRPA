#include "naive_async_quadtree.hpp"



naive_async_quadtree::naive_async_quadtree(const std::initializer_list<point>& init)
  : m_impl(init)
{
}

void naive_async_quadtree::init(const std::vector<point>& word_list) noexcept
{
  m_impl.init(word_list);
}


std::future<result_t> naive_async_quadtree::search(point query) const noexcept
{
  std::promise<result_t> p;
  p.set_value(m_impl.search(query));
  return p.get_future();
}


std::future<void> naive_async_quadtree::insert(point w) noexcept
{
  std::promise<void> p;
  m_impl.insert(w);
  p.set_value();
  return p.get_future();
}

std::future<void> naive_async_quadtree::erase(point w) noexcept
{
  std::promise<void> p;
  m_impl.erase(w);
  p.set_value();
  return p.get_future();
}

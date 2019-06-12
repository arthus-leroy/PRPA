#pragma once
#include "IQuadTree.hpp"
#include <future>

class IAsyncQuadTree : public IQuadTreeBase
{
public:
  IAsyncQuadTree() = default;

  virtual std::future<result_t> search(point p) const noexcept = 0;
  virtual std::future<void> insert(point p) noexcept = 0;
  virtual std::future<void> erase(point p) noexcept = 0;
};

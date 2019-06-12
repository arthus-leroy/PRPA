#pragma once

#include <array>
#include <vector>
#include <tuple>
#include <utility>
#include <cstdint>
#include <iosfwd>

struct point {
  std::int16_t x, y, z;

  constexpr bool operator<(const point &b) const {
    return std::tie(x, y, z) < std::tie(b.x, b.y, b.z);
  }

  constexpr bool operator==(const point &b) const {
    return std::tie(x, y, z) == std::tie(b.x, b.y, b.z);
  }
  
  friend std::ostream& operator<<(std::ostream &os, point p);
};

// Result = array of points
constexpr int K = 5;
using result_t = std::array<point, K>;

class IQuadTreeBase
{
public:
  // Reset the quadtree with this set of points
  virtual void init(const std::vector<point>& v) noexcept = 0;
};

class IQuadTree : public IQuadTreeBase
{
  public:
  IQuadTree() = default;

  virtual result_t search(point p) const noexcept = 0;
  virtual void insert(point p) noexcept = 0;
  virtual void erase(point p) noexcept = 0;
};

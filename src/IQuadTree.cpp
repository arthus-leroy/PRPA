#include <iostream>
#include "IQuadTree.hpp"

std::ostream &operator<<(std::ostream &os, point p)
{
  return os << '(' << p.x << ',' << p.y << ',' << p.z << ')';
}

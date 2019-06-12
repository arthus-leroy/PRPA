#pragma once
#include "IQuadTree.hpp"
#include "IAsyncQuadTree.hpp"
#include <string>
#include <vector>
#include <memory>


///! Generate random points
std::vector<point> generate_points(std::size_t n);


class Scenario
{
public:
  ~Scenario();
  Scenario();
  Scenario(Scenario&&) = default;

  // Create a scenario
  // \param n Number of points to generate
  // \param nqueries Number of queries on the dataset to perform
  Scenario(std::size_t n, std::size_t nqueries);

  // Prepare and populate the quadtree
  void prepare(IQuadTreeBase& qt) const;

  // Execute a scenario sync-way
  std::vector<result_t> execute(IQuadTree& qt) const;

  // Execute a scenario async-way
  std::vector<result_t> execute(IAsyncQuadTree& qt) const;

  void execute_verbose(IQuadTree& qt) const;

private:
  struct scenario_impl_t;
  std::unique_ptr<scenario_impl_t> m_impl;
};



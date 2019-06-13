# pragma once

# include <array>
# include <bitset>
# include <memory>

# include "IAsyncQuadTree.hpp"
# include "optimized_octree.hpp"

template <typename T, unsigned N>
class AsyncOptimizedOcTree : IAsyncQuadTree
{
public:
    AsyncOptimizedOcTree(const std::initializer_list<T>& init = {});

    void init(const std::initializer_list<T>& init);
    std::future<result_t> search(const T elem) const noexcept final;
    std::future<void> insert(const T elem) noexcept final;
    std::future<void> erase(const T elem) noexcept final;

private:
    OptimizedOcTree<T, N> octree_;
};
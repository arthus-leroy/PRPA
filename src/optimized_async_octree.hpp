# pragma once

# include <array>
# include <bitset>
# include <memory>

# include "IAsyncQuadTree.hpp"
# include "optimized_octree.hpp"

template <typename T, unsigned N>
class AsyncOptimizedOcTree : public IAsyncQuadTree
{
public:
    AsyncOptimizedOcTree() = default;
    AsyncOptimizedOcTree(const std::initializer_list<T>& init)
    {
        this->init(init);
    }

    inline void init(const std::vector<T>& init) noexcept
    {
        octree_.init(init);
    }

    inline std::future<result_t> search(const T e) const noexcept final
    {
        std::promise<result_t> promise;
        promise.set_value(octree_.search(e));
        return promise.get_future();
    }

    inline std::future<void> insert(const T e) noexcept final
    {
        std::promise<void> promise;
        octree_.insert(e);
        promise.set_value();
        return promise.get_future();
    }

    inline std::future<void> erase(const T e) noexcept final
    {
        std::promise<void> promise;
        octree_.erase(e);
        promise.set_value();
        return promise.get_future();
    }

private:
    OptimizedOcTree<T, N> octree_;
};
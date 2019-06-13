# include "optimized_async_octree.hpp"

template <typename T, unsigned N>
AsyncOptimizedOcTree<T, N>::AsyncOptimizedOcTree(const std::initializer_list<T>& init)
{
    this->init(init);
}

template <typename T, unsigned N>
void AsyncOptimizedOcTree<T, N>::init(const std::initializer_list<T>& init)
{
    for (const auto&  elem : init)
        octree_.insert(elem);
}

template <typename T, unsigned N>
std::future<result_t>
AsyncOptimizedOcTree<T, N>::search(const T e) const noexcept
{
    std::promise<result_t> promise;
    promise.set_value(octree_.search(e));
    return promise.get_future();
}

template <typename T, unsigned N>
std::future<void>
AsyncOptimizedOcTree<T, N>::insert(const T e) noexcept
{
    std::promise<void> promise;
    promise.set_value(octree_.insert(e));
    return promise.get_future();
}

template <typename T, unsigned N>
std::future<void>
AsyncOptimizedOcTree<T, N>::erase(const T e) noexcept
{
    std::promise<void> promise;
    promise.set_value(octree_.erase(e));
    return promise.get_future();
}
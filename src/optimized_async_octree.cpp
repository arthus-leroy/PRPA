# include <strings.h>
# include <queue>

# include "optimized_async_octree.hpp"

template <typename T>
node<T>::node()
    : free(-1ULL)
{}

template <typename T>
AsyncOptimizedQuadTree<T>::AsyncOptimizedQuadTree(const std::initializer_list<T>& init)
{
    this->init(init);
}

template <typename T>
void AsyncOptimizedQuadTree<T>::init(const std::initializer_list<T>& init)
{
    for (const auto&  elem : init)
        insert(elem);
}

namespace
{
    inline long distance(const point a, const point b)
    {
        long x = a.x - b.x;
        long y = a.y - b.y;
        long z = a.z - b.z;

        return x*x + y*y + z*z;
    }

    /// return index of direction corresponding to corrdinates in a cube
    inline int get_dir(const point a, const point b)
    {
        return 4 * ((a.z - b.z) >= 0)
             + 2 * ((a.y - b.y) >= 0)
             + 1 * ((a.x - b.x) >= 0);
    }
}

template <typename T>
std::future<result_t>
AsyncOptimizedQuadTree<T>::search(const T e) const noexcept
{
    // previous node's lock
    std::mutex prev;
    // return value
    std::promise<result_t> ret;
    // nearest element found
    T nearest;
    // queue of nodes for breadth-first search
    std::queue<struct node<T>*> nodes;

    // distance between the nearest element found and the target
    unsigned dist = -1U;
    // current node
    auto& n = root_;


    while (!nodes.empty())
    {
        n = nodes.pop();

        // lock current node
        n.m.lock();
        // release previous node
        prev.unlock();

        for (const auto elem : n.elems)
        {
            int d = distance(elem, e);
            if (d < dist)
            {
                nearest = elem;
                dist = d;
            }
        }

        // optimization: element found, no need to go any further
        if (dist == 0)
        {
            ret.set_value(nearest);
            return ret.get_future();
        }

        prev = n.m;
        for (const auto* node : n.nodes)
        // TODO: insert a test to eliminate nodes outside of rande (> distance)
            nodes.push(node);
    }

    n.m.unlock();

    // element found near position
    if (dist != -1U)
        ret.set_value(nearest);

    return ret.get_future();
}

template <typename T>
std::future<void>
AsyncOptimizedQuadTree<T>::insert(const T e) noexcept
{
    // previous node's lock
    std::mutex prev;
    // return value
    std::promise<void> ret;

    // current node
    auto& n = root_;

    while (true)
    {
        // lock current node
        n.m.lock();
        // release previous node
        prev.unlock();

        // FIXME: need to descend further to check if point isn't set somewhere
        for (const auto elem : n.elems)
            // already in tree, so don't go any further
            if (e == elem)
            {
                n.m.unlock();
                return ret.get_future();
            }


        // free slot(s)
        if (n.free.any())
        {
            const int i = ffsll(n.free.to_ullong());
            n.elems[i] = e;
            n.free[i] = false;

            // value inserted
            n.m.unlock();
            return ret.get_future();
        }

        // find next node
        auto dir = get_dir(e, n.center);
        prev = n.m;

        auto pt = n.nodes[dir];
        if (pt != nullptr)
        {
            pt = std::make_unique<struct node<T>>();
            n.nodes[dir] = pt;
        }

        n = *pt;
    }

    return ret.get_future();
}

template <typename T>
std::future<void>
AsyncOptimizedQuadTree<T>::erase(const T e) noexcept
{
    // previous node's lock
    std::mutex prev;

    // return value
    std::promise<void> ret;
    // current node
    auto& n = root_;

    while (true)
    {
        // lock current node
        n.m.lock();
        // release previous node
        prev.unlock();

        int i;
        auto used = (~n.free).to_ullong();
        for (int pos = -1; i < N; used >>= i)
        {
            i = ffsll(used);
            if (e == n.elems[i])
            {
                n.free[i] = true;

                // value deleted
                n.m.unlock();
                return ret.get_future();
            }
        }

        // find next node
        auto dir = get_dir(e, n.center);
        prev = n.m;

        const auto pt = n.nodes[dir];
        // no point in the targeted corner, aborting
        if (pt != nullptr)
        {
            prev.unlock();
            return ret.get_future();
        }

        n = *pt;
    }

    return ret.get_future();
}
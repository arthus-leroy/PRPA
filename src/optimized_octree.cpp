# include <strings.h>
# include <queue>

# include "optimized_octree.hpp"

template <typename T, unsigned N>
node<T, N>::node()
    : free(-1ULL)
{}

template <typename T, unsigned N>
OptimizedOcTree<T, N>::OptimizedOcTree(const std::initializer_list<T>& init)
{
    this->init(init);
}

template <typename T, unsigned N>
void OptimizedOcTree<T, N>::init(const std::initializer_list<T>& init)
{
    for (const auto&  elem : init)
        insert(elem);
}

namespace
{
    inline long distance(const point a, const point b)
    {
        const long x = a.x - b.x;
        const long y = a.y - b.y;
        const long z = a.z - b.z;

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

template <typename T, unsigned N>
result_t OptimizedOcTree<T, N>::search(const T e) const noexcept
{
    // previous node's lock
    std::mutex prev;
    // nearest element found
    T nearest;
    // queue of nodes for breadth-first search
    std::queue<struct node<T, N>*> nodes;

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
            return nearest;
        }

        prev = n.m;
        for (const auto* node : n.nodes)
        // TODO: insert a test to eliminate nodes outside of rande (> distance)
            nodes.push(node);
    }

    n.m.unlock();

    // element found near position
    if (dist != -1U)
        return nearest;

    return nearest;
}

template <typename T, unsigned N>
void OptimizedOcTree<T, N>::insert(const T e) noexcept
{
    // previous node's lock
    std::mutex prev;

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
                return;
            }


        // free slot(s)
        if (n.free.any())
        {
            // ffsll return least significant bit, N - i is most significant
            const int i = ffsll(n.free.to_ullong());
            n.elems[N - i] = e;
            n.free[i - 1] = false;

            // value inserted
            n.m.unlock();
            return;
        }

        // find next node
        auto dir = get_dir(e, n.center);
        prev = n.m;

        auto pt = n.nodes[dir];
        if (pt != nullptr)
        {
            pt = std::make_unique<struct node<T, N>>();
            n.nodes[dir] = pt;
        }

        n = *pt;
    }
}

template <typename T, unsigned N>
void OptimizedOcTree<T, N>::erase(const T e) noexcept
{
    // previous node's lock
    std::mutex prev;

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
        for (int pos = -1; used; used >>= i)
        {
            // ffsll return least significant bit, N - i is most significant
            i = ffsll(used);
            pos += i;
            if (e == n.elems[N - pos])
            {
                n.free[pos - 1] = true;

                // value deleted
                n.m.unlock();
                return;
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
            return;
        }

        n = *pt;
    }
}
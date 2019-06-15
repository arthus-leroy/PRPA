# pragma once

# include <array>
# include <bitset>
# include <memory>
# include <cassert>
# include <strings.h>
# include <queue>

# include "IAsyncQuadTree.hpp"
# include "weighted_queue.hpp"

/** TODO:
 *  ask how max number of elements (for possiblity of array based implem)
 *  ask max size of cube (for size of sub-cubes, since a sub-cube)
 *  verify if fairness is ensured
 *  add center for (almost) every node declaration
 */

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

/**
 *  0: back  left  down
 *  1: back  right down
 *  2: back  left  up
 *  3: back  right up
 *  4: front left  down
 *  5: front right down
 *  6: front left  up
 *  7: front right up
 */
// size = 168 bytes
template <typename T, unsigned N>
struct node
{
    // TODO: investigate on the invalid read
    /**
     *  m of every node should exist as long a root_ exist, but each
     *  operator= of node produce an invalid read of mutex pretending mutex
     *  have been freed
     *  the same happend to nodes, but far fewer
     */
    inline node(struct point center_point = {0, 0, 0})
        : m(std::make_shared<std::mutex>()), free(-1ULL), center(center_point)
    {
        assert(N > 0);
    }

    /// mutex to block the node if busy
    std::shared_ptr<std::mutex> m;
    // TODO: test the trade-off 16x16x16, 8x8x8 or 4x4x4 compared to 2x2x2
    /// pointers to the 8 corners of the octotree
    std::array<std::shared_ptr<struct node>, 8> nodes;
    /// elements contained in the leaf
    std::array<T, N> elems;
    /// map of slots used in the node (1 = used)
    std::bitset<N> free;
    /// coordinate of the center of the sub-octree
    struct point center;
};

template <typename T, unsigned N>
class OptimizedOcTree : public IQuadTree
{
public:
    OptimizedOcTree() = default;
    inline OptimizedOcTree(const std::initializer_list<T>& init)
    {
        this->init(init);
    }

    inline void init(const std::vector<T>& init) noexcept
    {
        for (const auto& e : init)
            insert(e);
    }

    result_t search(const T e) const noexcept final
    {
        /// functor to sort the weighted queue
        static const auto f = [&](const T& p) { return distance(p, e); };

        // placeholder to avoid undefined behaviors
        std::mutex tmp;
        tmp.lock();
        /// previous node's lock
        std::mutex* prev = &tmp;
        /// nearest elements found
        WeightedQueue<T, K> nearest;
        /// queue of nodes for breadth-first search
        std::queue<const struct node<T, N>*> nodes;
        /// current node
        struct node<T, N> n;

        // current node
        nodes.push(&root_);

        while (!nodes.empty())
        {
            n = *nodes.front();
            nodes.pop();

            // lock current node
            n.m->lock();
            // release previous node
            prev->unlock();

            for (const auto elem : n.elems)
                nearest.push(elem, f);

            prev = n.m.get();
            for (const auto node : n.nodes)
            // TODO: insert a test to eliminate nodes outside of rande (> distance)
                if (node.get() != nullptr)
                    nodes.push(node.get());
        }

        n.m->unlock();

        return nearest.get();
    }

    void insert(const T e) noexcept final
    {
        // placeholder to avoid undefined behaviors
        std::mutex tmp;
        tmp.lock();
        /// previous node's lock
        std::mutex* prev = &tmp;

        /// current node
        auto& n = root_;

        while (true)
        {
            // lock current node
            n.m->lock();
            // release previous node
            prev->unlock();

            // FIXME: need to descend to check if point isn't set somewhere

            // free slot(s)
            if (n.free.any())
            {
                // ffsll span from 1 to N
                // ffsll return least significant bit, N - i is most significant
                const int i = ffsll(n.free.to_ullong());
                n.elems[N - i] = e;
                n.free[i - 1] = false;

                // value inserted
                n.m->unlock();
                return;
            }

            // find next node
            auto dir = get_dir(e, n.center);
            prev = n.m.get();

            auto pt = n.nodes[dir].get();
            if (pt == nullptr)
            {
                n.nodes[dir] = std::make_shared<node<T, N>>();
                pt = n.nodes[dir].get();
            }

            n = *pt;
        }
    }

    void erase(const T e) noexcept final
    {
        // placeholder to avoid undefined behaviors
        std::mutex tmp;
        tmp.lock();
        /// previous node's lock
        std::mutex* prev = &tmp;

        /// current node
        auto& n = root_;

        while (true)
        {
            // lock current node
            n.m->lock();
            // release previous node
            prev->unlock();

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
                    n.m->unlock();
                    return;
                }
            }

            // find next node
            auto dir = get_dir(e, n.center);
            prev = n.m.get();

            const auto pt = n.nodes[dir].get();
            // no point in the targeted corner, aborting
            if (pt == nullptr)
            {
                prev->unlock();
                return;
            }

            n = *pt;
        }
    }

private:
    struct node<T, N> root_;
};
# pragma once

# include <array>
# include <bitset>
# include <memory>
# include <cassert>
# include <strings.h>
# include <queue>
# include <iostream>

# include "IAsyncQuadTree.hpp"
# include "weighted_queue.hpp"

/** TODO:
 *  ask max number of elements (for possiblity of array based implem)
 *  verify if fairness is ensured (priority queue seems better option)
 *  use rw locks for readers (performance ++)
 *      -> queuing_rw_mutex of tbb/queuing_rw_mutex.h
 */

namespace
{
    static constexpr int16_t side_size = INT16_MAX;

    /// Get distance between two points
    inline long distance(const point a, const point b)
    {
        const long x = a.x - b.x;
        const long y = a.y - b.y;
        const long z = a.z - b.z;

        return x*x + y*y + z*z;
    }

    /// Return index of direction corresponding to corrdinates in a cube
    inline unsigned get_dir(const point a, const point b)
    {
        return 4 * ((a.z - b.z) >= 0)
             + 2 * ((a.y - b.y) >= 0)
             + 1 * ((a.x - b.x) >= 0);
    }

    // TODO: complete the function
    // FIXME: this function should greatly enhance the performances of the
    //        program. Without it, it performs at around 3 search per second.
    /// Get nearest point to point p in the cube centered on center
    inline std::size_t get_nearest(const point center, const point p,
                                   const std::size_t depth)
    {
        return 0;
    }

    /// Get the scalar of a
    inline long direction(const long a)
    {
        if (a > 0)
            return 1;
        else if (a < 0)
            return -1;

        return 0;
    }

    /// Get center of the cube the point p is in
    inline struct point get_center(const point center, const point p,
                                   const std::size_t depth)
    {
        return
        {
            center.x + direction(p.x - center.x) * side_size / depth / 2,
            center.y + direction(p.y - center.y) * side_size / depth / 2,
            center.z + direction(p.z - center.z) * side_size / depth / 2
        };
    }
}

// The class does not support concurrency, so should be used with care
// There *seems* to be no problem *for the moment*
class Lock
{
public:
    Lock()
        : mutex_(nullptr)
    {}

    Lock(std::mutex& m)
        : mutex_(&m)
    {
        m.lock();
    }

    Lock(std::mutex* m)
        : mutex_(m)
    {
        m->lock();
    }

    ~Lock()
    {
        if (mutex_ != nullptr)
            mutex_->unlock();
    }

    // move mutex (would be great to do it atomic)
    // could cause double unlock if exception raises between the 2 assignations
    Lock& operator=(Lock& m)
    {
        mutex_ = m.mutex_;
        m.mutex_ = nullptr;

        return *this;
    }

    // assign mutex
    Lock& operator=(std::mutex& m)
    {
        mutex_ = &m;
        return *this;
    }

    Lock* operator=(std::mutex* m)
    {
        mutex_ = m;
        return this;
    }

    void release()
    {
        assert(mutex_ != nullptr);

        mutex_->unlock();
        mutex_ = nullptr;
    }

private:
    std::mutex* mutex_;
};

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
    node(struct point center_point = {0, 0, 0})
        : m(new std::mutex)
        , free(-1ULL)
        , center(center_point)

    {}

    ~node()
    {
        delete m;
        for (unsigned i = 0; i < 8; i++)
            delete nodes[i];
    }

    /// mutex to block the node if busy
    mutable std::mutex* m;
    // TODO: test the trade-off 16x16x16, 8x8x8 or 4x4x4 compared to 2x2x2
    /// pointers to the 8 corners of the octree
    std::array<struct node*, 8> nodes{};
    /// elements contained in the leaf
    std::array<T, N> elems;
    /// map of slots used in the node (1 = used)
    std::bitset<N> free;
    /// coordinate of the center of the sub-octree
    const struct point center;
};

template <typename T>
struct node<T, 1>
{
    node(struct point center_point = {0, 0, 0})
        : m(new std::mutex)
        , free(true)
        , center(center_point)

    {}

    ~node()
    {
        delete m;
        for (unsigned i = 0; i < 8; i++)
            delete nodes[i];
    }

    /// mutex to block the node if busy
    mutable std::mutex* m;
    // TODO: test the trade-off 16x16x16, 8x8x8 or 4x4x4 compared to 2x2x2
    /// pointers to the 8 corners of the octree
    std::array<struct node*, 8> nodes{};
    /// element contained in the leaf
    T elem;
    /// boolean indicating whether or not, the node is used
    bool free;
    /// coordinate of the center of the sub-octree
    const struct point center;
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

        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);
        /// nearest elements found
        WeightedQueue<T, K> nearest;
        /// queue of nodes for breadth-first search
        std::queue<const struct node<T, N>*> nodes;
        // depth in the tree (indicate cube size)
        std::size_t depth = 0;

        // current node
        nodes.push(&root_);

//        std::cout << "Searching " << e << std::endl;

        while (!nodes.empty())
        {
            const auto& n = *nodes.front();
            nodes.pop();

            // lock current node
            Lock cur(n.m);
            // release previous lock
            prev.release();

            // if there is a point, add it
            if constexpr(N == 1)
            {
                if (n.free == false)
                    nearest.push(n.elem, f);
            }
            else
            {
                int i;
                auto used = (~n.free).to_ulong();
                for (int pos = -1; used; used >>= i)
                {
                    // ffsll return least significant bit, N - i is most significant
                    i = ffsl(used);
                    pos += i;
                    nearest.push(n.elems[N - pos], f);
                }
            }

            depth++;

            prev = cur;

            for (const auto node : n.nodes)
                // node exists and isn't too far away
                if (node != nullptr
                    && get_nearest(node->center, e, depth)
                     < nearest.back().weight)
                    nodes.push(node);
        }

        return nearest.get();
    }

    void insert(const T e) noexcept final
    {
        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);
        /// depth in the tree (indicate cube size)
        std::size_t depth = 0;

        /// current node's pointer
        auto* pt = &root_;

//        std::cout << "Inserting " << e << std::endl;

        while (true)
        {
            auto& n = *pt;

            // lock current node
            Lock cur(n.m);
            // release previous node
            prev.release();

            if constexpr(N == 1)
            {
                if (n.free == false && e == n.elem)
                    // value exists, aborting
                    return;
            }
            else
            {
                int i;
                auto used = (~n.free).to_ulong();
                for (int pos = -1; used; used >>= i)
                {
                    // ffsll return least significant bit, N - i is most significant
                    i = ffsl(used);
                    pos += i;
                    if (e == n.elems[N - pos])
                        // value exists, aborting
                        return;
                }
            }

            // find next node
            auto dir = get_dir(e, n.center);
            pt = n.nodes[dir];

            // free slot(s)
            if constexpr(N == 1)
            {
                if (n.free)
                {
                    // value isn't in the tree, inserting
                    if (descend(pt, e))
                    {
                        n.elem = e;
                        n.free = false;
                    }

                    // value inserted or already exists
                    return;
                }
            }
            else
            {
                if (n.free.any())
                {
                    // value isn't in the tree, inserting
                    if (descend(pt, e))
                    {
                        // ffsll span from 1 to N
                        // ffsll return least significant bit, N - i is most significant
                        const int i = ffsl(n.free.to_ulong());
                        n.elems[N - i] = e;
                        n.free[i - 1] = false;
                    }

                    // value inserted or already exists
                    return;
                }
            }

            prev = cur;

            depth++;

            if (pt == nullptr)
            {
                const auto center = get_center(n.center, e, depth);
                n.nodes[dir] = new struct node<T, N>(center);
                pt = n.nodes[dir];
            }
        }
    }

    void erase(const T e) noexcept final
    {
        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);

        /// current node's pointer
        auto* pt = &root_;

//        std::cout << "Erasing " << e << std::endl;

        // pt == nullptr => no point in the targeted corner
        while (pt)
        {
            auto& n = *pt;

            // lock current node
            Lock cur(n.m);

            prev.release();

            if constexpr(N == 1)
            {
                if (n.free == false && e == n.elem)
                {
                    n.free = true;

                    // value deleted
                    return;
                }
            }
            else
            {
                int i;
                auto used = (~n.free).to_ulong();
                for (int pos = -1; used; used >>= i)
                {
                    // ffsll return least significant bit, N - i is most significant
                    i = ffsl(used);
                    pos += i;
                    if (e == n.elems[N - pos])
                    {
                        n.free[pos - 1] = true;

                        // value deleted
                        return;
                    }
                }
            }

            prev = cur;

            // find next node
            auto dir = get_dir(e, n.center);
            pt = n.nodes[dir];
        }
    }

private:
    /// Descend to see if the value e don't exists in the nodes down
    bool descend(struct node<T, N>* node, const T e)
    {
        std::mutex tmp;
        Lock prev(tmp);

        while (node)
        {
            const auto &n = *node;
            Lock cur(n.m);
            prev.release();

            if constexpr(N == 1)
            {
                if (n.free == false && e == n.elem)
                    return false;
            }
            else
            {
                int i;
                auto used = (~n.free).to_ulong();
                for (int pos = -1; used; used >>= i)
                {
                    // ffsll return least significant bit, N - i is most significant
                    i = ffsl(used);
                    pos += i;
                    if (e == n.elems[N - pos])
                        return false;
                }
            }

            prev = cur;

            // find next node
            auto dir = get_dir(e, n.center);
            node = n.nodes[dir];
        }

        return true;
    }

    struct node<T, N> root_{};
};
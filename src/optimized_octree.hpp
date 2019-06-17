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
    // TODO: investigate on the invalid read
    /**
     *  m of every node should exist as long a root_ exist, but each
     *  operator= of node produce an invalid read of mutex pretending mutex
     *  have been freed
     *  the same happend to nodes, but far fewer
     */
    inline node(struct point center_point = {0, 0, 0})
        : m(std::make_shared<std::mutex>())
        , free(-1ULL)
        , center(center_point)
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

        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);
        /// nearest elements found
        WeightedQueue<T, K> nearest;
        /// queue of nodes for breadth-first search
        std::queue<const struct node<T, N>*> nodes;
        /// current node
        struct node<T, N> n;
        // depth in the tree (indicate cube size)
        std::size_t depth = 0;

        // current node
        nodes.push(&root_);

        while (!nodes.empty())
        {
            n = *nodes.front();
            nodes.pop();

            // lock current node
            Lock cur(n.m.get());

            // release previous node (warning assert on first try)
            prev.release();

            for (const auto elem : n.elems)
                nearest.push(elem, f);

            depth++;

            prev = cur;

            for (const auto node : n.nodes)
                // node exists and isn't too far away
                if (node.get() != nullptr
                    && get_nearest(node.get()->center, e, depth)
                     < nearest.back().weight)
                    nodes.push(node.get());
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

        /// current node
        auto& n = root_;

        while (true)
        {
            // lock current node
            Lock cur(n.m.get());

            // release previous node
            prev.release();

            // FIXME: need to descend to check if point isn't set somewhere
            // a solution could be perform a detection as soon as we find a
            // free spot, but could cause dupes being generated
            int i;
            auto used = (~n.free).to_ullong();
            for (int pos = -1; used; used >>= i)
            {
                // ffsll return least significant bit, N - i is most significant
                i = ffsll(used);
                pos += i;
                if (e == n.elems[N - pos])
                {
                    // value exists, aborting
                    return;
                }
            }

            // find next node
            auto dir = get_dir(e, n.center);
            auto pt = n.nodes[dir].get();

            // free slot(s)
            if (n.free.any())
            {
                // value isn't in the tree, inserting
                if (descend(pt, e))
                {
                    // ffsll span from 1 to N
                    // ffsll return least significant bit, N - i is most significant
                    const int i = ffsll(n.free.to_ullong());
                    n.elems[N - i] = e;
                    n.free[i - 1] = false;
                }

                // value inserted or already exists
                return;
            }

            prev = cur;

            depth++;

            if (pt == nullptr)
            {
                const auto center = get_center(n.center, e, depth);
                n.nodes[dir] = std::make_shared<node<T, N>>(center);
                pt = n.nodes[dir].get();
            }

            n = *pt;
        }
    }

    void erase(const T e) noexcept final
    {
        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);

        /// current node
        auto& n = root_;

        while (true)
        {
            // lock current node
            Lock cur(n.m.get());

            prev.release();

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
                    return;
                }
            }

            // find next node
            auto dir = get_dir(e, n.center);

            prev = cur;

            const auto pt = n.nodes[dir].get();
            // no point in the targeted corner, aborting
            if (pt == nullptr)
                return;

            n = *pt;
        }
    }

private:
    // TODO-LOCK: here too
    /// Descend to see if the value e don't exists in the nodes down
    bool descend(struct node<T, N>* node, T e)
    {
        std::mutex tmp;
        Lock prev(tmp);

        while (node != nullptr)
        {
            const auto n = *node;
            Lock cur(n.m.get());
            prev.release();

            int i;
            auto used = (~n.free).to_ullong();
            for (int pos = -1; used; used >>= i)
            {
                // ffsll return least significant bit, N - i is most significant
                i = ffsll(used);
                pos += i;
                if (e == n.elems[N - pos])
                    return false;
            }

            prev = cur;

            // find next node
            auto dir = get_dir(e, n.center);
            node = n.nodes[dir].get();
        }

        return true;
    }

    struct node<T, N> root_;
};
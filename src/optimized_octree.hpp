# pragma once

# include <array>
# include <bitset>
# include <memory>
# include <cassert>
# include <strings.h>
# include <queue>
# include <limits>
# include <iostream>
# include <algorithm>

# include "IAsyncQuadTree.hpp"
# include "weighted_queue.hpp"

/** TODO:
 *  ask max number of elements (for possiblity of array based implem)
 *      -> if the number is unknown, a solution would be to alloc the tree in an array
        -> problem: when you need to make make the array grow, you need to block all nodes
        -> problem: you can't use 2 writers at the same time
        -> could be a big optimization, but also a big flop
 *  verify if fairness is ensured (priority queue seems best option)
 *  use rw locks for readers (performance ++)
 *      -> queuing_rw_mutex of tbb/queuing_rw_mutex.h
 *  remove a node when the point is no longer of use (could be too much to
 *  implement)
 *
 ** WARNING:
 *  a node can be invalid if the node array realloc and thet node is still in
 *  an intermediary state, meaning there would be reference on a freed object
 */

namespace
{
    static constexpr int16_t side_size = INT16_MAX;

    /// Get distance between two points
    inline long distance(const struct point a, const struct point b)
    {
        const long x = a.x - b.x;
        const long y = a.y - b.y;
        const long z = a.z - b.z;

        return x*x + y*y + z*z;
    }

    /// Return index of direction corresponding to corrdinates in a cube
    inline unsigned get_dir(const struct point a, const struct point b)
    {
        return 4 * ((a.z - b.z) >= 0)
             + 2 * ((a.y - b.y) >= 0)
             + 1 * ((a.x - b.x) >= 0);
    }

    // TODO: complete the function
    /// Get nearest point to point p in the cube centered on center
    inline std::size_t get_nearest(const struct point center,
                                   const struct point p,
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
    inline struct point get_center(const struct point center,
                                   const struct point p,
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
    Lock() = default;

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
    std::mutex* mutex_ = nullptr;
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
// size = 96 + 8N bytes
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
    inline node()
        : m(std::make_shared<std::mutex>())
        , center({0, 0, 0})
    {
        static_assert(N > 0);
    }

    /// initialize (or reinitialize) a node with the values it needs
    inline void init(const struct point center_point = {0, 0, 0})
    {
        free = -1ULL;
        center = center_point;
    }

    /// mutex to block the node if busy
    std::shared_ptr<std::mutex> m{};
    // TODO: test the trade-off 16x16x16, 8x8x8 or 4x4x4 compared to 2x2x2
    /// pointers to the 8 corners of the octotree
    std::array<std::size_t, 8> nodes{};
    /// elements contained in the leaf
    std::array<T, N> elems{};
    /// map of slots used in the node (1 = used)
    std::bitset<N> free{-1ULL};
    /// coordinate of the center of the sub-octree
    struct point center{};
};

// could have used vector<bool>, but inneficient as hell for ffs
// weidly, only works for size_t
/// Implementation of dynamic bitset
template<typename T>
class Bitset
{
    /// Subclass to ease the initialization of values
    struct Type
    {
        T value = std::numeric_limits<T>::max();
    };

public:
    Bitset(const std::size_t base_size, const bool filled = false)
        : filled_(filled)
    {
        static_assert(sizeof(T) <= sizeof(std::size_t),
                     "type T can't be larger than size_t");
        static_assert(sizeof(T) > 0,
                      "type T must be able to contain something");

        resize(base_size);
    }

    void resize(const std::size_t size)
    {
        // ensures each cell is used to avoid some checks
        assert(size / cell_size_ * cell_size_ == size);
        array_.resize(size / cell_size_);
    }

    // getter
    bool operator[](const std::size_t i)
    {
        const unsigned x = i % cell_size_;
        const std::size_t y = i / cell_size_;

        return array_[y].value & 1ULL << (cell_size_ - x - 1);
    }

    // setter
    void set(const std::size_t i, const bool b)
    {        
        const unsigned x = i % cell_size_;
        const std::size_t y = i / cell_size_;

        if (b)
            array_[y].value |= 1ULL << (cell_size_ - x - 1);
        else
            array_[y].value &= ~(1ULL << (cell_size_ - x - 1));
    }

    bool none() const
    {
        for (size_t i = 0; i < array_.size(); i++)
            if (array_[i].value)
                return false;

        return true;
    }

    bool any() const
    {
        return !none();
    }

    bool all() const
    {
        for (size_t i = 0; i < array_.size(); i++)
            if (array_[i].value == max)
                return true;

        return false;
    }

    std::size_t msb() const
    {
        for (size_t i = 0; i < array_.size(); i++)
            if (array_[i].value != 0)
            {
                if constexpr(std::is_same<T, std::size_t>::value)
                    return i * cell_size_
                         + __builtin_clzll(array_[i].value);
                else
                    return i * cell_size_
                         + __builtin_clzll(static_cast<std::size_t>(array_[i].value));
            }

        return -1ULL;
    }

private:
    static constexpr unsigned cell_size_ = sizeof(T) * 8;
    static constexpr T max = std::numeric_limits<T>::max();

    bool filled_;
    std::vector<Type> array_;
};

template <typename T, unsigned N>
class OptimizedOcTree : public IQuadTree
{
public:
    OptimizedOcTree() = default;
    OptimizedOcTree(const std::initializer_list<T>& init)
        : OptimizedOcTree()
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
        std::queue<std::size_t> nodes;
        /// depth in the tree (indicate cube size)
        std::size_t depth = 0;

        // current node
        nodes.push(0);

        while (!nodes.empty())
        {
            auto n = nodes_[nodes.front()];
            nodes.pop();

            // lock current node
            Lock cur(n.m.get());

            // release previous node
            prev.release();

            for (auto elem : n.elems)
                nearest.push(elem, f);

            depth++;

            prev = cur;

            {
                std::scoped_lock lock(mutex_);
                for (auto node : n.nodes)
                    // node exists and isn't too far away
                    if (node != 0
                        && get_nearest(nodes_[node].center, e, depth)
                        < nearest.back().weight)
                        nodes.push(node);
            }
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
        /// current node index
        std::size_t index = 0;

//        std::cout << "Inserting " << e << std::endl;

        while (true)
        {
            auto n = nodes_[index];

            // lock current node
            Lock cur(n.m.get());

            // release previous node
            prev.release();

            int i;
            auto used = (~n.free).to_ullong();
            for (int pos = -1; used; used >>= i)
            {
                // ffsll return least significant bit, N - i is most significant
                i = ffsll(used);
                pos += i;
                if (e == n.elems[N - pos])
                    // value exists, aborting
                    return;
            }

            // find next node
            const auto dir = get_dir(e, n.center);
            const auto prev_index = index;
            index = n.nodes[dir];

            // free slot(s)
            if (n.free.any())
            {
                // value isn't in the tree, inserting
                if (descend(index, e))
                {
                    // ffsll span from 1 to N
                    // ffsll return least significant bit, N - i is most significant
                    int i = ffsl(n.free.to_ulong());
                    nodes_[prev_index].elems[N - i] = e;
                    nodes_[prev_index].free[i - 1] = false;
                }

//                std::cout << "Inserted (depth = " << depth << ", index = " << index << ")" << std::endl;

                // value inserted or already exists
                return;
            }

            prev = cur;

            depth++;

            if (index == 0)
            {
                auto center = get_center(n.center, e, depth);
                index = new_node(center);
                nodes_[prev_index].nodes[dir] = index;
            }
        }
    }

    void erase(const T e) noexcept final
    {
        std::mutex tmp;
        /// previous node's lock
        Lock prev(tmp);
        /// Current node index
        std::size_t index = 0;

        while (true)
        {
            auto n = nodes_[index];

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
                    nodes_[index].free[pos - 1] = true;

                    std::cout << "Deleted " << e << std::endl;
                    // value deleted
                    return;
                }
            }

            // find next node
            auto dir = get_dir(e, n.center);

            prev = cur;

            index = n.nodes[dir];
            // no point in the targeted corner, aborting
            if (index == 0)
                return;
        }
    }

private:
    /// get a new node in the available ones
    std::size_t new_node(const struct point center)
    {
        std::scoped_lock lock(mutex_);

        if (free_ + 1 == nodes_.size())
            nodes_.resize(nodes_.size() * array_growth);

        free_++;

        std::cout << free_ << std::endl;

        // FIXME: if an exception pop between here and the node is forever
        //        lost and unusable. *Could* be a problem.

        nodes_[free_].init(center);
        return free_;
    }

    /// Descend to see if the value e don't exists in the nodes down
    bool descend(std::size_t index, const T e)
    {
        std::mutex tmp;
        Lock prev(tmp);

        while (index != 0)
        {
            std::cout << "Index (descend) = " << index << std::endl;
            auto& n = nodes_[index];

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
            const auto dir = get_dir(e, n.center);
            index = n.nodes[dir];
        }

        return true;
    }

    using bitset_type = std::size_t;
    // number of nodes at beginning
    static constexpr std::size_t array_size = sizeof(bitset_type) * 8;
    // multiplier of array size each time the array is full
    static constexpr std::size_t array_growth = 2;

    // the mutable are here since the variables seems to be const (weirdly)
    std::vector<struct node<T, N>> nodes_{array_size};
    std::size_t free_ = 0;
    mutable std::mutex mutex_{};
};
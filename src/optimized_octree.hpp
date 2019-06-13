# pragma once

# include <array>
# include <bitset>
# include <memory>

# include "IAsyncQuadTree.hpp"

/** TODO:
 *  ask how max number of elements (for possiblity of array based implem)
 *  ask max size of cube (for size of sub-cubes, since a sub-cube)
 *  verify if fairness is ensured
 */

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
template <typename T, unsigned N>
struct node
{
    node();

    /// mutex to block the node if busy
    std::mutex m;
    // TODO: test the trade-off 16x16x16, 8x8x8 or 4x4x4 compared to 2x2x2
    /// pointers to the 8 corners of the octotree
    std::array<std::unique_ptr<node*>, 8> nodes;
    /// elements contained in the leaf
    std::array<T, N> elems;
    /// map of slots used in the node (1 = used)
    std::bitset<N> free;
    /// coordinate of the center of the sub-octree
    struct point center;
};

template <typename T, unsigned N>
class OptimizedOcTree : IQuadTree
{
public:
    OptimizedOcTree(const std::initializer_list<T>& init = {});

    void init(const std::initializer_list<T>& init);
    result_t search(const T elem) const noexcept final;
    void insert(const T elem) noexcept final;
    void erase(const T elem) noexcept final;

private:
    struct node<T, N> root_;
};
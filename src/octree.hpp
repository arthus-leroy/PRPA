# include "optimized_octree.hpp"
# include "optimized_async_octree.hpp"

// SYNC OCTREE
template <unsigned N>
using SyncOctree = OptimizedOcTree<point, N>;

// ASYNC OCTREE
template <unsigned N>
using AsyncOctree = OptimizedOcTree<point, N>;
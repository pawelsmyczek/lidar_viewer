#ifndef LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H
#define LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H

#include "PointCloud.h"
#include "Box.h"
#include "Neighbour.h"
#include "Octree.h"
#include "lidar_viewer/geometry/functions/Utilities.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <type_traits>
#include <utility>

namespace lidar_viewer::geometry::types
{

/// value or nothing, placeholder until a proper error type is introduced
template <typename T>
using ErrorOr = std::optional<T>; // temporary

/// Octree indexing a point cloud. Nodes are bounding boxes, node containers hold the indices
/// (see Indices) of the points that fall into them.
/// The octree keeps a reference to the point cloud, which must outlive it.
template <typename PointType>
struct OctreeFromPointCloud
        : public Octree<Indices, Box<PointType>>
{
    using Base = Octree<Indices, Box<PointType>>;
    /// creates an empty octree (root only) covering the bounding box of the point cloud
    explicit OctreeFromPointCloud(const PointCloud<PointType>& pointCloud_)
    : Base(functions::calculateBoundingBoxFromPointCloud(pointCloud_))
    , pointCloud{pointCloud_}
    { }

    /// creates an octree covering the bounding box of the point cloud
    /// @param depth maximal depth, see Octree::createNodesRecursivelyAt
    /// @param prefill when true all points of the cloud are inserted right away
    OctreeFromPointCloud(const PointCloud<PointType>& pointCloud_, const size_t depth, bool prefill = true)
    : Base(functions::calculateBoundingBoxFromPointCloud(pointCloud_), depth)
    , pointCloud{pointCloud_}
    {
        if(!prefill)
        {
            return ;
        }
        fillWithPointCloud();
    }

    /// inserts point number `index` of the point cloud, creating the nodes leading to it when needed
    /// @return the node holding the index
    Base::NodeType* insert(const size_t index)
    {
        const auto point = pointCloud[index];
        auto comparisonFunction = [&point](const Box<PointType>& box, size_t i, bool divide) -> ErrorOr<Box<PointType>>
        {
            auto dividedBox = divide ? functions::subdivisionOfBounbdingBox(box, i) : box;
            if(!dividedBox.contains(point))
            {
                return std::nullopt;
            }
            // comparison against the nodes
            return dividedBox;
        };
        auto retNode = createNodesRecursivelyAt<decltype(comparisonFunction)>(this->root, comparisonFunction, this->getKey(), this->depth);
        retNode->getContainer().emplace_back(index);
        return retNode;
    }

    /// inserts every point of the point cloud
    void fillWithPointCloud()
    {
        for (size_t id = 0u; id < pointCloud.size(); ++id)
        {
            insert(id);

        }
    }

    /// the point cloud the octree indexes
    const PointCloud<PointType>& getPointCloud() const
    {
        return pointCloud;
    }

    /// Finds the point of the cloud which is the closest to `query`, among the points already
    /// inserted into the octree (all of them after the constructor with `prefill`).
    /// Branch and bound: a subtree is skipped when its box is farther than the best point found so
    /// far, and the children are visited nearest first. The query may lie outside of the octree.
    /// @param query the point to search for
    /// @param maxDistance only points within this distance (inclusive) are considered
    /// @return the nearest point, or nothing when the octree is empty or no point is close enough
    [[nodiscard]] std::optional<Neighbour<typename PointType::value_type>>
    nearest(const PointType& query,
            const typename PointType::value_type maxDistance
                = std::numeric_limits<typename PointType::value_type>::infinity()) const
    {
        using CoordType = typename PointType::value_type;
        static_assert(std::is_floating_point_v<CoordType>, "nearest neighbour search needs floating point coordinates");
        if (maxDistance < CoordType{0})
        {
            return std::nullopt;
        }
        CoordType bestSquared = maxDistance * maxDistance;
        std::optional<size_t> bestIndex;
        nearestIn(*this->getRootNode(), query, bestSquared, bestIndex);
        if (!bestIndex)
        {
            return std::nullopt;
        }
        return Neighbour<CoordType>{*bestIndex, std::sqrt(bestSquared)};
    }

private:
    /// Searches the node and its subtree, `bestSquared` and `bestIndex` hold the best point so far.
    void nearestIn(const typename Base::NodeType& node, const PointType& query,
                   typename PointType::value_type& bestSquared, std::optional<size_t>& bestIndex) const
    {
        using CoordType = typename PointType::value_type;

        for (const auto index : node.getContainer())
        {
            const auto difference = pointCloud[index] - query;
            const auto distanceSquared = std::inner_product(difference.data(), difference.data() + PointType::Dim,
                                                            difference.data(), CoordType{0});
            // a point exactly at the limit counts as long as nothing was found yet
            if (distanceSquared < bestSquared || (!bestIndex && distanceSquared <= bestSquared))
            {
                bestSquared = distanceSquared;
                bestIndex = index;
            }
        }

        std::array<std::pair<CoordType, const typename Base::NodeType*>, 8> children{};
        size_t count = 0;
        for (size_t i = 0; i < 8; ++i)
        {
            if (const auto* child = node[i]; child != nullptr)
            {
                children[count++] = {child->getKey().distanceSquared(query), child};
            }
        }
        std::sort(children.begin(), children.begin() + count,
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (size_t i = 0; i < count; ++i)
        {
            if (children[i].first > bestSquared)
            {
                break;   // sorted, so all the remaining children are farther as well
            }
            nearestIn(*children[i].second, query, bestSquared, bestIndex);
        }
    }

    const PointCloud<PointType>& pointCloud;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H

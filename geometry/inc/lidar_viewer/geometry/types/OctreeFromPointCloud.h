#ifndef LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H
#define LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H

#include "PointCloud.h"
#include "Box.h"
#include "Octree.h"
#include "lidar_viewer/geometry/functions/Utilities.h"

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
private:
    const PointCloud<PointType>& pointCloud;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H

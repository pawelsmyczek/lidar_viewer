#ifndef LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H
#define LIDAR_VIEWER_OCTREEFROMPOINTCLOUD_H

#include "PointCloud.h"
#include "Box.h"
#include "Octree.h"
#include "lidar_viewer/geometry/functions/Utilities.h"

namespace lidar_viewer::geometry::types
{

template <typename T>
using ErrorOr = std::optional<T>; // temporary

template <typename PointType>
struct OctreeFromPointCloud
        : public Octree<Indices, Box<PointType>>
{
    using Base = Octree<Indices, Box<PointType>>;
    explicit OctreeFromPointCloud(const PointCloud<PointType>& pointCloud_)
    : Base(functions::calculateBoundingBoxFromPointCloud(pointCloud_))
    , pointCloud{pointCloud_}
    { }

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

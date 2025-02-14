#ifndef LIDAR_VIEWER_DOWNSAMPLE_H
#define LIDAR_VIEWER_DOWNSAMPLE_H

#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/PointCloud.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace lidar_viewer::geometry::functions
{

template <typename CoordType>
types::PointCloud3D<CoordType> downSample(const types::PointCloud3D<CoordType>& pointCloud, float voxelSize)
{
    if (pointCloud.size() < 7)
    {
        return {};
    }
    const auto [xMin, xMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
      [](const types::Point3D<CoordType>& point1, const types::Point3D<CoordType>& point2)
        {
            return point1[0] < point2[0];
        });
    const auto [yMin, yMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
      [](const types::Point3D<CoordType>& point1, const types::Point3D<CoordType>& point2)
        {
            return point1[1] < point2[1];
        });
    const auto [zMin, zMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
      [](const types::Point3D<CoordType> & point1, const types::Point3D<CoordType>& point2)
        {
            return point1[2] < point2[2];
        });

    const auto voxelsSizeX = std::ceil(std::abs((*xMax)[0] - (*xMin)[0]) / voxelSize);
    const auto voxelsSizeY = std::ceil(std::abs((*yMax)[1] - (*yMin)[1]) / voxelSize);
    const auto voxelsSizeZ = std::ceil(std::abs((*zMax)[2] - (*zMin)[2]) / voxelSize);

    types::PointCloud3D<CoordType> voxels;
    voxels.resize ( voxelsSizeX * voxelsSizeY * voxelsSizeZ );
    std::vector<size_t> voxelsCounts;
    voxelsCounts.resize ( voxelsSizeX * voxelsSizeY * voxelsSizeZ );

    for (const auto & point : pointCloud)
    {
        const auto xFloored = std::floor((point[0] - (*xMin)[0]) / voxelSize);
        const auto yFloored = std::floor((point[1] - (*yMin)[1]) / voxelSize);
        const auto zFloored = std::floor((point[2] - (*zMin)[2]) / voxelSize);
        const auto id = xFloored + voxelsSizeX *  (yFloored + voxelsSizeY * zFloored);
        if(id >= voxelsSizeX * voxelsSizeY * voxelsSizeZ)
        {
            continue;
        }
        voxels[id] += point;
        voxelsCounts[id] += 1u;
    }

    types::PointCloud3D<CoordType> ret{};
    for( auto i = 0u; i < voxelsSizeX; ++i )
    {
        for( auto j = 0u; j < voxelsSizeY; ++j )
        {
            for( auto k = 0u; k < voxelsSizeZ; ++k )
            {
                const auto id = i + voxelsSizeX *  (j + voxelsSizeY * k);
//                auto point = voxels[id] / voxelsCounts[id];
                if(id >= voxelsSizeX * voxelsSizeY * voxelsSizeZ)
                {
                    continue;
                }
                if(voxelsCounts[id] == 0)
                {
                    continue;
                }
                auto tmp = voxels[id] / voxelsCounts[id];
                ret.emplace_back(tmp);
            }
        }

    }
    return ret;
}

} // namespace lidar_viewer::geometry::functions


#endif //LIDAR_VIEWER_DOWNSAMPLE_H

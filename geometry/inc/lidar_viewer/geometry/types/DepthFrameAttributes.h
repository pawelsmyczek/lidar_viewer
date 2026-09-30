#ifndef LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H
#define LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H

#include "ScreenRanges.h"

namespace lidar_viewer::geometry::types
{

/// Describes a depth frame delivered by a depth sensor and its field of view.
struct DepthFrameAttributes
{
    /// @param frameResolution_ frame size in pixels as {width, height}
    /// @param depthRange_ range of valid depth values as {min, max}, values outside of it are discarded
    /// @param rotationX_ horizontal field of view in degrees
    /// @param rotationY_ vertical field of view in degrees
    constexpr DepthFrameAttributes(UintRange frameResolution_,
                                   UintRange depthRange_,
                                   float rotationX_, float rotationY_)
            : frameResolution { frameResolution_ }
            , depthRange { depthRange_ }
            , rotationX { rotationX_ }
            , rotationY { rotationY_ }
    {}
    UintRange frameResolution; ///< {width, height} in pixels
    UintRange depthRange;      ///< {min, max} valid depth value
    float rotationX;           ///< horizontal field of view, degrees
    float rotationY;           ///< vertical field of view, degrees
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H

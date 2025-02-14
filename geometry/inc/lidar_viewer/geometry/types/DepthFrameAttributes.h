#ifndef LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H
#define LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H

#include "ScreenRanges.h"

namespace lidar_viewer::geometry::types
{

struct DepthFrameAttributes
{
    constexpr DepthFrameAttributes(UintRange frameResolution_,
                                   UintRange depthRange_,
                                   float rotationX_, float rotationY_)
            : frameResolution { frameResolution_ }
            , depthRange { depthRange_ }
            , rotationX { rotationX_ }
            , rotationY { rotationY_ }
    {}
    UintRange frameResolution   ;
    UintRange depthRange;
    float rotationX;
    float rotationY;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_DEPTHFRAMEATTRIBUTES_H

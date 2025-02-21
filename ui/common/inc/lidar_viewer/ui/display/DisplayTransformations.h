#ifndef LIDAR_VIEWER_DISPLAYTRANSFORMATIONS_H
#define LIDAR_VIEWER_DISPLAYTRANSFORMATIONS_H

#include <atomic>

namespace lidar_viewer
{
namespace ui
{

struct ViewManager;

struct TransformationParameters
{
    TransformationParameters(int rx, int ry, int rz, float scale)
    : rotx{rx}
    , roty{ry}
    , rotz{rz}
    , windowScale{scale}
    {
    }

    TransformationParameters(const TransformationParameters&) = delete;
    TransformationParameters& operator = (const TransformationParameters&) = delete;
    TransformationParameters(TransformationParameters&&) = delete;
    TransformationParameters& operator = (TransformationParameters&&) = delete;

    std::atomic<int> rotx{};
    std::atomic<int> roty{};
    std::atomic<int> rotz{};
    float windowScale{};
};



} // ui
} // lidar_viewer

#endif //LIDAR_VIEWER_DISPLAYTRANSFORMATIONS_H

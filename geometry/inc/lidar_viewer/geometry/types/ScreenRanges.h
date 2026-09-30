#ifndef LIDAR_VIEWER_SCREENRANGES_H
#define LIDAR_VIEWER_SCREENRANGES_H

#include <utility>

namespace lidar_viewer::geometry::types
{

/// pair of values {first, second}, describing where a coordinate starts and ends
template <typename T>
using Range = std::pair<T, T>;

using UintRange = Range<unsigned int>;
using FloatRange = Range<float>;

/// Interface describing the coordinate ranges of the target screen (display space).
/// Keeps the geometry code independent from any concrete graphics API.
/// A range may be reversed (first > second) to flip an axis.
struct ScreenRanges
{
    virtual ~ScreenRanges() = default;

    /// range of the horizontal axis
    [[nodiscard]] virtual FloatRange fullRangeX() const = 0;
    /// range of the vertical axis
    [[nodiscard]] virtual FloatRange fullRangeY() const = 0;
    /// range of the depth axis
    [[nodiscard]] virtual FloatRange fullRangeZ() const = 0;
};

/// Screen ranges of the OpenGL normalized device coordinates:
/// X in [-1, 1], Y from 1 (top) to -1 (bottom), Z from 1 (near) to 0.
struct ScreenRangeGl
        : public ScreenRanges
{
    [[nodiscard]] FloatRange fullRangeX() const override
    {
        return glFullScreenRangeX;
    }

    [[nodiscard]] FloatRange fullRangeY() const override
    {
        return glFullScreenRangeY;
    }

    [[nodiscard]] FloatRange fullRangeZ() const override
    {
        return glFullScreenRangeZ;
    }

private:
    static constexpr FloatRange glFullScreenRangeX {-1.f, 1.f};
    static constexpr FloatRange glFullScreenRangeY {1.f, -1.f};
    static constexpr FloatRange glFullScreenRangeZ {1.f, .0f};
};
} // namespace lidar_viewer::geometry::types


#endif //LIDAR_VIEWER_SCREENRANGES_H

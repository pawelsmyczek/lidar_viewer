#ifndef LIDAR_VIEWER_SCREENRANGES_H
#define LIDAR_VIEWER_SCREENRANGES_H

#include <utility>

namespace lidar_viewer::geometry::types
{

template <typename T>
using Range = std::pair<T, T>;

using UintRange = Range<unsigned int>;
using FloatRange = Range<float>;

struct ScreenRanges
{
    [[nodiscard]] virtual FloatRange fullRangeX() const = 0;
    [[nodiscard]] virtual FloatRange fullRangeY() const = 0;
    [[nodiscard]] virtual FloatRange fullRangeZ() const = 0;
};

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

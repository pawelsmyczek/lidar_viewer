#ifndef LIDAR_VIEWER_SCREENPARAMETERS_H
#define LIDAR_VIEWER_SCREENPARAMETERS_H

#include <functional>

namespace lidar_viewer::ui
{

template<typename CoordType>
struct ScreenParameters
{
    CoordType x, y, w, h;
};

template <typename CoordType>
using ScreenParametersGetter = std::function<void(ScreenParameters<CoordType>&)>;

using ScreenParametersGetterInt = ScreenParametersGetter<int>;

}// namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_SCREENPARAMETERS_H

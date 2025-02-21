#ifndef LIDAR_VIEWER_DISPLAYSTATISTICS_H
#define LIDAR_VIEWER_DISPLAYSTATISTICS_H

#include "lidar_viewer/ui/drawing/DrawingFunctions.h"
#include "lidar_viewer/ui/window/ScreenParameters.h"

namespace lidar_viewer::ui::display
{

bool displayStatistics(const lidar_viewer::ui::drawing::DrawStdStringColorFloatArr&,
                            const lidar_viewer::ui::ScreenParametersGetterInt&);

} // namespace lidar_viewer::ui::display

#endif //LIDAR_VIEWER_DISPLAYSTATISTICS_H

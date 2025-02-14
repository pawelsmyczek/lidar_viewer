#ifndef LIDAR_VIEWER_DRAWPOINT_H
#define LIDAR_VIEWER_DRAWPOINT_H

#include "lidar_viewer/geometry/types/Point.h"

#include <GL/glew.h>

namespace lidar_viewer::ui
{

template <typename CoordType>
void drawPoint(const geometry::types::Point3D<CoordType>& point)
{
    glBegin(GL_POINTS);
    glVertex3fv( point.data() );
    glEnd();
}

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_DRAWPOINT_H

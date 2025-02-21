#ifndef LIDAR_VIEWER_DRAWPOINT_H
#define LIDAR_VIEWER_DRAWPOINT_H

#include "lidar_viewer/geometry/types/Point.h"

#include <array>
#include <GL/glew.h>

namespace lidar_viewer::ui::drawing
{

template <typename CoordType>
void drawPoint(const geometry::types::Point3D<CoordType>& point, const std::array<float, 3>& color)
{
    glColor3fv( color.data() );
    glBegin(GL_POINTS);
    glVertex3fv( point.data() );
    glEnd();
}

template <typename CoordType>
void drawPointByteColored(const geometry::types::Point3D<CoordType>& point, const std::array<uint8_t, 3>& color)
{
    glColor3ubv( color.data() );
    glBegin(GL_POINTS);
    glVertex3fv( point.data() );
    glEnd();
}

} // namespace lidar_viewer::ui::drawing

#endif //LIDAR_VIEWER_DRAWPOINT_H

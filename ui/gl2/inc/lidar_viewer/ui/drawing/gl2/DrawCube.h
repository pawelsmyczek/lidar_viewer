#ifndef LIDAR_VIEWER_DRAWCUBE_H
#define LIDAR_VIEWER_DRAWCUBE_H

#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/Box.h"

#include <array>
#include <GL/glew.h>

namespace lidar_viewer::ui::drawing
{

template <typename CoordType>
using Box3D = lidar_viewer::geometry::types::Box<lidar_viewer::geometry::types::Point3D<CoordType>>;

template <typename CoordType>
void drawCube(const Box3D<CoordType>& box, const std::array<float, 3>& color)
{
    glColor3fv( color.data() );
    glBegin(GL_LINE_LOOP);
    // Front face (y = 1.0f)
    glVertex3fv(box.hi.data());
    glVertex3f(box.hi[0], box.lo[1], box.hi[2]);
    glVertex3f(box.lo[0], box.lo[1], box.hi[2]);
    glVertex3f(box.lo[0], box.hi[1], box.hi[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    // Back face (y = -1.0f)
    glVertex3f(box.hi[0], box.hi[1], box.lo[2]);
    glVertex3f(box.hi[0], box.lo[1], box.lo[2]);
    glVertex3fv(box.lo.data());
    glVertex3f(box.lo[0], box.hi[1], box.lo[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    // Top face  (z = 1.0f)
    glVertex3fv(box.hi.data());
    glVertex3f(box.lo[0], box.hi[1], box.hi[2]);
    glVertex3f(box.lo[0], box.hi[1], box.lo[2]);
    glVertex3f(box.hi[0], box.hi[1], box.lo[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
// Bottom face (z = -1.0f)
    glVertex3f(box.hi[0], box.lo[1], box.hi[2]);
    glVertex3f(box.lo[0], box.lo[1], box.hi[2]);
    glVertex3fv(box.lo.data());
    glVertex3f(box.hi[0], box.lo[1], box.lo[2]);
    glEnd();
}

} // namespace lidar_viewer::ui::drawing

#endif //LIDAR_VIEWER_DRAWCUBE_H

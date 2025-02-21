#include "lidar_viewer/ui/display/gl2/DisplayManagerGl.h"
#include "lidar_viewer/ui/window/ViewManager.h"
#include "lidar_viewer/ui/display/DisplayTransformations.h"

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

namespace transformations
{

void scaleUp(lidar_viewer::ui::TransformationParameters& ref)
{
    ref.windowScale+=.1f;
}
void scaleDown(lidar_viewer::ui::TransformationParameters& ref)
{

    ref.windowScale=ref.windowScale > .0f? ref.windowScale-.1f:.0f;
}
void rotateUp(lidar_viewer::ui::TransformationParameters& ref)
{
    --ref.rotx;

}
void rotateDown(lidar_viewer::ui::TransformationParameters& ref)
{

    ++ref.rotx;
}
void rotateLeft(lidar_viewer::ui::TransformationParameters& ref)
{
    --ref.roty;

}
void rotateRight(lidar_viewer::ui::TransformationParameters& ref)
{
    ++ref.roty;

}
void reset(lidar_viewer::ui::TransformationParameters& ref)
{
    ref.rotx = 0;
    ref.roty = 0;
    ref.windowScale = 1.f;
}

} // namespace transformations

namespace
{

lidar_viewer::ui::DisplayManagerGl* viewerPtr = nullptr;

void setPtr(lidar_viewer::ui::DisplayManagerGl* viewer)
{
    if(!viewerPtr)
    {
        viewerPtr = viewer;
    }
}

void disp()
{
    if(!viewerPtr)
    {
        return ;
    }
    viewerPtr->display();
}

void reshape(int w, int h)
{
    glViewport( 0, 0, w, h );

    // chose projection matrix
    glMatrixMode( GL_PROJECTION );

    // preojection unit matrix
    glLoadIdentity();
    // disp();
}

void keyboard(unsigned char key, int , int)
{
    switch ( key )
    {
        case '=':
            transformations::scaleUp(viewerPtr->getDisplayTransforms());
            break;
        case '-':
            transformations::scaleDown(viewerPtr->getDisplayTransforms());
            break;
        case 'b':
            transformations::reset(viewerPtr->getDisplayTransforms());
            break;
        case '1':
            viewerPtr->toggleFunction(lidar_viewer::ui::DisplayManagerBase::ViewType::Flat);
            break;
        case '2':
            viewerPtr->toggleFunction(lidar_viewer::ui::DisplayManagerBase::ViewType::PointCloud);
            break;
        case '3':
            viewerPtr->toggleFunction(lidar_viewer::ui::DisplayManagerBase::ViewType::Octree);
            break;
        case 's':
            viewerPtr->toggleFunction(lidar_viewer::ui::DisplayManagerBase::ViewType::Statistics);
            break;

        default:
            break;
    }
    reshape( glutGet( GLUT_WINDOW_WIDTH ), glutGet( GLUT_WINDOW_HEIGHT ) );
}

void special(int key, int , int)
{
    auto & displayTransforms = viewerPtr->getDisplayTransforms();
    switch( key )
    {
        case GLUT_KEY_LEFT:
            transformations::rotateLeft(displayTransforms);
            break;
        case GLUT_KEY_UP:
            transformations::rotateUp(displayTransforms);
            break;
        case GLUT_KEY_RIGHT:
            transformations::rotateRight(displayTransforms);
            break;
        case GLUT_KEY_DOWN:
            transformations::rotateDown(displayTransforms);
            break;
        default:
            break;
    }
    reshape(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
}

}

namespace lidar_viewer::ui
{

void DisplayFunctionGuardGl::preDisplay()
{
    glClearColor( 0.f, 0.f, 0.f, 0.f );
    glClear( GL_COLOR_BUFFER_BIT );

    // choose modelling matrix
    glMatrixMode( GL_MODELVIEW );

    // identity matrix - load unit matrix
    glLoadIdentity();

    glPushMatrix();
}

void DisplayFunctionGuardGl::postDisplay()
{
    glPopMatrix();
    glFlush();
}

DisplayManagerGl::DisplayManagerGl(ViewManager& ref, std::chrono::milliseconds sync)
: DisplayManager(ref, sync)
, transformationParameters{0, 0, 0, 1.f}
{
    setPtr(this);
}

void DisplayManagerGl::start()
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutReshapeFunc( reshape );
    glutSpecialFunc( special );
    glutKeyboardFunc( keyboard );
    glutDisplayFunc( disp );
    glutMainLoop();
}

std::unique_ptr<DisplayFunctionGuard> DisplayManagerGl::makeDisplayGuard(ViewManager& viewManager)
{
    return std::make_unique<DisplayFunctionGuardGl>([&viewManager]()
    {
        // glutDestroyWindow invalidates context,
        // so let's not create room for issue here after stop() call
        if(!viewManager.isStopped())
        {
            glutSwapBuffers();
            glutPostRedisplay();
        }
    });
}

void DisplayManagerGl::workOnRegisteredFunction(const std::pair<ViewType, std::function<bool()>> & pair,
                                                           ViewManager& view)
{
    if(pair.first <= ViewType::Octree) // scale / rotate everything but the text
    {
        const auto& displayParameters = getDisplayTransforms();
        glScalef( displayParameters.windowScale,
                  displayParameters.windowScale,
                  displayParameters.windowScale );

        glRotatef(static_cast<float>(displayParameters.rotx.load()), 1.f, 0.f, 0.f);
        glRotatef(static_cast<float>(displayParameters.rotx.load()), 0.f, 1.0, 0.f );
    }

    if(!pair.second())
    {
        glPopMatrix();
        view.stop();
    }
}

TransformationParameters &DisplayManagerGl::getDisplayTransforms()
{
    return transformationParameters;
}

const TransformationParameters &DisplayManagerGl::getDisplayTransforms() const
{
    return transformationParameters;
}

} // namespace lidar_viewer::ui

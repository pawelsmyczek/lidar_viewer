#include "lidar_viewer/ui/Viewer.h"

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <iostream>

namespace
{

lidar_viewer::ui::Viewer* viewerPtr = nullptr;

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
            viewerPtr->scaleUp();
            break;
        case '-':
            viewerPtr->scaleDown();
            break;
        case 'b':
            viewerPtr->zeroWindowTransforms();
            break;
        default:
            break;
    }
    reshape( glutGet( GLUT_WINDOW_WIDTH ), glutGet( GLUT_WINDOW_HEIGHT ) );
}

void special(int key, int , int)
{
    switch( key )
    {
        case GLUT_KEY_LEFT:
            viewerPtr->rotateLeft();
            break;
        case GLUT_KEY_UP:
            viewerPtr->rotateUp();
            break;
        case GLUT_KEY_RIGHT:
            viewerPtr->rotateRight();
            break;
        case GLUT_KEY_DOWN:
            viewerPtr->rotateDown();
            break;
        default:
            break;
    }
    reshape(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
}

}

namespace lidar_viewer::ui
{

Viewer::Viewer(Config configuration_) noexcept
    : configuration{configuration_}, rotx{}, roty{}, windowScale{1.f}, windowId{}, stopped{false}
{
    if(!viewerPtr)
    {
        viewerPtr = this;
    }
}

Viewer::~Viewer() noexcept = default;

void Viewer::start(int* argc, char** argv)
{
    glutInit(argc, argv);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION); // continue execution ofter window close to cleanup properly
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(static_cast<int>(configuration.w),
                       static_cast<int>(configuration.h));
    windowId = glutCreateWindow("LidarViewer");
    glutDisplayFunc( disp );
    glutReshapeFunc( reshape );
    glutSpecialFunc( special );
    glutKeyboardFunc( keyboard );
    glutMainLoop();
}

void Viewer::stop()
{
    std::cout << "trying to destroy window : " << windowId << '\n';
    stopped = true;
    glutDestroyWindow(windowId);
}

void Viewer::display()
{

    glClearColor( 0.f, 0.f, 0.f, 0.f );
    glClear( GL_COLOR_BUFFER_BIT );

    // choose modelling matrix
    glMatrixMode( GL_MODELVIEW );

    // identity matrix - load unit matrix
    glLoadIdentity();

    glScalef( windowScale, windowScale, windowScale );

    glRotatef(static_cast<float>(rotx), 1.f, 0.f, 0.f);
    glRotatef(static_cast<float>(roty), 0.f, 1.0, 0.f );

    for( auto& func: viewerFuncitons )
    {
        if(!func.worker || !func.function)
        {
            continue;
        }
        glPushMatrix();
        if(!func.function(func.worker))
        {
            glPopMatrix();
            stop();
        }
        glPopMatrix();
    }
    glFlush();
    // glutDestroyWindow invalidates context,
    // so let's not create room for issue here after close() call
    if(!isStopped())
    {
        glutSwapBuffers();
        glutPostRedisplay();
    }
}

void Viewer::rotateUp() noexcept
{
    --rotx;
}

void Viewer::rotateDown() noexcept
{
    ++rotx;
}

void Viewer::rotateLeft() noexcept
{
    --roty;
}

void Viewer::rotateRight() noexcept
{
    ++roty;
}

void Viewer::scaleUp() noexcept
{
    windowScale+=.1f;
}

void Viewer::scaleDown() noexcept
{
    windowScale=windowScale > .0f? windowScale-.1f:.0f;
}

void Viewer::zeroWindowTransforms() noexcept
{
    rotx = 0; roty = 0; windowScale = 1.f;
}

bool Viewer::isStopped() const noexcept
{
    return stopped;
}

}

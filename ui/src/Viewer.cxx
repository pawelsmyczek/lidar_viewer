#include "Viewer.h"

#include <GL/glew.h>
#include <GL/glut.h>

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
    disp();
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
        case 'q':
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

Viewer::Viewer(Config configuration_,
               int* argc, char** argv) noexcept
    : configuration{configuration_}, rotx{}, roty{}, windowScale{1.f}
{
    if(!viewerPtr)
    {
        viewerPtr = this;
    }
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA |GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(configuration.w, configuration.h);
    glutCreateWindow("LidarViewer");
}

void Viewer::start() const
{
    glutDisplayFunc( disp );
    glutReshapeFunc( reshape );
    glutSpecialFunc( special );
    glutKeyboardFunc( keyboard );
    glutMainLoop();
}

void Viewer::display()
{
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear( GL_COLOR_BUFFER_BIT );

    // choose modelling matrix
    glMatrixMode( GL_MODELVIEW );

    // identity matrix - load unit matrix
    glLoadIdentity();

    glScalef( windowScale, windowScale, windowScale );

    glRotatef( rotx, 1.0, 0, 0 );
    glRotatef( roty, 0, 1.0, 0 );

    {
        for( auto& func: viewerFuncitons )
        {
            std::lock_guard lGuard(mutex);
            glPushMatrix();
            func.function(func.worker);
            glPopMatrix();
        }

    }
    glFlush();
    glutSwapBuffers();

    glutPostRedisplay();
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

}

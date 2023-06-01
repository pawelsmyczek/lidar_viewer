#ifndef LIDAR_VIEWER_VIEWER_H
#define LIDAR_VIEWER_VIEWER_H

#include "CygLidarD1.h"
#include <vector>
#include <functional>

namespace lidar_viewer::ui
{

///@brief function to be viewed by viewer
struct ViewerFunction
{
    void(*function)(const void*);
    const void* worker;
};

/// @brief viewer, performs display actions using registered objects via registerViewerFunction
class Viewer
{
public:
    enum class ViewType
    {
        Flat, PointCloud
    };
    struct Config
    {
        unsigned int x;
        unsigned int y;
        unsigned int w;
        unsigned int h;
        ViewType viewType;
    };

    Viewer(Config configuration, int* argc, char** argv) noexcept;

    /// starts displaying using Config passed as a ctor, shall be used as a main thread loop
    void start() const;

    void display();

    void rotateUp() noexcept;
    void rotateDown() noexcept;
    void rotateLeft() noexcept;
    void rotateRight() noexcept;
    void scaleUp() noexcept;
    void scaleDown() noexcept;
    void zeroWindowTransforms() noexcept;

    /// @brief registers a function to draw a particular object
    /// @param func object drawing function
    /// @param obj object to work on
    template < class Object >
    void registerViewerFunction( void(*func)(const Object*), const Object* obj)
    {
        viewerFuncitons.push_back(
                ViewerFunction{ .function = reinterpret_cast<void (*)(const void*)>(func),
                                .worker = obj }
                 );
    }
private:
    std::vector<ViewerFunction> viewerFuncitons;
    Config configuration;
    std::atomic<int> rotx;
    std::atomic<int> roty;
    float windowScale;
    mutable std::mutex mutex;
};

}

#endif //LIDAR_VIEWER_VIEWER_H

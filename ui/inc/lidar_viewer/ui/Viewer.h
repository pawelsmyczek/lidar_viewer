#ifndef LIDAR_VIEWER_VIEWER_H
#define LIDAR_VIEWER_VIEWER_H

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <vector>
#include <thread>
#include <sstream>

namespace lidar_viewer::ui
{

/// @brief viewer, performs display actions using registered objects via registerViewerFunction
class Viewer
{
public:
    enum class ViewType: uint8_t
    {
        Flat,
        PointCloud,
        Octree,
        Statistics
    };
    struct Config
    {
        unsigned int x;
        unsigned int y;
        unsigned int w;
        unsigned int h;
    };

    Viewer(Config configuration) noexcept;
    ~Viewer() noexcept;

    /// starts displaying using Config passed as a ctor, shall be used as a main thread loop
    void start(int* argc, char** argv);

    /// closes the window, shall be used only in signal handlers etc
    void stop();

    void display();

    void rotateUp() noexcept;
    void rotateDown() noexcept;
    void rotateLeft() noexcept;
    void rotateRight() noexcept;
    void scaleUp() noexcept;
    void scaleDown() noexcept;
    void zeroWindowTransforms() noexcept;

    /// @brief registers a drawing function
    /// @param viewType type of viewer function
    /// @param func object drawing function
    /// @param args aruments to drawing function
    template < typename Func, typename ... Args  >
    void registerViewerFunction(ViewType viewType, Func&& func, Args && ... args )
    {
        if(auto iter = viewerFunctions.find(viewType); iter != viewerFunctions.end())
        {
            std::cout << "Function already registered\n";
            return ;
        }

        viewerFunctions.emplace(viewType, std::bind(std::forward<Func>(func)
                , std::forward<Args>(args)...));
    }

    void enableFunction(ViewType viewType);
    void disableFunction(ViewType viewType);
    void toggleFunction(ViewType viewType);

private:
    bool isStopped() const noexcept;

    std::unordered_map<ViewType, std::function<bool()>> viewerFunctions;
    std::vector<ViewType> enabledFunctions;
    Config configuration;
    std::atomic<int> rotx;
    std::atomic<int> roty;
    float windowScale;
    int windowId;
    std::atomic<bool> stopped;
    mutable std::mutex mutex;
    std::chrono::system_clock::time_point nextUpdate;
    std::chrono::milliseconds nextUpdateTime;
};

std::ostream& operator << (std::ostream& ostream, Viewer::ViewType viewType);

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_VIEWER_H

#ifndef LIDAR_VIEWER_DISPLAYMANAGER_H
#define LIDAR_VIEWER_DISPLAYMANAGER_H

#include "DisplayTransformations.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>

namespace lidar_viewer::ui
{

struct ViewManager;

struct DisplayFunctionGuard
{
    using DisplayGuardingFunction = std::function<void()>;
    explicit DisplayFunctionGuard(DisplayFunctionGuard::DisplayGuardingFunction &&f)
    : func{f}
    { }

    virtual void preDisplay() = 0;
    virtual void postDisplay() = 0;

    virtual ~DisplayFunctionGuard()
    {
        if(!func)
        {
            return ;
        }
        try
        {
            func();
        }
        catch (...)
        { /*NOOP*/ }
    }
private:
    DisplayGuardingFunction func;
};

struct DisplayManagerBase
{

    enum class ViewType: uint8_t
    {
        Flat,
        PointCloud,
        Octree,
        Statistics
    };

    virtual void start() = 0;
    virtual TransformationParameters& getDisplayTransforms() = 0;
    virtual const TransformationParameters& getDisplayTransforms() const = 0;

    virtual std::unique_ptr<DisplayFunctionGuard> makeDisplayGuard(ViewManager&) = 0;
    virtual void workOnRegisteredFunction(const std::pair<ViewType, std::function<bool()>> &, ViewManager&) = 0;
    virtual ~DisplayManagerBase() = default;
};

struct DisplayManager
        : public DisplayManagerBase
{
    DisplayManager(ViewManager& r, std::chrono::milliseconds updateTime)
            : viewerFunctions{}
            , enabledFunctions{}
            , viewManager{r}
            , nextUpdate{}
            , nextUpdateTime{updateTime}
    {
    }
    void display();

    template < typename Func, typename ... Args  >
    void registerDisplayFunction(ViewType viewType, Func&& func, Args && ... args )
    {
        if(auto iter = viewerFunctions.find(viewType); iter != viewerFunctions.end())
        {
            std::cout << "Function already registered\n";
            return ;
        }

        viewerFunctions.emplace(viewType, std::bind(std::forward<Func>(func)
                , std::forward<Args>(args)...));
    }

    void toggleFunction(ViewType viewType);

    void enableFunction(ViewType viewType);
    void disableFunction(ViewType viewType);

    ~DisplayManager() override = default;

protected:
    std::unordered_map<ViewType, std::function<bool()>> viewerFunctions;
    std::vector<ViewType> enabledFunctions;
private:
    ViewManager& viewManager;
    std::chrono::system_clock::time_point nextUpdate;
    std::chrono::milliseconds nextUpdateTime;
    mutable std::mutex mutex;

};

}
#endif //LIDAR_VIEWER_DISPLAYMANAGER_H

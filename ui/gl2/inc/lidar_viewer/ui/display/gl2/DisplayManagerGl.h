#ifndef LIDAR_VIEWER_DISPLAYMANAGERGL_H
#define LIDAR_VIEWER_DISPLAYMANAGERGL_H

#include "lidar_viewer/ui/display/DisplayManager.h"
#include "lidar_viewer/ui/display/DisplayTransformations.h"

#include <chrono>
#include <memory>

namespace lidar_viewer::ui
{

struct ViewManager;

struct DisplayFunctionGuardGl
    : public DisplayFunctionGuard
{
    explicit DisplayFunctionGuardGl(DisplayFunctionGuard::DisplayGuardingFunction &&f)
    : DisplayFunctionGuard{std::move(f)}
    { }
    ~DisplayFunctionGuardGl() override = default;

    void preDisplay() override;
    void postDisplay() override;

};

struct DisplayManagerGl
        : public DisplayManager
{
    DisplayManagerGl(ViewManager&, std::chrono::milliseconds);
    ~DisplayManagerGl() override = default;

    void start() override;

    TransformationParameters& getDisplayTransforms() override;
    const TransformationParameters& getDisplayTransforms() const override;

    std::unique_ptr<DisplayFunctionGuard> makeDisplayGuard(ViewManager&) override;
    void workOnRegisteredFunction(const std::pair<ViewType, std::function<bool()>> &, ViewManager&) override;
private:
    TransformationParameters transformationParameters;
};


} // namespace lidar_viewer::ui


#endif //LIDAR_VIEWER_DISPLAYMANAGERGL_H

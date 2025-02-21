#include "lidar_viewer/ui/display/DisplayManager.h"

namespace lidar_viewer
{
namespace ui
{

void DisplayManager::display()
{
    auto guard = makeDisplayGuard(viewManager);

    const auto now = std::chrono::system_clock::now();

    // try to keep constant FPS
    if(now <= nextUpdate)
    {
        return;
    }

    guard->preDisplay();
    std::for_each(viewerFunctions.begin(), viewerFunctions.end(),
                  [this](const auto& pair)
                  {
                      std::lock_guard lGuard{mutex};
                      auto iter = std::find(enabledFunctions.begin(), enabledFunctions.end(), pair.first);
                      if(iter == enabledFunctions.end())
                      {
                          return;
                      }
                      if(!pair.second)
                      {
                          return;
                      }
                      workOnRegisteredFunction(pair, viewManager);
                  }
    );
    guard->postDisplay();
    nextUpdate = now + nextUpdateTime;
}

void DisplayManager::enableFunction(DisplayManager::ViewType viewType)
{

    std::lock_guard lGuard{mutex};
    if(viewType <= ViewType::Octree)
    {
        // erase remove idiom - replace an existing display function with a new one
        enabledFunctions.erase(std::remove_if(enabledFunctions.begin(), enabledFunctions.end(),
                                              [](const auto& type)
                                              {return type <= ViewType::Octree;}), enabledFunctions.end());
    }
    enabledFunctions.emplace_back(viewType);
}

void DisplayManager::disableFunction(DisplayManager::ViewType viewType)
{

    std::lock_guard lGuard{mutex};
    // erase remove idiom - remove only one
    enabledFunctions.erase(std::remove_if(enabledFunctions.begin(), enabledFunctions.end(),
                                          [&](const auto& type) {return type == viewType;}));
}

void DisplayManager::toggleFunction(DisplayManager::ViewType viewType)
{
    if (auto iter = std::find_if(enabledFunctions.begin(), enabledFunctions.end(),
                                 [&viewType](const auto enabled)
                                 { return enabled == viewType;});
            iter == enabledFunctions.end())
    {
        enableFunction(viewType);
    }
    else
    {
        disableFunction(viewType);
    }
}

} // ui
} // lidar_viewer
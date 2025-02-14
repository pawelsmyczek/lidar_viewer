#include "lidar_viewer/ui/window/ViewManager.h"
#include "lidar_viewer/ui/display/DisplayManager.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>


namespace lidar_viewer::tests::units
{
// Mock ViewManager for testing
struct MockViewManager : public ui::ViewManager
{
    MOCK_METHOD(void, startImpl, (int*, char**), (override));
    MOCK_METHOD(void, stopImpl, (), (override));
};

// Mock DisplayFunctionGuard for testing
struct MockDisplayFunctionGuard : public ui::DisplayFunctionGuard
{
    explicit MockDisplayFunctionGuard(DisplayGuardingFunction&& f) : DisplayFunctionGuard(std::move(f)) {}

    MOCK_METHOD(void, preDisplay, (), (override));
    MOCK_METHOD(void, postDisplay, (), (override));
};

// Subclass DisplayManager to override makeDisplayGuard for testability
struct TestDisplayManager : public ui::DisplayManager
{
    TestDisplayManager(ui::ViewManager& r, std::chrono::milliseconds updateTime)
            : DisplayManager(r, updateTime)
            , params(0,0,0,0.f){}



    void start() override {};
    ui::TransformationParameters& getDisplayTransforms() override
    {
        return params;
    }
    const ui::TransformationParameters& getDisplayTransforms() const override
    {
        return params;
    }

    std::unique_ptr<ui::DisplayFunctionGuard> makeDisplayGuard(ui::ViewManager&) override {
        return std::make_unique<MockDisplayFunctionGuard>([]() {});
    }

    void workOnRegisteredFunction(const std::pair<ViewType, std::function<bool()>>& pair, ui::ViewManager&) override {
        if (pair.second) {
            pair.second(); // Call the function if it's valid
        }
    }

    std::unordered_map<ViewType, std::function<bool()>>& getDisplayFunctions()
    {
        return viewerFunctions;
    }

    std::vector<ViewType>& getEnabledFunctions()
    {
        return enabledFunctions;
    }

private:
    ui::TransformationParameters params;
};

class DisplayManagerTest : public ::testing::Test
{
protected:
    MockViewManager mockViewManager;
    TestDisplayManager displayManager;

    DisplayManagerTest()
            : displayManager(mockViewManager, std::chrono::milliseconds(16)) {}  // Simulate ~60 FPS (16ms/frame)
};

TEST_F(DisplayManagerTest, InitialState)
{
    EXPECT_TRUE(displayManager.getDisplayFunctions().empty());
    EXPECT_TRUE(displayManager.getEnabledFunctions().empty());
}

TEST_F(DisplayManagerTest, RegisterDisplayFunction)
{
    bool functionCalled = false;
    displayManager.registerDisplayFunction(ui::DisplayManager::ViewType::Flat, [&]() {
        functionCalled = true;
        return true;
    });

    ASSERT_EQ(displayManager.getDisplayFunctions().size(), 1);
    EXPECT_FALSE(functionCalled); // Function should not be called yet
}

TEST_F(DisplayManagerTest, EnableDisableFunction)
{
    displayManager.registerDisplayFunction(ui::DisplayManager::ViewType::Flat, []() { return true; });

    displayManager.enableFunction(ui::DisplayManager::ViewType::Flat);
    EXPECT_EQ(displayManager.getEnabledFunctions().size(), 1);

    displayManager.disableFunction(ui::DisplayManager::ViewType::Flat);
    EXPECT_EQ(displayManager.getEnabledFunctions().size(), 0);
}

TEST_F(DisplayManagerTest, ToggleFunction)
{
    displayManager.registerDisplayFunction(ui::DisplayManager::ViewType::Flat, []() { return true; });

    displayManager.toggleFunction(ui::DisplayManager::ViewType::Flat);
    EXPECT_EQ(displayManager.getEnabledFunctions().size(), 1);

    displayManager.toggleFunction(ui::DisplayManager::ViewType::Flat);
    EXPECT_EQ(displayManager.getEnabledFunctions().size(), 0);
}

TEST_F(DisplayManagerTest, DisplayFunctionExecution)
{
    bool functionCalled = false;

    displayManager.registerDisplayFunction(ui::DisplayManager::ViewType::Flat, [&]() {
        functionCalled = true;
        return true;
    });

    displayManager.enableFunction(ui::DisplayManager::ViewType::Flat);
    displayManager.display();

    EXPECT_TRUE(functionCalled);
}

} // namespace lidar_viewer::tests::units

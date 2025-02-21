#include "lidar_viewer/ui/window/ViewManager.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace lidar_viewer::tests::units
{

using namespace std::string_literals;

// Define a mock implementation of ViewManager for testing
struct MockViewManager : public ui::ViewManager
{
    MockViewManager() : startCalled(false), stopCalled(false) {}

    void startImpl(int *argc, char **argv) override
    {
        startCalled = true;
        stopped.store(false);
    }

    void stopImpl() override
    {
        stopCalled = true;
        stopped.store(true);
    }

    bool startCalled;
    bool stopCalled;
};

// Test case to check that ViewManager initializes correctly
TEST(ViewManagerTest, InitialState)
{
    MockViewManager viewManager;
    EXPECT_FALSE(viewManager.isStopped()) << "ViewManager should not be stopped on initialization.";
}

TEST(ViewManagerTest, StartTest)
{
    MockViewManager viewManager;
    int argc = 0;
    char** argv = nullptr;

    viewManager.start(&argc, argv);

    EXPECT_TRUE(viewManager.startCalled) << "startImpl should be called when start() is invoked.";
    EXPECT_FALSE(viewManager.isStopped()) << "ViewManager should not be stopped after start().";
}

TEST(ViewManagerTest, StopTest)
{
    MockViewManager viewManager;
    int argc = 0;
    char** argv = nullptr;

    viewManager.start(&argc, argv);
    viewManager.stop();

    EXPECT_TRUE(viewManager.stopCalled) << "stopImpl should be called when stop() is invoked.";
    EXPECT_TRUE(viewManager.isStopped()) << "ViewManager should be stopped after stop() is called.";
}

// Test case to ensure stop() does nothing if already stopped
TEST(ViewManagerTest, StopTwiceTest)
{
    MockViewManager viewManager;
    int argc = 0;
    char** argv = nullptr;

    viewManager.start(&argc, argv);
    viewManager.stop();
    viewManager.stop();  // Calling stop again

    EXPECT_TRUE(viewManager.stopCalled) << "stopImpl should be called at least once.";
    EXPECT_TRUE(viewManager.isStopped()) << "ViewManager should remain stopped after multiple stop() calls.";
}

} // namespace lidar_viewer::tests::units
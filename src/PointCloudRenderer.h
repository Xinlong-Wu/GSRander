#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#include "Vulkan/VulkanManager.h"

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

const uint32_t WIDTH = 800;
const uint32_t HEIGHT = 600;

class PointCloudRenderer {
public:
    PointCloudRenderer() {
        createWindow();
        manager = std::make_unique<VulkanManager>(window);
    };
    ~PointCloudRenderer();

    void render(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud);
private:
    GLFWwindow* window = nullptr;
    std::unique_ptr<VulkanManager> manager;

    void createWindow();
    void mainLoop();
};

#endif // POINT_CLOUD_RENDERER_H
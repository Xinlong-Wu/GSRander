#include "PointCloudRenderer.h"

GSRender::PointCloudRenderer::~PointCloudRenderer() {
    manager.reset();

    // Cleanup window
    glfwDestroyWindow(window);
    glfwTerminate();
}

void GSRender::PointCloudRenderer::render(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud) {
    mainLoop();
}

void GSRender::PointCloudRenderer::createWindow() {
    glfwInit();
    
    // Do not create OpenGL context
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    
    window = glfwCreateWindow(WIDTH, HEIGHT, "Gaussian Point Cloud Renderer", nullptr, nullptr);
    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, GSRender::framebufferResizeCallback);

    if (window == nullptr) {
        throw std::runtime_error("Failed to create window!");
    }
}

void GSRender::PointCloudRenderer::mainLoop() {
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        manager->drawFrame();
    }

    manager->waitForIdle();
}

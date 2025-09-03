#ifndef GSRENDER_VULKAN_MANAGER_H
#define GSRENDER_VULKAN_MANAGER_H

#include "Memory/Buffer.h"
#include "Memory/MemoryAllocator.h"

#include <memory>
#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vector>

namespace GSRender {

const int MAX_FRAMES_IN_FLIGHT = 2;

class VulkanManager {
private:
    bool initialized;
    // GLFW and Vulkan handles
    GLFWwindow* window = nullptr;
    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue graphicsQueue;
    VkSurfaceKHR surface;
    VkQueue presentQueue;
    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    VkRenderPass renderPass;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;
    std::vector<VkFramebuffer> swapChainFramebuffers;
    VkCommandPool commandPool;
    std::vector<VkCommandBuffer> commandBuffers;

    uint32_t currentFrame = 0;
    bool framebufferResized = false;
    bool useFixedViewport = false;
    VkExtent2D fixedViewportExtent;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;

    std::unique_ptr<GSRender::Buffer> vertexBuffer;
    std::unique_ptr<GSRender::Buffer> indexBuffer;
    
    // 内存分配器
    std::unique_ptr<GSRender::MemoryAllocator> memoryAllocator;
    
    // Private methods
    void createInstance();
    void setupDebugMessenger();
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSurface();
    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
    void createSwapChain();
    void createImageViews();
    void createGraphicsPipeline();
    void createRenderPass();

    void recreateSwapChain();
    void cleanupSwapChain();

    void createFramebuffers();
    void createCommandPool();
    void createCommandBuffers();
    void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
    
    void createSyncObjects();

    void createVertexBuffer();
    void createIndexBuffer();

    // helper methods
    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
    std::vector<const char*> getRequiredExtensions();
    bool checkValidationLayerSupport();
public:
    VulkanManager(GLFWwindow* window);
    ~VulkanManager();

    void drawFrame();
    void waitForIdle() {
        vkDeviceWaitIdle(device);
    }
    void setFramebufferResized(bool resized) {
        framebufferResized = resized;
    }
};

} // namespace GSRender

#endif // VULKAN_MANAGER_H
#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vector>
#include <glm/glm.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const uint32_t WIDTH = 800;
const uint32_t HEIGHT = 600;

struct UniformBufferObject {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;
};

class PointCloudRenderer {
public:
    PointCloudRenderer();
    ~PointCloudRenderer();

    bool initialize();
    void render(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud);
    void cleanup();

private:
    // Vulkan specific members
    bool initialized;
    
    // GLFW and Vulkan handles
    GLFWwindow* window;
    VkInstance instance;
    VkPhysicalDevice physicalDevice;
    VkDevice device;
    VkQueue graphicsQueue;
    VkSurfaceKHR surface;
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
    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
    std::vector<VkFence> imagesInFlight;
    size_t currentFrame = 0;
    
    // Point cloud rendering resources
    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;
    VkBuffer uniformBuffer;
    VkDeviceMemory uniformBufferMemory;
    VkDescriptorSetLayout descriptorSetLayout;
    VkDescriptorPool descriptorPool;
    VkDescriptorSet descriptorSet;
    
    // Private methods
    bool createWindow();
    bool createInstance();
    bool setupDebugMessenger();
    bool createSurface();
    bool pickPhysicalDevice();
    bool createLogicalDevice();
    bool createSwapChain();
    bool createImageViews();
    bool createRenderPass();
    bool createDescriptorSetLayout();
    bool createGraphicsPipeline();
    bool createFramebuffers();
    bool createCommandPool();
    bool createVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud);
    bool createIndexBuffer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud);
    bool createUniformBuffers();
    bool createDescriptorPool();
    bool createDescriptorSets();
    bool createCommandBuffers();
    bool createSyncObjects();
    
    void drawFrame();
    void updateUniformBuffer();
    
    // Helper methods
    bool checkValidationLayerSupport();
    std::vector<const char*> getRequiredExtensions();
    bool isDeviceSuitable(VkPhysicalDevice device);
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    VkShaderModule createShaderModule(const std::vector<char>& code);
    
    // Static callback methods
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
};

#endif // POINT_CLOUD_RENDERER_H
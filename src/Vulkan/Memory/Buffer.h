#ifndef GSRENDER_BUFFER_H
#define GSRENDER_BUFFER_H

#include "MemoryBlock.h"

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

namespace GSRender {

struct UniformBufferObject {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;
};

class Buffer {
public:
    ~Buffer();

    VkBuffer getVkBuffer() const { return buffer; }
    const GSRender::Memory* getBindMemory() const { return memory; }
private:
    Buffer(VkDevice device, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties);
    VkDevice device;
    VkBuffer buffer;
    // VkDeviceSize size;
    // VkBufferUsageFlags usage;
    // VkMemoryPropertyFlags properties;

    const GSRender::Memory* memory;

    void destroyBuffer();
    void bindMemory(const GSRender::Memory* memory);

    friend class MemoryAllocator;
};
}

#endif // GSRENDER_BUFFER_H
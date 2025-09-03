#ifndef GSRENDER_BUFFER_H
#define GSRENDER_BUFFER_H

#include "MemoryBlock.h"

#include <vulkan/vulkan.h>

namespace GSRender {

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
#ifndef GSRENDER_MEMORY_ALLOCATOR_H
#define GSRENDER_MEMORY_ALLOCATOR_H

#include "Buffer.h"
#include "MemoryBlock.h"

#include <vulkan/vulkan.h>
#include <vector>
#include <unordered_map>

namespace GSRender {

class MemoryAllocator {
private:
    VkPhysicalDevice physicalDevice;
    VkDevice device;
    VkDeviceSize minBlockSize; // 每个内存块的大小
    
    // 存储内存块的向量，每个块包含多个缓冲区分配
    std::unordered_map<uint32_t, std::vector<std::unique_ptr<GSRender::MemoryBlock>>> memoryPools;
    
    // GSMemory 到 Buffer 的映射
    std::unordered_map<const GSRender::Memory*, GSRender::Buffer*> memoryToBufferMap;
    
    void free(VkDeviceMemory memory, VkDeviceSize offset, uint32_t memoryTypeIndex);

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    
public:
    MemoryAllocator(VkPhysicalDevice physicalDevice, VkDevice device, VkDeviceSize minBlockSize = 64 * 1024 * 1024); // 默认块大小为64MB
    ~MemoryAllocator() { clear(); };
    
    const GSRender::Memory* allocate(VkDeviceSize size, VkDeviceSize alignment, VkMemoryPropertyFlags properties, uint32_t memoryTypeIndex);
    void free(const GSRender::Memory* memory);
    void clear();

    std::unique_ptr<GSRender::Buffer> createBufferWithMemory(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, const GSRender::Memory* memory);
    std::unique_ptr<GSRender::Buffer> createBufferWithMemory(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties);
    std::unique_ptr<GSRender::Buffer> createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties);
    void destroyBuffer(std::unique_ptr<GSRender::Buffer> buffer);
    void freeBuffer(std::unique_ptr<GSRender::Buffer> buffer);
    void bindMemory(GSRender::Buffer* buffer, const GSRender::Memory* memory);
};

}

#endif // MEMORY_ALLOCATOR_H
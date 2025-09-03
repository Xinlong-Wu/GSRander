#include "MemoryAllocator.h"
#include "Buffer.h"
#include "MemoryBlock.h"
#include <memory>
#include <stdexcept>

using namespace GSRender;

MemoryAllocator::MemoryAllocator(VkPhysicalDevice physicalDevice, VkDevice device, VkDeviceSize minBlockSize)
    : physicalDevice(physicalDevice), device(device), minBlockSize(minBlockSize) {}

MemoryAllocator::~MemoryAllocator() {
    clear();
}

uint32_t MemoryAllocator::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }

    throw std::runtime_error("failed to find suitable memory type!");
}

const GSRender::Memory* MemoryAllocator::allocate(VkDeviceSize size, VkDeviceSize alignment, VkMemoryPropertyFlags properties, uint32_t memoryTypeIndex) {
    auto [memoryBlocksIt, inserted] = memoryPools.try_emplace(
        memoryTypeIndex,
        std::vector<std::unique_ptr<GSRender::MemoryBlock>>()
    );

    std::vector<std::unique_ptr<GSRender::MemoryBlock>>& memoryBlocks = memoryBlocksIt->second;

    // 遍历现有的内存块，寻找合适的空闲空间
    for (size_t i = 0; i < memoryBlocks.size(); i++) {
        const GSRender::Memory* gsMemory = memoryBlocks[i]->allocate(size, alignment);
        if (gsMemory != nullptr) {
            return gsMemory;
        }
    }
    
    // 如果没有找到合适的空闲空间，创建新的内存块
    // 使用较大的块大小以减少碎片
    VkDeviceSize blockSize = std::max(size, minBlockSize);
    std::unique_ptr<GSRender::MemoryBlock> newMemoryBlock = GSRender::MemoryBlock::create(device, blockSize, memoryTypeIndex);
    if (!newMemoryBlock) {
        throw std::runtime_error("failed to create memory block!");
        // return false;
    }

    // 创建新的分配记录
    const GSRender::Memory* gsMemory = newMemoryBlock->allocate(size, alignment);
    
    // 添加新的内存块到列表
    memoryBlocks.push_back(std::move(newMemoryBlock));
    
    return gsMemory;
}

void MemoryAllocator::free(const GSRender::Memory* memory) {
    free(memory->getDeviceMemory(), memory->getOffset(), memory->getMemoryTypeIndex());
    return;
}

void MemoryAllocator::free(VkDeviceMemory memory, VkDeviceSize offset, uint32_t memoryTypeIndex) {
    auto memoryBlocksIt = memoryPools.find(memoryTypeIndex);
    if(memoryBlocksIt == memoryPools.end()) {
        throw std::runtime_error("failed to find memory blocks with specific MemoryType!");
    }

    std::vector<std::unique_ptr<GSRender::MemoryBlock>>& memoryBlocks = memoryBlocksIt->second;

    // 查找对应的内存块
    for (auto it = memoryBlocks.begin(); it != memoryBlocks.end(); it++) {
        if (it->get()->getDeviceMemory() == memory) {
            it->get()->free(offset);
            if (it->get()->isEmpty()) {
                memoryBlocks.erase(it);
            }
            return;
        }
    }
}

void MemoryAllocator::clear() {
    // 释放所有内存块
    memoryPools.clear();
    memoryToBufferMap.clear();
}

std::unique_ptr<GSRender::Buffer> MemoryAllocator::createBufferWithMemory(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, const GSRender::Memory* memory) {
    GSRender::Buffer* gsBuffer = new GSRender::Buffer(device, size, usage, properties);
    try {
        bindMemory(gsBuffer, memory);
        return std::unique_ptr<GSRender::Buffer>(gsBuffer);
    } catch (...) {
        delete gsBuffer;
        throw std::runtime_error("failed to create buffer with memory!");
    }
}

std::unique_ptr<GSRender::Buffer> MemoryAllocator::createBufferWithMemory(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties) {
    GSRender::Buffer* gsBuffer = new GSRender::Buffer(device, size, usage, properties);
    const GSRender::Memory* gsMemory = nullptr;

    try {
        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device, gsBuffer->getVkBuffer(), &memRequirements);
        // 使用内存分配器来分配内存
        uint32_t memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);
        gsMemory = this->allocate(memRequirements.size, memRequirements.alignment, properties, memoryTypeIndex);
        if (!gsMemory) {
            throw ;
        }
        bindMemory(gsBuffer, gsMemory);
        return std::unique_ptr<GSRender::Buffer>(gsBuffer);
    } catch (...) {
        delete gsBuffer;
        if (gsMemory) {
            free(gsMemory);
        }
        throw std::runtime_error("failed to create buffer with memory!");
    }
}

std::unique_ptr<GSRender::Buffer> MemoryAllocator::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties) {
    GSRender::Buffer* gsBuffer = new GSRender::Buffer(device, size, usage, properties);
    return std::unique_ptr<GSRender::Buffer>(gsBuffer);
}

void MemoryAllocator::destroyBuffer(std::unique_ptr<GSRender::Buffer> buffer) {
    if (!buffer) {
        return;
    }
    
    const GSRender::Memory* memory = buffer->getBindMemory();
    buffer->destroyBuffer();
    if (memory) {
        memoryToBufferMap.erase(memory);
    }
}

void MemoryAllocator::freeBuffer(std::unique_ptr<GSRender::Buffer> buffer) {
    if (!buffer) {
        return;
    }
    
    const GSRender::Memory* memory = buffer->getBindMemory();
    destroyBuffer(std::move(buffer));
    if (memory) {
        free(memory);
    }
}

void MemoryAllocator::bindMemory(GSRender::Buffer* buffer, const GSRender::Memory* memory) {
    if (!buffer || !memory) {
        throw std::runtime_error("Invalid buffer or memory pointer!");
    }
    
    memoryToBufferMap[memory] = buffer;
    buffer->bindMemory(memory);
}

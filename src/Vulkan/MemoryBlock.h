#ifndef MEMORY_BLOCK_H
#define MEMORY_BLOCK_H

#include <vulkan/vulkan.h>
#include <list>
#include <memory>

namespace GSRender {

class Buffer;
class Memory {
private:
    Memory(uint32_t memoryTypeIndex, VkDeviceMemory memory, VkDeviceSize offset, VkDeviceSize size, VkDeviceSize bias, bool inUse) : memory(memory), offset(offset), size(size), bias(bias), memoryTypeIndex(memoryTypeIndex), inUse(inUse) {};
    VkDeviceMemory memory;
    VkDeviceSize offset;
    VkDeviceSize size;
    VkDeviceSize bias;
    uint32_t memoryTypeIndex;
    bool inUse;

    // copy constructor
    Memory(const Memory&) = delete;
    // copy assignment operator
    Memory& operator=(const Memory&) = delete;
    // move constructor
    Memory(Memory&&) = delete;
    // move assignment operator
    Memory& operator=(Memory&&) = delete;
public:
    ~Memory() = default;
    VkDeviceMemory getDeviceMemory() const { return memory; }
    VkDeviceSize getOffset() const { return offset; }
    VkDeviceSize getSize() const { return size; }
    uint32_t getMemoryTypeIndex() const { return memoryTypeIndex; }

    friend class MemoryBlock;
    friend class Buffer;
};

class MemoryBlock {
private:
    MemoryBlock(VkDevice device, VkDeviceMemory memory, VkDeviceSize size, uint32_t memoryTypeIndex);
    VkDevice device;
    VkDeviceMemory memory;
    VkDeviceSize size;
    uint32_t memoryTypeIndex;
    std::list<std::unique_ptr<GSRender::Memory>> allocations;

    // copy constructor
    MemoryBlock(const MemoryBlock&) = delete;
    // copy assignment operator
    MemoryBlock& operator=(const MemoryBlock&) = delete;
    // move constructor
    MemoryBlock(MemoryBlock&&) = delete;
    // move assignment operator
    MemoryBlock& operator=(MemoryBlock&&) = delete;
public:
    ~MemoryBlock();

    VkDeviceMemory getDeviceMemory() const { return memory; }
    VkDeviceSize getBlockSize() const { return size; }
    bool isEmpty() const { return allocations.size() == 1; }
    // const std::list<std::unique_ptr<GSRender::Memory>>& getAllocations() const { return allocations; }

    const GSRender::Memory* allocate(VkDeviceSize size, VkDeviceSize alignment);
    bool free(VkDeviceSize offset);

    static std::unique_ptr<MemoryBlock> create(VkDevice device, VkDeviceSize size, uint32_t memoryTypeIndex);
};
}

#endif // MEMORY_BLOCK_H

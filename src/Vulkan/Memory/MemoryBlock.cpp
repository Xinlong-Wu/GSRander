#include "MemoryBlock.h"
#include <cassert>
#include <cstdint>
#include <memory>
#include <stdexcept>

GSRender::MemoryBlock::MemoryBlock(VkDevice device, VkDeviceMemory memory, VkDeviceSize size, uint32_t memoryTypeIndex) : device(device), memory(memory), size(size), memoryTypeIndex(memoryTypeIndex) {
    allocations.push_back(std::unique_ptr<Memory>(new Memory(memoryTypeIndex,memory, 0, size, 0, false)));
}

GSRender::MemoryBlock::~MemoryBlock() {
    allocations.clear();
    vkFreeMemory(device, memory, nullptr);
}

const GSRender::Memory* GSRender::MemoryBlock::allocate(VkDeviceSize size, VkDeviceSize alignment) {
    for (auto it = allocations.begin(); it != allocations.end(); ++it) {
        Memory* block = it->get();
        if (block->inUse) {
            continue;
        }

        // 计算对齐后的偏移量
        VkDeviceSize alignedOffset = block->offset;
        if (alignedOffset % alignment != 0) {
            alignedOffset += alignment - (alignedOffset % alignment);
        }

        // 检查是否有足够的空间
        if (alignedOffset + size > block->offset + block->size) {
            continue;
        }

        GSRender::Memory* newAllocation = new Memory(memoryTypeIndex, this->memory, alignedOffset, size, alignedOffset - block->offset, true);
        VkDeviceSize oldEnd = block->offset + block->size;
        assert(block->bias == 0 && "Bias of unused block must be 0");
        block->offset = alignedOffset + size;
        block->size = oldEnd - block->offset;

        allocations.insert(it, std::unique_ptr<Memory>(newAllocation));
        return newAllocation;
    }

    return nullptr;
}

bool GSRender::MemoryBlock::free(VkDeviceSize offset) {
    auto it = std::find_if(allocations.begin(), allocations.end(),
                           [offset](const std::unique_ptr<Memory>& m) { return m->offset == offset; });
    if (it == allocations.end()) return false;

    GSRender::Memory* block = it->get();
    if (!block->inUse) throw std::runtime_error("memory is not in use");

    block->inUse = false;
    block->size += block->bias;
    block->offset -= block->bias;
    block->bias = 0;

    if (it != allocations.begin()) {
        auto prevIt = std::prev(it);
        GSRender::Memory* prevBlock = prevIt->get();

        if (prevBlock->inUse == false) {
            block->size += prevBlock->size;
            block->offset = prevBlock->offset;
            allocations.erase(prevIt);
        }
    }

    auto nextIt = std::next(it);
    if (nextIt != allocations.end()) {
        GSRender::Memory* nextBlock = nextIt->get();

        if (nextBlock->inUse == false) {
            block->size += nextBlock->size;
            allocations.erase(nextIt);
        }
    }
    return true;
}

std::unique_ptr<GSRender::MemoryBlock> GSRender::MemoryBlock::create(VkDevice device, VkDeviceSize size, uint32_t memoryTypeIndex) {
    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = size;
    allocInfo.memoryTypeIndex = memoryTypeIndex;

    VkDeviceMemory newBlock;
    if (vkAllocateMemory(device, &allocInfo, nullptr, &newBlock) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate memory!");
        // return nullptr; // 分配失败
    }

    return std::unique_ptr<GSRender::MemoryBlock>(new GSRender::MemoryBlock(device, newBlock, size, memoryTypeIndex));
}

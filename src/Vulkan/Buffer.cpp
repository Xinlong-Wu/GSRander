#include "Buffer.h"
#include <stdexcept>

GSRender::Buffer::Buffer(VkDevice device, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties) : device(device), memory(nullptr) {
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to create buffer!");
    }
}

GSRender::Buffer::~Buffer() {
    destroyBuffer();
}

void GSRender::Buffer::destroyBuffer() {
    if (buffer) {
        vkDestroyBuffer(device, buffer, nullptr);
        buffer = VK_NULL_HANDLE;
    }
    // 不返回内存指针，避免所有权混乱
    // 内存的释放由MemoryAllocator负责管理
}

void GSRender::Buffer::bindMemory(const GSRender::Memory* memory) {
    this->memory = memory;
    VkResult result = vkBindBufferMemory(device, buffer, memory->getDeviceMemory(), memory->getOffset());
    if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to bind buffer memory!");
    }
}

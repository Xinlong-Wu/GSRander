
# Find Vulkan
find_package(Vulkan REQUIRED)

# Find glfw3
find_library_with_env_fallback(glfw3 FALSE)

add_subdirectory(Memory)
set(VULKAN_DRIVER_SRC_LIST
    VulkanManager.cpp
    utils/VkUtils.cpp
    ${MEMORY_SRC_LIST}
)

add_library(render STATIC ${VULKAN_DRIVER_SRC_LIST})

# Link glfw3 library (either from find_package or from environment variable)
if(glfw3_FOUND)
    # If found through find_package, use the imported target
    target_link_libraries(render PUBLIC GLFW::GLFW)
else()
    # If found through environment variable, use the library directly
    target_link_libraries(render PUBLIC ${glfw3_LIBRARY})
    target_include_directories(render PUBLIC ${glfw3_INCLUDE_DIRS})
endif()

target_link_libraries(render PUBLIC
    Vulkan::Vulkan
)

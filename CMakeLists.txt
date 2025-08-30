cmake_minimum_required(VERSION 3.10)
project(GSRander)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

# Find Vulkan
find_package(Vulkan REQUIRED)

# Add executable
add_executable(gsrander
    src/main.cpp
    src/PointCloudRenderer.cpp
    src/GaussianPointCloudLoader.cpp
)

# Compile shaders
find_program(GLSL_COMPILER glslangValidator)

if(GLSL_COMPILER)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.vert.spv
        COMMAND ${GLSL_COMPILER} --target-env vulkan1.0 -o ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.vert.spv ${CMAKE_CURRENT_SOURCE_DIR}/shaders/point_cloud.vert
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/point_cloud.vert
        COMMENT "Compiling vertex shader"
    )
    
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.frag.spv
        COMMAND ${GLSL_COMPILER} --target-env vulkan1.0 -o ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.frag.spv ${CMAKE_CURRENT_SOURCE_DIR}/shaders/point_cloud.frag
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/point_cloud.frag
        COMMENT "Compiling fragment shader"
    )
    
    add_custom_target(shaders ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.vert.spv ${CMAKE_CURRENT_BINARY_DIR}/point_cloud.frag.spv
    )
    
    add_dependencies(gsrander shaders)
endif()

# Link libraries
target_link_libraries(gsrander
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    Vulkan::Vulkan
    glfw
)

target_include_directories(gsrander PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${VULKAN_INCLUDE_DIRS}
)

target_compile_definitions(gsrander PRIVATE
    ${PCL_DEFINITIONS}
)
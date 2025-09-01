cmake_minimum_required(VERSION 3.10)
project(GSRander)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Try to find glfw3 with find_package first
find_package(glfw3 QUIET)

# If find_package failed, try to use glfw3_DIR environment variable
if(NOT glfw3_FOUND)
    if(DEFINED ENV{glfw3_DIR})
        message(STATUS "Using glfw3 from glfw3_DIR environment variable: $ENV{glfw3_DIR}")
        # Add include directories
        include_directories($ENV{glfw3_DIR}/include)
        
        # Determine library path based on platform
        if(WIN32)
            # For Windows, check for both Release and Debug libraries
            if(EXISTS $ENV{glfw3_DIR}/lib/glfw3.lib)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/glfw3.lib)
            elseif(EXISTS $ENV{glfw3_DIR}/lib/glfw3dll.lib)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/glfw3dll.lib)
            endif()
        elseif(APPLE)
            # For macOS, look for .dylib or .a files
            if(EXISTS $ENV{glfw3_DIR}/lib/libglfw3.dylib)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/libglfw3.dylib)
            elseif(EXISTS $ENV{glfw3_DIR}/lib/libglfw3.a)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/libglfw3.a)
            endif()
        else()
            # For Linux, look for .so or .a files
            if(EXISTS $ENV{glfw3_DIR}/lib/libglfw3.so)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/libglfw3.so)
            elseif(EXISTS $ENV{glfw3_DIR}/lib/libglfw3.a)
                set(GLFW3_LIBRARY $ENV{glfw3_DIR}/lib/libglfw3.a)
            endif()
        endif()
        
        # Check if we found the library
        if(GLFW3_LIBRARY)
            message(STATUS "Found glfw3 library: ${GLFW3_LIBRARY}")
        else()
            message(FATAL_ERROR "Could not find glfw3 library in $ENV{glfw3_DIR}/lib/")
        endif()
    else()
        # If neither find_package nor glfw3_DIR environment variable worked, fatal error
        message(FATAL_ERROR "Could not find glfw3. Please either install it system-wide or set the glfw3_DIR environment variable to point to its location.")
    endif()
endif()

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
)

# Link glfw3 library (either from find_package or from environment variable)
if(glfw3_FOUND)
    # If found through find_package, use the imported target
    target_link_libraries(gsrander glfw)
else()
    # If found through environment variable, use the library directly
    target_link_libraries(gsrander ${GLFW3_LIBRARY})
endif()
target_include_directories(gsrander PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${VULKAN_INCLUDE_DIRS}
)

target_compile_definitions(gsrander PRIVATE
    ${PCL_DEFINITIONS}
)

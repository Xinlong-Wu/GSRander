cmake_minimum_required(VERSION 3.10)
project(GSRander)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Function to find library with environment variable fallback
function(find_library_with_env_fallback PACKAGE_NAME HEADER_ONLY)
    # Try to find package with find_package first
    find_package(${PACKAGE_NAME} QUIET)

    # If find_package failed, try to use ${PACKAGE_NAME}_DIR environment variable
    if(NOT ${PACKAGE_NAME}_FOUND)
        if(DEFINED ENV{${PACKAGE_NAME}_DIR})
            message(STATUS "Using ${PACKAGE_NAME} from ${PACKAGE_NAME}_DIR environment variable: $ENV{${PACKAGE_NAME}_DIR}")
            set(${PACKAGE_NAME}_INCLUDE_DIRS $ENV{${PACKAGE_NAME}_DIR}/include PARENT_SCOPE)

            # If it's a header-only library, we're done
            if(HEADER_ONLY)
                return()
            endif()
            
            # Determine library path based on platform
            set(_lib_path "")
            if(WIN32)
                if(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/${PACKAGE_NAME}.lib)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/${PACKAGE_NAME}.lib)
                elseif(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/${PACKAGE_NAME}dll.lib)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/${PACKAGE_NAME}dll.lib)
                endif()
            elseif(APPLE)
                if(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.dylib)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.dylib)
                elseif(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.a)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.a)
                endif()
            else() # Linux
                if(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.so)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.so)
                elseif(EXISTS $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.a)
                    set(_lib_path $ENV{${PACKAGE_NAME}_DIR}/lib/lib${PACKAGE_NAME}.a)
                endif()
            endif()
            
            # Set library variable both in current and parent scope
            if(_lib_path)
                set(${PACKAGE_NAME}_LIBRARY "${_lib_path}" PARENT_SCOPE)
                message(STATUS "Found ${PACKAGE_NAME} library: ${_lib_path}")
            else()
                message(FATAL_ERROR "Could not find ${PACKAGE_NAME} library in $ENV{${PACKAGE_NAME}_DIR}/lib/")
            endif()
        else()
            message(FATAL_ERROR "Could not find ${PACKAGE_NAME}. Please either install it system-wide or set the ${PACKAGE_NAME}_DIR environment variable to point to its location.")
        endif()
    endif()
endfunction()

# Find required packages
find_package(PCL REQUIRED)

# Find glm (header-only library)
find_library_with_env_fallback(glm TRUE)

# Find glfw3
find_library_with_env_fallback(glfw3 FALSE)

# Find Eigen3
# find_package(Eigen3 3.3 REQUIRED)

add_subdirectory(src/Vulkan)

# Add executable
add_executable(gsrander
    src/main.cpp
    src/PointCloudRenderer.cpp
    src/GaussianPointCloudLoader.cpp
)

# Compile shaders
find_program(GLSL_COMPILER glslangValidator)

if(GLSL_COMPILER)
    function(compile_shader SHADER_FILE)
        get_filename_component(SHADER_NAME ${SHADER_FILE} NAME_WE)
        get_filename_component(SHADER_EXT ${SHADER_FILE} LAST_EXT)
        set(SHADER_TYPE "")
        
        if(SHADER_EXT STREQUAL ".vert")
            set(SHADER_TYPE "vertex")
        elseif(SHADER_EXT STREQUAL ".frag")
            set(SHADER_TYPE "fragment")
        else()
            message(FATAL_ERROR "Unknown shader type for ${SHADER_FILE}")
        endif()
        
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME}_${SHADER_EXT}.spv
            COMMAND ${GLSL_COMPILER} --target-env vulkan1.0 -o ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME}${SHADER_EXT}.spv ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SHADER_FILE}
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SHADER_FILE}
            COMMENT "Compiling ${SHADER_TYPE} shader ${SHADER_FILE}"
        )
        
        # Add to the list of shader outputs
        list(APPEND SHADER_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME}_${SHADER_EXT}.spv)
        set(SHADER_OUTPUTS ${SHADER_OUTPUTS} PARENT_SCOPE)
    endfunction()

    compile_shader(point_cloud.vert)
    compile_shader(point_cloud.frag)
    compile_shader(shader.vert)
    compile_shader(shader.frag)
    
    add_custom_target(shaders ALL
        DEPENDS ${SHADER_OUTPUTS}
    )
    
    add_dependencies(gsrander shaders)
endif()

# Link libraries
target_link_libraries(gsrander PRIVATE
    render
    ${PCL_LIBRARIES}
    # Eigen3::Eigen
)

# Link glfw3 library (either from find_package or from environment variable)
if(glfw3_FOUND)
    # If found through find_package, use the imported target
    target_link_libraries(gsrander PRIVATE GLFW::GLFW)
else()
    # If found through environment variable, use the library directly
    target_link_libraries(gsrander PRIVATE ${glfw3_LIBRARY})
    target_include_directories(gsrander PRIVATE ${glfw3_INCLUDE_DIRS})
endif()

target_include_directories(gsrander PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(gsrander PRIVATE
    ${PCL_DEFINITIONS}
)

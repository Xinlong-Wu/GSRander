#include <iostream>
#include "PointCloudRenderer.h"
#include "GaussianPointCloudLoader.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
        return -1;
    }

    // Load Gaussian point cloud
    GaussianPointCloudLoader loader;
    auto pointCloud = loader.loadFromFile(argv[1]);
    
    if (!pointCloud || pointCloud->empty()) {
        std::cerr << "Failed to load point cloud from file: " << argv[1] << std::endl;
        return -1;
    }

    // Initialize renderer
    PointCloudRenderer renderer;
    if (!renderer.initialize()) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return -1;
    }

    // Render point cloud
    renderer.render(pointCloud);

    // Cleanup
    renderer.cleanup();

    return 0;
}
#include <iostream>
#include "PointCloudRenderer.h"
#include "GaussianPointCloudLoader.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
        return -1;
    }

    // Load Gaussian point cloud
    GSRender::GaussianPointCloudLoader loader;
    auto pointCloud = loader.loadFromFile(argv[1]);
    
    if (!pointCloud || pointCloud->empty()) {
        std::cerr << "Failed to load point cloud from file: " << argv[1] << std::endl;
        return -1;
    }

    // Initialize renderer
    GSRender::PointCloudRenderer renderer;

    // Render point cloud
    renderer.render(pointCloud);

    return 0;
}
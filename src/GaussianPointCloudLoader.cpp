#include "GaussianPointCloudLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

GaussianPointCloudLoader::GaussianPointCloudLoader() {}

GaussianPointCloudLoader::~GaussianPointCloudLoader() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GaussianPointCloudLoader::loadFromFile(const std::string& filename) {
    auto pointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return pointCloud;
    }
    
    std::string line;
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> colors;
    
    // Parse the file
    // This is a simple parser that assumes a specific format
    // You may need to adjust this based on your actual file format
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z, r, g, b;
        
        if (iss >> x >> y >> z >> r >> g >> b) {
            // Assuming the file contains position and color data
            positions.emplace_back(x, y, z);
            colors.emplace_back(r, g, b);
        }
    }
    
    file.close();
    
    // Convert to PCL format
    pointCloud->resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = positions[i].x();
        point.y = positions[i].y();
        point.z = positions[i].z();
        
        // Assuming color values are in range [0, 1], convert to [0, 255]
        point.r = static_cast<uint8_t>(colors[i].x() * 255);
        point.g = static_cast<uint8_t>(colors[i].y() * 255);
        point.b = static_cast<uint8_t>(colors[i].z() * 255);
        
        pointCloud->at(i) = point;
    }
    
    std::cout << "Loaded " << pointCloud->size() << " points from " << filename << std::endl;
    
    return pointCloud;
}
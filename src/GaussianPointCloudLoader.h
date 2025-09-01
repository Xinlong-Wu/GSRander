#ifndef GAUSSIAN_POINT_CLOUD_LOADER_H
#define GAUSSIAN_POINT_CLOUD_LOADER_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GaussianPointCloudLoader {
public:
    GaussianPointCloudLoader() = default;
    ~GaussianPointCloudLoader() = default;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadFromFile(const std::string& filename);

private:
    // Private members for file parsing would go here
};

#endif // GAUSSIAN_POINT_CLOUD_LOADER_H
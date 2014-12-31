#ifndef FILE_IO_H
#define FILE_IO_H

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "common_type.h"

bool loadPointCloud_pcd(char* fileName, PointCloudPtr_RGB_NORMAL cloud);
bool loadPointCloud_ply(char* fileName, PointCloudPtr_RGB cloud);
bool loadPointCloud_normal_ply(char* fileName, PointCloudPtr_RGB_NORMAL cloud);

#endif // FILE_IO_H
#ifndef New_ICP_H
#define New_ICP_H

#include "common_type.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>

void computerICPWithNormal(PointCloudPtr_RGB_NORMAL cloud_in0, PointCloudPtr_RGB_NORMAL cloud_in1, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations);
bool loadPointCloud_ply(char* fileName, PointCloudPtr_RGB cloud);
bool loadPointCloud_normal_ply(char* fileName, PointCloudPtr_RGB_NORMAL cloud);
//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name);
void showPointCloud_RGB_NORMAL (PointCloudPtr_RGB_NORMAL cloud,std::string name);
//append a cloud to another cloud
void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud);
void appendCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL sourceCloud,PointCloudPtr_RGB_NORMAL targetCloud);
void computeIcpPCL(PointCloudPtr_RGB cloud_in0, PointCloudPtr_RGB cloud_in1, PointCloudPtr_RGB cloud_out, int iterations, int max_iterations);
void meargePointCloudsNoNormal(vector<MyPointCloud_RGB_NORMAL> clouds, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations);
void meargePointCloudsWithNormal(vector<MyPointCloud_RGB_NORMAL> clouds, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations);

#endif 
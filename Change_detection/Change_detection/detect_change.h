#ifndef DETECT_CHANGE_H
#define DETECT_CHANGE_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>

#include "common_type.h"
#include "common_func.h"
#include "visualizer.h"

#include <iostream>
#include <vector>
#include <ctime>

#include <Wm5IntrTriangle3Triangle3.h>
#include <Wm5IntrTriangle3Sphere3.h>
#include <Wm5IntrTriangle3Cylinder3.h>
#include <Wm5IntrSegment2Segment2.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name);
//Euclidean Cluster Extraction
void object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points);
//detect change of two point cloud
void detect_change(PointCloudPtr_RGB cloud0, PointCloudPtr_RGB cloud1, PointCloudPtr_RGB result0, PointCloudPtr_RGB result1);
//detect table
void detect_table(PointCloudPtr_RGB sourceCloud, pcl::ModelCoefficients& plane_coefficients, PointCloudPtr_RGB planeCloud, PointCloudPtr rect_cloud, PointCloudPtr_RGB remainingCloud);
//get transform matrix between plane and x_y plane 
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r);
//get points cloud on the table rect
void getCloudOnTable(PointCloudPtr_RGB cloud, PointCloudPtr rect_cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr_RGB resultCloud);

#endif // FILE_IO_H
#ifndef BIGSCENE_PRESEG_H
#define BIGSCENE_PRESEG_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "common_func.h"
#include "common_type.h"
#include "visualizer.h"
#include "utility.h"

using namespace std;
using namespace Wm5;

#define PI 3.1415926535
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 

#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d.h>

//detct floor
void detect_floor_and_walls(PointCloudPtr_RGB cloud, MyPointCloud_RGB& floor_cloud, pcl::ModelCoefficients& floor_coefficients, MyPointCloud& floor_rect_cloud, vector<MyPointCloud_RGB> &wall_clouds, std::vector<MyPointCloud> &wall_rect_clouds, PointCloudPtr_RGB remained_cloud);
//mark cloud by bounding box
void mark_cloud(PointCloudPtr_RGB sourceCloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr box_cloud);
//get transform matrix between plane and x_y plane
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, MyPointCloud& rect_cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r);
//remove floor_bottom and wall_back
void remove_outliers(PointCloudPtr_RGB remained_cloud, MyPointCloud& floor_rect_cloud, std::vector<MyPointCloud>& wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr_RGB new_remained_cloud, Visualizer& vs);
//pre-segment scene
void pre_segment_scene(PointCloudPtr_RGB cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, vector<MyPointCloud_RGB>& cluster_projected_pcs, vector<MyPointCloud_RGB>& cluster_origin_pcs, PointCloudPtr_RGB colored_projected_pc, PointCloudPtr_RGB colored_origin_pc);
//Set Priority for Clusters
void setPriorityforClusters(vector<MyPointCloud_RGB>& cluster_projected_pcs, vector<MyPointCloud_RGB>& cluster_origin_pcs ,vector<int>& priority_vec);
//get position that robot should go
void getRobotPosition(PointCloudPtr_RGB sourceCloud, vector<MyPointCloud> &wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, Point& position, Visualizer& vs);
//get position that robot should go
void getRobotPosition1(PointCloudPtr_RGB sourceCloud, vector<MyPointCloud> &wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, Point& position, Visualizer& vs);
#endif //BIGSCENE_PRESEG_H

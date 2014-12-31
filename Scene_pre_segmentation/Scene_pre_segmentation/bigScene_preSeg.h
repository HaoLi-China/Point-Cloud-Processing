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

#include <Wm5IntrSegment2Segment2.h>
#include <Wm5IntrTriangle3Triangle3.h>

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

//void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud);
//void appendCloud(PointCloudPtr sourceCloud,PointCloudPtr targetCloud);

//detct floor
void detect_floor_and_walls(PointCloudPtr_RGB cloud, MyPointCloud_RGB& floor_cloud, pcl::ModelCoefficients& floor_coefficients, MyPointCloud& floor_rect_cloud, vector<MyPointCloud_RGB> &wall_clouds, std::vector<MyPointCloud> &wall_rect_clouds, PointCloudPtr_RGB remained_cloud);
//Euclidean Cluster Extraction
void big_object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points);
//mark remaining cloud by bounding box
void mark_remaining_cloud(PointCloudPtr_RGB sourceCloud, PointCloudPtr cloud);
//compute gaussian curvature
void compute_gaussian_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored);
//compute gaussian curvature
void compute_mean_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored);
//projecte curvature to x_y plane
void curvature_projected(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& projected_curvatures);
//normalize projected curvature
void normalize_projected_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& projected_curvatures, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& new_curvatures, int *rows, int *cols);
//Gaussian Blur
void gaussian_blur(vector<Point_Cur_RGB>& curvatures, int rows, int cols, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& new_curvatures);
//segment scene
void segment_scene(PointCloudPtr_RGB cloud, vector<MyPointCloud>& support_plane_clouds, vector<MyPointCloud>& separation_rect_clouds, vector<MyPointCloud>& clustering_cloud, Visualizer& vs);
//detct support plane
void detect_support_plane(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &support_clouds, std::vector<MyPointCloud> &support_rect_clouds, PointCloudPtr_RGB remained_cloud);
//detct separation plane
void detect_separation_plane(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &separation_clouds, std::vector<MyPointCloud> &separation_rect_clouds, PointCloudPtr_RGB remained_cloud);
//get transform matrix between plane and x_y plane
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, MyPointCloud& rect_cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r);
//remove floor_bottom and wall_back
void remove_outliers(PointCloudPtr_RGB remained_cloud, MyPointCloud& floor_rect_cloud, std::vector<MyPointCloud>& wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr_RGB new_remained_cloud, Visualizer& vs);
//pre-segment scene
void pre_segment_scene(PointCloudPtr_RGB transformed_cloud, vector<MyPointCloud>& sum_support_clouds, vector<MyPointCloud>& sum_separation_rect_clouds);
#endif //BIGSCENE_PRESEG_H

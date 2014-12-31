#ifndef COMMON_FUNC_H
#define COMMON_FUNC_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <vector>

#include <Wm5IntrTriangle3Triangle3.h>
#include <Wm5IntrTriangle3Sphere3.h>
#include <Wm5IntrTriangle3Cylinder3.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

#include "common_type.h"

#define PI 3.1415926535

using namespace std;
using namespace Wm5;

//compute bounding box
void com_bounding_box(PointCloudPtr_RGB cloud,float *min_x,float *min_y,float *min_z, float *max_x, float *max_y, float *max_z);
//compute max value, min value, and average value along z axis of the point cloud
void com_max_and_min_and_avg_z(PointCloudPtr_RGB cloud,float *min_z,float *max_z,float *avg_z);
//append a cloud to another cloud
void appendCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL sourceCloud,PointCloudPtr_RGB_NORMAL targetCloud);
//append a cloud to another cloud
void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud);
//append a cloud to another cloud
void appendCloud(PointCloudPtr sourceCloud,PointCloudPtr targetCloud);
//get rotation matrix
void getRotationMatrix(Eigen::Vector3d &axis, double angleArc, Eigen::Matrix4d &matrix);
//for least square
bool getPlaneByLeastSquare(PointCloudPtr_RGB cloud_all_in_plane, pcl::ModelCoefficients::Ptr coefficients);
//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3);
//pcl pointCloud_RGB pop up
void pointCloud_RGBPopUp(PointCloudPtr_RGB cloud);
//pcl pointCloud_RGB_NORMA pop up
void pointCloud_RGB_NORMAPopUp(PointCloudPtr_RGB_NORMAL cloud);
//get Rect For PlaneCloud
void getRectForPlaneCloud(PointCloudPtr_RGB plane_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr rect_cloud);
//sample rect
void samplePlane(MyPointCloud& rect_mpc, float grid_length, MyPointCloud& sample_mpt, int *rows, int *cols);
//sample cylinder
void sampleCylinder(Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, MyPointCloud& mpt);
//sample sphere
void sampleSphere(Point cenPoint, float r, float grid_length, MyPointCloud& mpt);
//if a point is in a cloud
bool isPointInCloud(Point pt, PointCloudPtr cloud);
//find nearest neighbor
bool findNearestNeighbor(PointCloudPtr cloud, PointCloudPtr except_cloud, Point search_pt, Point& finded_pt);
//get intersection points
void get_intr_points(MyPointCloud& source_mpc, MyPointCloud& sample_mpc, float search_r, int* intr_points_num);
//compute Jaccard Index
void computeJaccardIndex(int a_num, int intr_num, float *result);
//compute Plane Jaccard Index
void computePlaneJaccardIndex(MyPointCloud& source_mpc, MyPointCloud& rect_mpc, float grid_length, float rate_threshold, float *result);
//compute Cylinder Jaccard Index
void computeCylinderJaccardIndex(MyPointCloud& source_mpc, Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, float *result);
//compute Sphere Jaccard Index
void computeSphereJaccardIndex(MyPointCloud& source_mpc, Point cenPoint, float r, float grid_length, float *result);
//Wm5IntrTriangle3Triangle3
bool testIntrTriangle3Triangle3(MyPt p00,MyPt p01,MyPt p02, MyPt p10,MyPt p11,MyPt p12);
//If a rectangle intersects with the other
bool testIntrRectangle3Rectangle3(MyPt p0_0, MyPt p0_1,MyPt p0_2,MyPt p0_3, MyPt p1_0,MyPt p1_1,MyPt p1_2,MyPt p1_3);
//rotate a 2d point by a 2d point
void rotatePoint2ByPoint2(float p_x,float p_y,float cen_x,float cen_y,float ang,float *new_x,float *new_y);

#endif
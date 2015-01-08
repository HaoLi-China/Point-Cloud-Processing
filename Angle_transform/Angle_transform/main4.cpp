//#include "visualizer.h"
//#include "file_io.h"
//#include "utility.h"
//#include "bigScene_preSeg.h"
//#include <pcl/io/ply_io.h>
//
//const float PIOver2=1.570796;
//
//void quaternionToMatrix44(float x, float y, float z, float w, Eigen::Matrix4f& mat)
//{
//  mat = Eigen::Matrix4f::Identity();
//
//  /* mat(0,0) = (2. * (x*x + w*w) -1.);
//  mat(0,1) = (2. * (x*y - z*w));
//  mat(0,2) = (2. * (x*z + y*w));
//  mat(1,0) = (2. * (x*y + z*w));
//  mat(1,1) = (2. * (y*y + w*w)-1.);
//  mat(1,2) = (2. * (y*z - x*w));
//  mat(2,0) = (2. * (x*z - y*w));
//  mat(2,1) = (2. * (y*z + x*w));
//  mat(2,2) = (2. * (z*z + w*w)-1.);*/
//
//  mat(0,0) = (1.0f - 2.0f * (y*y + z*z));
//  mat(0,1) = (2.0f * (x*y + z*w));
//  mat(0,2) = (2.0f * (x*z - y*w));
//  mat(1,0) = (2.0f * (x*y - z*w));
//  mat(1,1) = (1.0f - 2.0f * (x*x + z*z));
//  mat(1,2) = (2.0f * (y*z + x*w));
//  mat(2,0) = (2.0f * (x*z + y*w));
//  mat(2,1) = (2.0f * (y*z - x*w));
//  mat(2,2) = (1.0f - 2.0f * (x*x + y*y));
//}
//
//void Matrix44ToQuaternion(Eigen::Matrix4f& matrix_transform, float *x, float *y, float *z, float *w)
//{
//  float w_squared=matrix_transform(0,0)+matrix_transform(1,1)+matrix_transform(2,2);
//  float x_squared=matrix_transform(0,0)-matrix_transform(1,1)-matrix_transform(2,2);
//  float y_squared=matrix_transform(1,1)-matrix_transform(0,0)-matrix_transform(2,2);
//  float z_squared=matrix_transform(2,2)-matrix_transform(0,0)-matrix_transform(1,1);
//
//  int max_index=0;
//
//  float max_squared=w_squared;
//
//  if(max_squared<x_squared){
//    max_squared=x_squared;
//    max_index=1;
//  }
//
//  if(max_squared<y_squared){
//    max_squared=y_squared;
//    max_index=2;
//  }
//
//  if(max_squared<z_squared){
//    max_squared=z_squared;
//    max_index=3;
//  }
//
//  float biggest_val=sqrt(max_squared+1.0f)*0.5f;
//  float mult=0.25f/biggest_val;
//
//  switch(max_index){
//  case 0:
//    *w=biggest_val;
//    *x=(matrix_transform(1,2)-matrix_transform(2,1))*mult;
//    *y=(matrix_transform(2,0)-matrix_transform(0,2))*mult;
//    *z=(matrix_transform(0,1)-matrix_transform(1,0))*mult;
//    break;
//  case 1:
//    *x=biggest_val;
//    *w=(matrix_transform(1,2)-matrix_transform(2,1))*mult;
//    *y=(matrix_transform(0,1)+matrix_transform(1,0))*mult;
//    *z=(matrix_transform(2,0)+matrix_transform(0,2))*mult;
//    break;
//  case 2:
//    *y=biggest_val;
//    *w=(matrix_transform(2,0)-matrix_transform(0,2))*mult;
//    *x=(matrix_transform(0,1)+matrix_transform(1,0))*mult;
//    *z=(matrix_transform(1,2)+matrix_transform(2,1))*mult;
//    break;
//  case 3:
//    *z=biggest_val;
//    *w=(matrix_transform(0,1)-matrix_transform(1,0))*mult;
//    *x=(matrix_transform(2,0)+matrix_transform(0,2))*mult;
//    *y=(matrix_transform(1,2)+matrix_transform(2,1))*mult;
//    break;
//  }
//}
//
//void QuaternionToEulerAngle(float x, float y, float z, float w, Eigen::Vector3f& angle)
//{
//  cout<<"x:"<<x<<"y:"<<y<<"z:"<<z<<endl;
//
//
//  /*angle[0] = atan2(2.0f * x * w + 2.0f * y * z, 1.0f - 2.0f * x*x - 2.0f * y*y);
//  angle[1] = asin(2.0f * (w*y-z*x));
//  angle[2] = atan2(2.0f * z * w + 2.0f * y * x, 1.0f - 2.0f * y*y - 2.0f * z*z);*/
//
//  float test = y*z + x*w;
//  if (test > 0.4999f)
//  {
//    angle[2] = 2.0f * atan2(y, w);
//    angle[1] = PIOver2;
//    angle[0] = 0.0f;
//    return;
//  }
//  if (test < -0.4999f)
//  {
//    angle[2] = 2.0f * atan2(y, w);
//    angle[1] = -PIOver2;
//    angle[0] = 0.0f;
//    return;
//  }
//  float sqx = x * x;
//  float sqy = y * y;
//  float sqz = z * z;
//
//  angle[0] = asin(2.0f * test);
//  angle[1] = atan2(2.0f * y * w - 2.0f * z * x, 1.0f - 2.0f * sqy - 2.0f * sqx);
//  angle[2] = atan2(2.0f * z * w - 2.0f * y * x, 1.0f - 2.0f * sqz - 2.0f * sqx);
//
//}
//
//
//void EulerAngleToQuaternion(const Eigen::Vector3f& angle, float *x, float *y, float *z, float *w)
//{
//  float cx = cos(angle[0]/2);
//  float sx = sin(angle[0]/2);
//  float cy = cos(angle[1]/2);
//  float sy = sin(angle[1]/2);
//  float cz = cos(angle[2]/2);
//  float sz = sin(angle[2]/2);
//
//  *w = cx*cy*cz + sx*sy*sz;
//  *x = sx*cy*cz - cx*sy*sz;
//  *y = cx*sy*cz + sx*cy*sz;
//  *z = cx*cy*sz - sx*sy*cz;
//}
//
//int main (int argc, char *argv[])
//{
//  Visualizer vs;
//  vs.viewer->removeAllPointClouds();
//  vs.viewer->removeCoordinateSystem();
//  vs.viewer->setBackgroundColor(0,0,0);
//
//  PointCloudPtr_RGB cloud(new PointCloud_RGB);
//  PointCloudPtr_RGB scan1(new PointCloud_RGB);
//  PointCloudPtr_RGB scan2(new PointCloud_RGB);
//  PointCloudPtr_RGB scan3(new PointCloud_RGB);
//  PointCloudPtr_RGB scan4(new PointCloud_RGB);
//  PointCloudPtr_RGB scan5(new PointCloud_RGB);
//  PointCloudPtr_RGB scan6(new PointCloud_RGB);
//  NormalCloudTPtr normals(new NormalCloudT);
//
//  PointCloudPtr_RGB cloud_out(new PointCloud_RGB);
//
//  /*loadPointCloud_ply("data/calibration_data.ply", cloud);
//  loadPointCloud_ply("data/room_tf/scan1.ply", scan1);
//  loadPointCloud_ply("data/room_tf/scan2.ply", scan2);
//  loadPointCloud_ply("data/room_tf/scan3.ply", scan3);
//  loadPointCloud_ply("data/room_tf/scan4.ply", scan4);
//  loadPointCloud_ply("data/room_tf/scan5.ply", scan5);
//  loadPointCloud_ply("data/room_tf/scan6.ply", scan6);
//
//  MyPointCloud_RGB plane_cloud1;
//  MyPointCloud rect_cloud1;
//  pcl::ModelCoefficients::Ptr plane_coefficients1(new pcl::ModelCoefficients);
//  PointCloudPtr_RGB remained_cloud1(new PointCloud_RGB);
//  big_plane_fitting(cloud, plane_cloud1, rect_cloud1, plane_coefficients1, remained_cloud1);
//
//  Eigen::Vector3d plane1_normal;
//  plane1_normal << plane_coefficients1->values[0], plane_coefficients1->values[1], plane_coefficients1->values[2];
//  plane1_normal.normalize();
//
//  double angle1=acos(plane1_normal.dot(Eigen::Vector3d(1,0,0)));
//  Eigen::Vector3d axis1 = plane1_normal.cross(Eigen::Vector3d(1,0,0));
//
//  Eigen::Matrix4d matrix1;
//  getRotationMatrix(axis1, angle1, matrix1);
//  Eigen::Matrix4f matrix_transform1 = matrix1.cast<float>();
//
//  MyPointCloud_RGB plane_cloud2;
//  MyPointCloud rect_cloud2;
//  pcl::ModelCoefficients::Ptr plane_coefficients2(new pcl::ModelCoefficients);
//  PointCloudPtr_RGB remained_cloud2(new PointCloud_RGB);
//  big_plane_fitting(remained_cloud1, plane_cloud2, rect_cloud2, plane_coefficients2, remained_cloud2);
//
//  Eigen::Vector3d plane2_normal;
//  plane2_normal << -plane_coefficients2->values[0], -plane_coefficients2->values[1], -plane_coefficients2->values[2];
//  plane2_normal.normalize();
//
//  double angle2=acos(plane2_normal.dot(Eigen::Vector3d(0,0,1)));
//  Eigen::Vector3d axis2 = plane2_normal.cross(Eigen::Vector3d(0,0,1));
//
//  Eigen::Matrix4d matrix2;
//  getRotationMatrix(axis2, angle2, matrix2);
//
//  Eigen::Matrix4f matrix_transform2 = matrix2.cast<float>();
//
//  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
//
//  Eigen::Matrix4d matrix3=matrix2*matrix1;
//  Eigen::Matrix4f matrix_transform3 = matrix3.cast<float>();
//  matrix_transform3(0, 3)+=0.676;
//  matrix_transform3(1, 3)+=0.0524;
//  matrix_transform3(2, 3)+=1.33;
//
//  pcl::transformPointCloud(*scan1, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan1);
//  pcl::transformPointCloud(*scan2, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan2);
//  pcl::transformPointCloud(*scan3, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan3);
//  pcl::transformPointCloud(*scan4, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan4);
//  pcl::transformPointCloud(*scan5, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan5);
//  pcl::transformPointCloud(*scan6, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan6);
//
//  pcl::io::savePCDFileASCII ("data/scan1_out.pcd", *scan1);
//  pcl::io::savePCDFileASCII ("data/scan2_out.pcd", *scan2);
//  pcl::io::savePCDFileASCII ("data/scan3_out.pcd", *scan3);
//  pcl::io::savePCDFileASCII ("data/scan4_out.pcd", *scan4);
//  pcl::io::savePCDFileASCII ("data/scan5_out.pcd", *scan5);
//  pcl::io::savePCDFileASCII ("data/scan6_out.pcd", *scan6);*/
//
//  loadPointCloud_ply("data/calibration_data.ply", cloud);
//  loadPointCloud_ply("data/table.ply", scan1);
//
//  MyPointCloud_RGB plane_cloud1;
//  MyPointCloud rect_cloud1;
//  pcl::ModelCoefficients::Ptr plane_coefficients1(new pcl::ModelCoefficients);
//  PointCloudPtr_RGB remained_cloud1(new PointCloud_RGB);
//  big_plane_fitting(cloud, plane_cloud1, rect_cloud1, plane_coefficients1, remained_cloud1);
//
//  Eigen::Vector3d plane1_normal;
//  plane1_normal << plane_coefficients1->values[0], plane_coefficients1->values[1], plane_coefficients1->values[2];
//  plane1_normal.normalize();
//
//  double angle1=acos(plane1_normal.dot(Eigen::Vector3d(1,0,0)));
//  Eigen::Vector3d axis1 = plane1_normal.cross(Eigen::Vector3d(1,0,0));
//
//  Eigen::Matrix4d matrix1;
//  getRotationMatrix(axis1, angle1, matrix1);
//  Eigen::Matrix4f matrix_transform1 = matrix1.cast<float>();
//
//  MyPointCloud_RGB plane_cloud2;
//  MyPointCloud rect_cloud2;
//  pcl::ModelCoefficients::Ptr plane_coefficients2(new pcl::ModelCoefficients);
//  PointCloudPtr_RGB remained_cloud2(new PointCloud_RGB);
//  big_plane_fitting(remained_cloud1, plane_cloud2, rect_cloud2, plane_coefficients2, remained_cloud2);
//
//  Eigen::Vector3d plane2_normal;
//  plane2_normal << -plane_coefficients2->values[0], -plane_coefficients2->values[1], -plane_coefficients2->values[2];
//  plane2_normal.normalize();
//
//  double angle2=acos(plane2_normal.dot(Eigen::Vector3d(0,0,1)));
//  Eigen::Vector3d axis2 = plane2_normal.cross(Eigen::Vector3d(0,0,1));
//
//  Eigen::Matrix4d matrix2;
//  getRotationMatrix(axis2, angle2, matrix2);
//
//  Eigen::Matrix4f matrix_transform2 = matrix2.cast<float>();
//
//  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
//
//  Eigen::Matrix4d matrix3=matrix2*matrix1;
//  Eigen::Matrix4f matrix_transform3 = matrix3.cast<float>();
//  matrix_transform3(0, 3)+=0.676;
//  matrix_transform3(1, 3)+=0.0524;
//  matrix_transform3(2, 3)+=1.33;
//
//  pcl::transformPointCloud(*scan1, *cloud_tem, matrix_transform3);
//  pcl::copyPointCloud(*cloud_tem, *scan1);
//  
//  showPointCloud(scan1, "scan1");
//
//  pcl::io::savePLYFileASCII("data/table_out.ply", *scan1);
//
//  vs.show();
//
//  return 0;
//}

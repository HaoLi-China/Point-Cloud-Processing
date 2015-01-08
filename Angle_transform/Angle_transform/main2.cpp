//#include "visualizer.h"
//#include "file_io.h"
//#include "utility.h"
//#include "bigScene_preSeg.h"
//
//const float PIOver2=1.570796;
//
//void quaternionToMatrix44(float x, float y, float z, float w, Eigen::Matrix4f& mat)
//{
//  mat = Eigen::Matrix4f::Identity();
//
// /* mat(0,0) = (2. * (x*x + w*w) -1.);
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
//    float test = y*z + x*w;
//    if (test > 0.4999f)
//    {
//    angle[2] = 2.0f * atan2(y, w);
//    angle[1] = PIOver2;
//    angle[0] = 0.0f;
//    return;
//    }
//    if (test < -0.4999f)
//    {
//    angle[2] = 2.0f * atan2(y, w);
//    angle[1] = -PIOver2;
//    angle[0] = 0.0f;
//    return;
//    }
//    float sqx = x * x;
//    float sqy = y * y;
//    float sqz = z * z;
//    
//    angle[0] = asin(2.0f * test);
//    angle[1] = atan2(2.0f * y * w - 2.0f * z * x, 1.0f - 2.0f * sqy - 2.0f * sqx);
//    angle[2] = atan2(2.0f * z * w - 2.0f * y * x, 1.0f - 2.0f * sqz - 2.0f * sqx);
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
//  PointCloudPtr_RGB cloud2(new PointCloud_RGB);
//  NormalCloudTPtr normals(new NormalCloudT);
//
//  PointCloudPtr_RGB cloud_out(new PointCloud_RGB);
//
//  loadPointCloud_ply("data/calibration_data.ply", cloud);
//  loadPointCloud_ply("data/calibration_origin.ply", cloud2);
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
//  PointCloudPtr_RGB new_cloud(new PointCloud_RGB);
// 
//  Eigen::Matrix4d matrix3=matrix2*matrix1;
//  Eigen::Matrix4f matrix_transform3 = matrix3.cast<float>();
//  matrix_transform3(0, 3)+=0.676;
//  matrix_transform3(1, 3)+=0.0524;
//  matrix_transform3(2, 3)+=1.33;
//
//  pcl::transformPointCloud(*cloud2, *new_cloud, matrix_transform3);
//  pcl::copyPointCloud(*new_cloud, *cloud2);
//
//  Eigen::Matrix4f test_mat=matrix_transform3.inverse();
//  pcl::transformPointCloud(*cloud2, *new_cloud, test_mat);
//  pcl::copyPointCloud(*new_cloud, *cloud2);
//  
//  vs.viewer->addPointCloud(cloud2, "new_cloud");
//
//  Eigen::Matrix4f matrix_base_to_led;
//  quaternionToMatrix44(-0.044, 0.187, 0.024, 0.981, matrix_base_to_led);
//
//  cout<<"matrix_base_to_led:\n"<<matrix_base_to_led<<endl;
//
//
//  Eigen::Matrix4f matrix_base_to_led_inverse = matrix_base_to_led.inverse();
//
//  matrix_transform3(0,3)=0;
//  matrix_transform3(1,3)=0;
//  matrix_transform3(2,3)=0;
//  matrix_transform3(3,3)=1.0f;
//
//  cout<<"matrix_transform3:\n"<<matrix_transform3<<endl;
//
//  float x;
//  float y;
//  float z;
//  float w;
//
//  Eigen::Matrix4f matrix_base_to_kinect = matrix_transform3.inverse();
//  cout<<"matrix_base_to_kinect_before:\n"<<matrix_base_to_kinect<<endl;
//
//
//  cout<<"bbbbb:\n"<<matrix_base_to_kinect*matrix_transform3<<endl;
//
//  Matrix44ToQuaternion(matrix_base_to_kinect, &x, &y, &z, &w);
//  cout<<"x:"<<x<<endl;
//  cout<<"y:"<<y<<endl;
//  cout<<"z:"<<z<<endl;
//  cout<<"w:"<<w<<endl;
//
//  quaternionToMatrix44(x, y, z, w, matrix_base_to_kinect);
//  cout<<"matrix_base_to_kinect_after:\n"<<matrix_base_to_kinect<<endl;
//
//
//  //Eigen::Matrix4f matrix_base_to_kinect_inverse = matrix_transform3.inverse();
//
//  Eigen::Matrix4f matrix_led_to_kinect = matrix_base_to_led_inverse*matrix_base_to_kinect;
//  cout<<"matrix_base_to_led_inverse:\n"<<matrix_base_to_led_inverse<<endl;
//  cout<<"matrix_led_to_kinect:\n"<<matrix_led_to_kinect<<endl;
//
//  Matrix44ToQuaternion(matrix_base_to_kinect, &x, &y, &z, &w);
//  cout<<"x:"<<x<<endl;
//  cout<<"y:"<<y<<endl;
//  cout<<"z:"<<z<<endl;
//  cout<<"w:"<<w<<endl;
//
//  Eigen::Vector3f angle;
//  QuaternionToEulerAngle(x, y, z, w, angle);
//  cout<<"angle:\n"<<angle<<endl;
//
//
//
//
//
//
//
//
//
//
//
//  //Eigen::Matrix4f matrix_test;
//  //quaternionToMatrix44(0, 0.707107, 0, 0.707107, matrix_test);
//  //cout<<"matrix_test:\n"<<matrix_test<<endl;
//
//  //Matrix44ToQuaternion(matrix_test, &x, &y, &z, &w);
//  //cout<<"x:"<<x<<endl;
//  //cout<<"y:"<<y<<endl;
//  //cout<<"z:"<<z<<endl;
//  //cout<<"w:"<<w<<endl;
//
//  //Eigen::Vector3f angle_test1;
//  //QuaternionToEulerAngle(x, y, z, w, angle_test1);
//  //cout<<"angle_test1:\n"<<angle_test1<<endl;
//
//  //EulerAngleToQuaternion(Eigen::Vector3f(0, 1.570796, 0), &x, &y, &z, &w);
//  //cout<<"x1:"<<x<<endl;
//  //cout<<"y2:"<<y<<endl;
//  //cout<<"z3:"<<z<<endl;
//  //cout<<"w4:"<<w<<endl;
//
//  //Eigen::Vector3f angle_test2;
//  //QuaternionToEulerAngle(x, y, z, w, angle_test2);
//  //cout<<"angle_test2:\n"<<angle_test2<<endl;
//
//
//  //Eigen::Vector3f angle_test3;
//  //QuaternionToEulerAngle(0, 0.707107, 0, 0.707107, angle_test3);
//  //cout<<"angle_test3:\n"<<angle_test3<<endl;
//
//  //Eigen::Vector3f angle_test4;
//  //QuaternionToEulerAngle(0.0f, 0.707107f, 0.0f, 0.707107f, angle_test4);
//  //cout<<"angle_test4:\n"<<angle_test4<<endl;
//  ////float x;
//  //float y;
//  //float z;
//  //float w;
//
//  ////Matrix44ToQuaternion(matrix_led_to_kinect, &x, &y, &z, &w);
//  //Matrix44ToQuaternion(matrix_base_to_kinect, &x, &y, &z, &w);
//  //cout<<"x2:"<<x<<endl;
//  //cout<<"y2:"<<y<<endl;
//  //cout<<"z2:"<<z<<endl;
//  //cout<<"w2:"<<w<<endl;
//
//  //Eigen::Vector3f angle;
//  //GetEulerAngle(x, y, z, w, angle);
//
//  //cout<<"angle:"<<angle<<endl;
//
//  //CQuaternion(Eigen::Vector3f(-0.083, 0.377, 0.033), &x, &y, &z, &w);
//  //cout<<"x3:"<<x<<endl;
//  //cout<<"y3:"<<y<<endl;
//  //cout<<"z3:"<<z<<endl;
//  //cout<<"w3:"<<w<<endl;
//
//  //GetEulerAngle(x, y, z, w, angle);
//  //cout<<"angle2:"<<angle<<endl;
//  vs.show();
//
//  return 0;
//}

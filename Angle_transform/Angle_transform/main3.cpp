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
//  mat(0,0) = (2. * (x*x + w*w) -1.);
//  mat(0,1) = (2. * (x*y - z*w));
//  mat(0,2) = (2. * (x*z + y*w));
//  mat(1,0) = (2. * (x*y + z*w));
//  mat(1,1) = (2. * (y*y + w*w)-1.);
//  mat(1,2) = (2. * (y*z - x*w));
//  mat(2,0) = (2. * (x*z - y*w));
//  mat(2,1) = (2. * (y*z + x*w));
//  mat(2,2) = (2. * (z*z + w*w)-1.);
//}
//
//void Matrix44ToQuaternion(Eigen::Matrix4f& matrix_transform, float *x, float *y, float *z, float *w)
//{
//  float w_squared=matrix_transform(0,0)+matrix_transform(1,1)+matrix_transform(2,2);
//  float x_squared=matrix_transform(0,0)-matrix_transform(1,1)-matrix_transform(2,2);
//  float y_squared=matrix_transform(1,1)+matrix_transform(0,0)+matrix_transform(2,2);
//  float z_squared=matrix_transform(2,2)+matrix_transform(0,0)+matrix_transform(1,1);
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
//
//
//
//
//
//void GetEulerAngle(float x, float y, float z, float w, Eigen::Vector3f& angle)
//{
//  /*float test = y*z + x*w;
//  if (test > 0.4999f)
//  {
//  angle[2] = 2.0f * atan2(y, w);
//  angle[1] = PIOver2;
//  angle[0] = 0.0f;
//  return;
//  }
//  if (test < -0.4999f)
//  {
//  angle[2] = 2.0f * atan2(y, w);
//  angle[1] = -PIOver2;
//  angle[0] = 0.0f;
//  return;
//  }
//  float sqx = x * x;
//  float sqy = y * y;
//  float sqz = z * z;
//  angle[2] = atan2(2.0f * z * w - 2.0f * y * x, 1.0f - 2.0f * sqz - 2.0f * sqx);
//  angle[1] = asin(2.0f * test);
//  angle[0] = atan2(2.0f * y * w - 2.0f * z * x, 1.0f - 2.0f * sqy - 2.0f * sqx);*/
//
//  angle[0] = atan2(2.0f * x * w + 2.0f * y * z, 1.0f - 2.0f * x*x - 2.0f * y*y);
//  angle[1] = asin(2.0f * (w*y-z*x));
//  angle[2] = atan2(2.0f * z * w + 2.0f * y * x, 1.0f - 2.0f * y*y - 2.0f * z*z);
//
//}
//
//void CQuaternion(Eigen::Matrix4f& matrix_transform, float *x, float *y, float *z, float *w)
//{
//  float tr, s, q[4];
//  int i, j, k;
//
//  int nxt[3] = {1, 2, 0 };
//  // 计算矩阵轨迹
//  tr = matrix_transform(0,0) + matrix_transform(1,1) + matrix_transform(2,2);
//
//  // 检查矩阵轨迹是正还是负
//  if(tr>0.0f)
//  {
//    s = sqrt(tr + 1.0f);
//    *w = s / 2.0f;
//    s = 0.5f / s;
//    *x = (matrix_transform(1,2) - matrix_transform(2,1)) * s;
//    *y = (matrix_transform(2,0) - matrix_transform(0,2)) * s;
//    *z = (matrix_transform(0,1) - matrix_transform(1,0)) * s;
//  }
//  else
//  {
//    // 轨迹是负
//    // 寻找m11 m22 m33中的最大分量
//    i = 0;
//    if(matrix_transform(1,1)>matrix_transform(0,0)) i = 1;
//    if(matrix_transform(2,2)>matrix_transform(i,i)) i = 2;
//    j = nxt[i];
//    k = nxt[j];
//
//    s = sqrt((matrix_transform(i,i) - (matrix_transform(j,j) + matrix_transform(k,k))) + 1.0f);
//    q[i] = s * 0.5f;
//    if( s!= 0.0f) s = 0.5f / s;
//    q[3] = (matrix_transform(j,k) - matrix_transform(k,j)) * s;
//    q[j] = (matrix_transform(i,j) - matrix_transform(j,i)) * s;
//    q[k] = (matrix_transform(i,k) - matrix_transform(k,i)) * s;
//    *x = q[0];
//    *y = q[1];
//    *z = q[2];
//    *w = q[3];
//  }
//}
//
//
//void CQuaternion(const Eigen::Vector3f& angle, float *x, float *y, float *z, float *w)
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
//void GetMatrixRH(float x, float y, float z, float w, Eigen::Vector3f& translation, Eigen::Matrix4f& matrix_transform)
//{
//  float xx = x*x;
//  float yy = y*y;
//  float zz = z*z;
//  float xy = x*y;
//  float wz = -w*z;
//  float wy = -w*y;
//  float xz = x*z;
//  float yz = y*z;
//  float wx = -w*x;
//
//  matrix_transform(0,0) = 1.0f-2*(yy+zz);
//  matrix_transform(0,1) = 2*(xy-wz);
//  matrix_transform(0,2) = 2*(wy+xz);
//  matrix_transform(0,3) = translation[0];
//
//  matrix_transform(1,0) = 2*(xy+wz);
//  matrix_transform(1,1) = 1.0f-2*(xx+zz);
//  matrix_transform(1,2) = 2*(yz-wx);
//  matrix_transform(1,3) = translation[1];
//
//  matrix_transform(2,0) = 2*(xz-wy);
//  matrix_transform(2,1) = 2*(yz+wx);
//  matrix_transform(2,2) = 1.0f-2*(xx+yy);
//  matrix_transform(2,3) = translation[2];
//
//  matrix_transform(3,0) = 0;
//  matrix_transform(3,1) = 0;
//  matrix_transform(3,2) = 0;
//  matrix_transform(3,3) = 1.0f;
//
//
//
//  //float xx = x*x;
//  //float yy = y*y;
//  //float zz = z*z;
//  //float xy = x*y;
//  //float wz = w*z;
//  //float wy = w*y;
//  //float xz = x*z;
//  //float yz = y*z;
//  //float wx = w*x;
//
//  //matrix_transform(0,1) = 1.0f-2*(yy+zz);
//  //matrix_transform(0,2) = 2*(xy-wz);
//  //matrix_transform(0,3) = 2*(wy+xz);
//  //matrix_transform(0,4) = 0.0f;
//
//  //matrix_transform(1,1) = 2*(xy+wz);
//  //matrix_transform(1,2) = 1.0f-2*(xx+zz);
//  //matrix_transform(1,3) = 2*(yz-wx);
//  //matrix_transform(1,4) = 0.0f;
//
//  //matrix_transform(2,1) = 2*(xy-wy);
//  //matrix_transform(2,2) = 2*(yz+wx);
//  //matrix_transform(2,3) = 1.0f-2*(xx+yy);
//  //matrix_transform(2,3) = 0.0f;
//
//  //matrix_transform(3,1) = translation[0];
//  //matrix_transform(3,2) = translation[1];
//  //matrix_transform(3,3) = translation[2];
//  //matrix_transform(3,4) = 1.0f;
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
//
//  Eigen::Vector3d new_axis = plane2_normal.cross(plane1_normal);
//  double angle3=acos(new_axis.dot(Eigen::Vector3d(0,1,0)));
//
//
//  cout<<"plane1_normal:"<<plane1_normal<<endl;
//  cout<<"plane2_normal:"<<plane2_normal<<endl;
//  cout<<"new_axis:"<<new_axis<<endl;
//
//  cout<<"angle1:"<<angle1<<endl;
//  cout<<"angle2:"<<angle2<<endl;
//  cout<<"angle3:"<<angle3<<endl;
//
//
//
//  vs.show();
//
//  return 0;
//}

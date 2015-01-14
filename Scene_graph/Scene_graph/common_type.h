#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>

using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZRGB Point_RGB;
typedef pcl::PointCloud<Point_RGB> PointCloud_RGB;
typedef PointCloud_RGB::Ptr PointCloudPtr_RGB;
typedef PointCloud_RGB::ConstPtr PointCloudConstPtr_RGB;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> NormalCloudT;
typedef NormalCloudT::Ptr NormalCloudTPtr;
typedef NormalCloudT::ConstPtr NormalCloudTConstPtr;

typedef pcl::PointXYZRGBNormal Point_RGB_NORMAL;
typedef pcl::PointCloud<Point_RGB_NORMAL> PointCloud_RGB_NORMAL;
typedef PointCloud_RGB_NORMAL::Ptr PointCloudPtr_RGB_NORMAL;
typedef PointCloud_RGB_NORMAL::ConstPtr PointCloudConstPtr_RGB_NORMAL;

typedef struct MyPoint{
  float x;
  float y;
  float z;
}MyPt;

typedef struct MyPoint_RGB{
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float a;
}MyPt_RGB;

typedef struct MyPoint_RGB_NORMAL{
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float a;
  float normal_x;
  float normal_y;
  float normal_z;
}MyPt_RGB_NORMAL;

class MyPointCloud{
public:
  MyPointCloud();
  ~MyPointCloud();

public:
  vector<MyPt> mypoints;
};

class MyPointCloud_RGB{
public:
  MyPointCloud_RGB();
  ~MyPointCloud_RGB();

public:
  vector<MyPt_RGB> mypoints;
};

class MyPointCloud_RGB_NORMAL{
public:
  MyPointCloud_RGB_NORMAL();
  ~MyPointCloud_RGB_NORMAL();

public:
  vector<MyPt_RGB_NORMAL> mypoints;
};

class MyLine{
public:
  MyLine();
  ~MyLine();

public:
  MyPt p0;
  MyPt p1;
};

typedef struct Point_Curvature_RGB{
  float x;
  float y;
  float z;
  float curvature;
  float r;
  float g;
  float b;
}Point_Cur_RGB;

void MyPointCloud2PointCloud(MyPointCloud& mc, PointCloudPtr pc);
void PointCloud2MyPointCloud(PointCloudPtr pc, MyPointCloud& mc);

void MyPointCloud_RGB2PointCloud_RGB(MyPointCloud_RGB& mc, PointCloudPtr_RGB pc);
void PointCloud_RGB2MyPointCloud_RGB(PointCloudPtr_RGB pc, MyPointCloud_RGB& mc);

void MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(MyPointCloud_RGB_NORMAL& mc, PointCloudPtr_RGB_NORMAL pc);
void PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL pc, MyPointCloud_RGB_NORMAL& mc);

void MyPointCloud_RGB2MyPointCloud(MyPointCloud_RGB& mc_rgb, MyPointCloud& mc);
void MyPointCloud_RGB_NORMAL2MyPointCloud(MyPointCloud_RGB_NORMAL& mc_rgb, MyPointCloud& mc);

void CopyMyPointCloud_RGB(MyPointCloud_RGB& source, MyPointCloud_RGB& target);
void CopyMyPointCloud(MyPointCloud& source, MyPointCloud& target);
#endif 
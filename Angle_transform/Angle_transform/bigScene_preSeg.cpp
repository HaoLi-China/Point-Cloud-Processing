#include "bigScene_preSeg.h"

#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//for least square
bool getPlaneByLeastSquare(PointCloudPtr_RGB cloud_all_in_plane, pcl::ModelCoefficients::Ptr coefficients)
{
  double coeffA = 0.0, coeffB = 0.0, coeffC = 0.0, coeffD = 0.0;
  int matrixSize = 3;
  Eigen::Matrix3Xd mpara( matrixSize, matrixSize );
  for (int i =0; i<matrixSize;i++){
    for (int j=0; j<matrixSize;j++){
      mpara(i,j) = 0;
    }
  }

  double sumx, sumy, sumz;
  double averx, avery, averz;

  sumx=sumy=sumz=0;

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    sumx+=cloud_all_in_plane->points.at(i).x;
    sumy+=cloud_all_in_plane->points.at(i).y;
    sumz+=cloud_all_in_plane->points.at(i).z;
  }

  averx=sumx/cloud_all_in_plane->points.size();
  avery=sumy/cloud_all_in_plane->points.size();
  averz=sumz/cloud_all_in_plane->points.size();

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    mpara( 0, 0 ) += pow( cloud_all_in_plane->points.at(i).x - averx, 2 );
    mpara( 0, 1 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 0, 2 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 1, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 1, 1 ) += pow( cloud_all_in_plane->points.at(i).y - avery, 2 );
    mpara( 1, 2 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 2, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 1 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 2 ) += pow( cloud_all_in_plane->points.at(i).z - averz, 2 );
  }

  Eigen::EigenSolver<Eigen::Matrix3Xd> msolver( mpara );
  complex<double> lambda1 = msolver.eigenvalues()[0];
  complex<double> lambda2 = msolver.eigenvalues()[1];
  complex<double> lambda3 = msolver.eigenvalues()[2];
  int minEigenValue = (( lambda1.real() < lambda2.real() ) ? 0 :1 );
  minEigenValue = (( msolver.eigenvalues()[minEigenValue].real() < lambda3.real() )? minEigenValue : 2);
  coeffA = msolver.eigenvectors().col(minEigenValue)[0].real();
  coeffB = msolver.eigenvectors().col(minEigenValue)[1].real();
  coeffC = msolver.eigenvectors().col(minEigenValue)[2].real();
  coeffD = -( coeffA * averx + coeffB * avery + coeffC * averz );

  cout<<endl;
  cout<<coeffA<<"==========="<<coeffB<<"=============="<<coeffC<<"============"<<coeffD<<endl;

  coefficients->values.push_back(coeffA);
  coefficients->values.push_back(coeffB);
  coefficients->values.push_back(coeffC);
  coefficients->values.push_back(coeffD);

  return true;
}


//compute bounding box
void com_bounding_box(PointCloudPtr_RGB cloud,float *min_x,float *min_y,float *min_z, float *max_x, float *max_y, float *max_z){
  *min_x=cloud->points[0].x;
  *min_y=cloud->points[0].y;
  *min_z=cloud->points[0].z;
  *max_x=cloud->points[0].x;
  *max_y=cloud->points[0].y;
  *max_z=cloud->points[0].z;

  for (int i=0; i<cloud->size(); ++i) {
    float x, y, z;
    x=cloud->points[i].x;
    y=cloud->points[i].y;
    z=cloud->points[i].z;

    if(x<(*min_x)){
      (*min_x)=x;
    }
    else if(x>(*max_x)){
      (*max_x)=x;
    }

    if(y<(*min_y)){
      (*min_y)=y;
    }
    else if(y>(*max_y)){
      (*max_y)=y;
    }

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }
}

//compute max value, min value, and average value along z axis of the point cloud
void com_max_and_min_and_avg_z(PointCloudPtr_RGB cloud,float *min_z,float *max_z,float *avg_z){
  *min_z=cloud->points[0].z;
  *max_z=cloud->points[0].z;

  float sum_z=0;

  for (int i=0; i<cloud->size(); ++i) {
    float z;

    z=cloud->points[i].z;

    sum_z+=z;

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }

  *avg_z=sum_z/cloud->size();
}

//append a cloud to another cloud
void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//append a cloud to another cloud
void appendCloud(PointCloudPtr sourceCloud,PointCloudPtr targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//get rotation matrix
void getRotationMatrix(Eigen::Vector3d &axis, double angleArc, Eigen::Matrix4d &matrix)
{
  axis.normalize();

  matrix(0,0) = cos(angleArc)+(1-cos(angleArc))*axis(0)*axis(0) ;
  matrix(1,0) = (1-cos(angleArc))*axis(0)*axis(1) + sin(angleArc)*axis(2);
  matrix(2,0) = (1-cos(angleArc))*axis(0)*axis(2)-sin(angleArc)*axis(1);
  matrix(3,0) = 0;

  matrix(0,1) = (1-cos(angleArc))*axis(0)*axis(1) -sin(angleArc)*axis(2);
  matrix(1,1) = cos(angleArc)+(1-cos(angleArc))*axis(1)*axis(1);
  matrix(2,1) = (1-cos(angleArc))*axis(2)*axis(1) + sin(angleArc)*axis(0);
  matrix(3,1) = 0;

  matrix(0,2) = (1-cos(angleArc))*axis(2)*axis(0) + sin(angleArc)*axis(1);
  matrix(1,2) = (1-cos(angleArc))*axis(2)*axis(1) - sin(angleArc)*axis(0);
  matrix(2,2) = cos(angleArc) + (1-cos(angleArc))*axis(2)*axis(2);
  matrix(3,2) = 0;

  matrix(0,3) = 0;
  matrix(1,3) = 0;
  matrix(2,3) = 0;
  matrix(3,3) = 1;
}

//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3){
  std::vector<cv::Point2f> points_clu_2d;

  for(int j=0;j<cloud->points.size();j++){
    points_clu_2d.push_back(cv::Point2f(cloud->points[j].x, cloud->points[j].y));
  }

  cv::RotatedRect rect = cv::minAreaRect(cv::Mat(points_clu_2d));

  float width= rect.size.width;
  float height= rect.size.height;

  p0.x=rect.center.x-width/2.0;
  p0.y=rect.center.y-height/2.0;

  p1.x=rect.center.x-width/2.0;
  p1.y=rect.center.y+height/2.0;

  p2.x=rect.center.x+width/2.0;
  p2.y=rect.center.y+height/2.0;

  p3.x=rect.center.x+width/2.0;
  p3.y=rect.center.y-height/2.0;

  float ang=(rect.angle/180.0)*PI;

  float x0=rect.center.x+(p0.x-rect.center.x)*cos(ang)-(p0.y-rect.center.y)*sin(ang);
  float y0=rect.center.y+(p0.x-rect.center.x)*sin(ang)+(p0.y-rect.center.y)*cos(ang);

  float x1=rect.center.x+(p1.x-rect.center.x)*cos(ang)-(p1.y-rect.center.y)*sin(ang);
  float y1=rect.center.y+(p1.x-rect.center.x)*sin(ang)+(p1.y-rect.center.y)*cos(ang);

  float x2=rect.center.x+(p2.x-rect.center.x)*cos(ang)-(p2.y-rect.center.y)*sin(ang);
  float y2=rect.center.y+(p2.x-rect.center.x)*sin(ang)+(p2.y-rect.center.y)*cos(ang);

  float x3=rect.center.x+(p3.x-rect.center.x)*cos(ang)-(p3.y-rect.center.y)*sin(ang);
  float y3=rect.center.y+(p3.x-rect.center.x)*sin(ang)+(p3.y-rect.center.y)*cos(ang);

  p0.x=x0;
  p0.y=y0;

  p1.x=x1;
  p1.y=y1;

  p2.x=x2;
  p2.y=y2;

  p3.x=x3;
  p3.y=y3;
}

//pcl pointCloud pop up
void pointCloudPopUp(PointCloudPtr_RGB cloud){
  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<cloud->size()-1;i++){
    pc->push_back(cloud->at(i));
  }

  cloud->clear();

  pcl::copyPointCloud(*pc,*cloud);
}

//get Rect For PlaneCloud
void getRectForPlaneCloud(PointCloudPtr_RGB plane_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr rect_cloud){
  PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB);

  PointCloudPtr_RGB plane_cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*plane_cloud,*plane_cloud_tem);

  Point_RGB pr;
  pr.x=0;
  pr.y=0;
  pr.z=(-plane_coefficients->values[3])/plane_coefficients->values[2];

  plane_cloud_tem->push_back(pr);

  Eigen::Vector3d plane_normal;
  plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
  plane_normal.normalize();

  double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);

  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  pcl::transformPointCloud (*plane_cloud_tem, *cloud_in_plane, matrix_transform);

  Point_RGB new_pr=cloud_in_plane->at(cloud_in_plane->size()-1);

  pointCloudPopUp(cloud_in_plane);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
  find_min_rect(cloud_in_plane, p0,p1,p2,p3);

  PointCloudPtr points(new PointCloud());
  points->push_back(Point(p0.x,p0.y,new_pr.z));
  points->push_back(Point(p1.x,p1.y,new_pr.z));
  points->push_back(Point(p2.x,p2.y,new_pr.z));
  points->push_back(Point(p3.x,p3.y,new_pr.z));

  Eigen::Matrix4d matrix_reverse;
  getRotationMatrix(axis, -angle, matrix_reverse);

  Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
  pcl::transformPointCloud (*points, *rect_cloud, matrix_transform_reverse);
}

//sample rect
void samplePlane(MyPointCloud& rect_mpc, float grid_length, MyPointCloud& sample_mpt){
  sample_mpt.mypoints.clear();

  float w=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(1).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(1).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(1).z, 2));
  float h=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(3).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(3).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(3).z, 2));

  int num_w=(int)(w/grid_length);
  int num_h=(int)(h/grid_length);

  Eigen::Vector3f normal_w;
  normal_w << rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(1).x, rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(1).y, rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(1).z;
  normal_w.normalize();

  Eigen::Vector3f normal_h;
  normal_h << rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(3).x, rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(3).y, rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(3).z;
  normal_h.normalize();

  Eigen::Vector3f first_pt;
  first_pt<<rect_mpc.mypoints.at(0).x,rect_mpc.mypoints.at(0).y,rect_mpc.mypoints.at(0).z;

  for(int i=0;i<num_w;i++){
    for(int j=0;j<num_h;j++){
      Eigen::Vector3f new_pt=first_pt-normal_w*(i+1)*grid_length-normal_h*(j+1)*grid_length;

      MyPt mp={new_pt[0],new_pt[1],new_pt[2]};
      sample_mpt.mypoints.push_back(mp);
    }
  }
}

//if a point is in a cloud
bool isPointInCloud(Point pt, PointCloudPtr cloud){
  for(int i=0;i<cloud->size();i++){
    if(pt.x==cloud->at(i).x&&pt.y==cloud->at(i).y&&pt.z==cloud->at(i).z){
      return true;
    }
  }

  return false;
}

//find nearest neighbor
bool findNearestNeighbor(PointCloudPtr cloud, PointCloudPtr except_cloud, Point search_pt, Point& finded_pt){
  int min=10;
  bool flag=false;

  for(int i=0;i<cloud->size();i++){
    int dis=sqrt(pow(search_pt.x-cloud->at(i).x, 2)+pow(search_pt.y-cloud->at(i).y, 2)+pow(search_pt.z-cloud->at(i).z, 2));
    if(dis<min){
      min=dis;
      finded_pt.x=cloud->at(i).x;
      finded_pt.y=cloud->at(i).y;
      finded_pt.z=cloud->at(i).z;

      if(!isPointInCloud(finded_pt, except_cloud)){
        flag=true;
      }
    }
  }

  return flag;
}

//get intersection points
void get_intr_points(MyPointCloud& source_mpc, MyPointCloud& sample_mpc, float search_r, int* intr_points_num){
  *intr_points_num=0;

  PointCloudPtr source_cloud(new PointCloud);
  MyPointCloud2PointCloud(source_mpc, source_cloud);

  PointCloudPtr sample_cloud(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc, sample_cloud);

  PointCloudPtr intr_cloud(new PointCloud);

  float resolution = 0.005f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

  octree.setInputCloud (source_cloud);
  octree.addPointsFromInputCloud ();

  for(int i=0;i<sample_mpc.mypoints.size();i++){
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = search_r;

    if (octree.radiusSearch(sample_cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      if(pointIdxRadiusSearch.size()>0){
        PointCloudPtr cloud_tem(new PointCloud);

        for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j){
          cloud_tem->push_back(source_cloud->points[ pointIdxRadiusSearch[j]]);
        }

        Point finded_pt;

        if(findNearestNeighbor(cloud_tem, intr_cloud, sample_cloud->at(i), finded_pt)){
          intr_cloud->push_back(finded_pt);
          (*intr_points_num)+=1;
        }
      }
    }
  }
}

//compute Jaccard Index
void computeJaccardIndex(int a_num, int intr_num, float *result){
  *result=intr_num*1.0/(a_num+intr_num);
}

//compute Plane Jaccard Index
void computePlaneJaccardIndex(MyPointCloud& source_mpc, MyPointCloud& rect_mpc, float grid_length, float rate_threshold, float *result){
  MyPointCloud sample_mpc;
  samplePlane(rect_mpc, grid_length, sample_mpc);

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_plane.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_plane.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_plane:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_plane>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  if(rate>rate_threshold){
    computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);
  }
  else{
    *result=0;
  }
}

//Euclidean Cluster Extraction
void big_object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.015); // 1.5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudPtr_RGB cloud_cluster (new PointCloud_RGB);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    }
    cluster_points.push_back(cloud_cluster);

    j++;
  }
}

//Wm5IntrTriangle3Triangle3
bool testIntrTriangle3Triangle3(MyPt p00,MyPt p01,MyPt p02, MyPt p10,MyPt p11,MyPt p12){

  Triangle3<float> triangle1(Vector3<float>(p00.x,p00.y,p00.z),Vector3<float>(p01.x,p01.y,p01.z),Vector3<float>(p02.x,p02.y,p02.z));
  Triangle3<float> triangle2(Vector3<float>(p10.x,p10.y,p10.z),Vector3<float>(p11.x,p11.y,p11.z),Vector3<float>(p12.x,p12.y,p12.z));

  IntrTriangle3Triangle3<float> intTri3Tri3(triangle1, triangle2);
  bool bo=intTri3Tri3.Test();

  return bo;
}

//If a rectangle intersects with the other
bool testIntrRectangle3Rectangle3(MyPt p0_0, MyPt p0_1,MyPt p0_2,MyPt p0_3, MyPt p1_0,MyPt p1_1,MyPt p1_2,MyPt p1_3){
  if(testIntrTriangle3Triangle3(p0_0,p0_1,p0_2, p1_0,p1_1,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_3,p0_2, p1_0,p1_1,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_1,p0_2, p1_0,p1_3,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_3,p0_2, p1_0,p1_3,p1_2)){
    return true;
  }

  return false;
}

//merge plane in objects
void merge_Plane(std::vector<MyPointCloud_RGB>& plane_clouds, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& new_plane_clouds, std::vector<MyPointCloud>& new_rect_clouds){

  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  if(coefficients_vector.size()<2){
    if(coefficients_vector.size()==1){
      PointCloudPtr_RGB cloud_temp(new PointCloud_RGB);
      PointCloud points_temp;

      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());

      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_in_plane);
      std::cout<<"plane_clouds.at(0).mypoints.size:"<< plane_clouds.at(0).mypoints.size() <<std::endl;
      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;

      new_plane_clouds.push_back(plane_clouds.at(0));

      Eigen::Vector3d plane_normal;
      plane_normal << coefficients_vector.at(0).values[0], coefficients_vector.at(0).values[1], coefficients_vector.at(0).values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
      axis.normalize();

      Eigen::Matrix4d matrix;
      getRotationMatrix(axis, angle, matrix);

      Eigen::Matrix4f matrix_transform = matrix.cast<float>();
      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_temp);
      pcl::transformPointCloud (*cloud_temp, *cloud_in_plane, matrix_transform);

      cv::Point2f p0;
      cv::Point2f p1;
      cv::Point2f p2;
      cv::Point2f p3;

      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
      find_min_rect(cloud_in_plane, p0,p1,p2,p3);

      float min_z,max_z,avg_z;
      com_max_and_min_and_avg_z(cloud_in_plane,&min_z,&max_z,&avg_z);

      float cloud_z=min_z;

      if(max_z-avg_z<avg_z-min_z){
        cloud_z=max_z;
      }

      PointCloudPtr points(new PointCloud());
      points->push_back(Point(p0.x,p0.y,cloud_z));
      points->push_back(Point(p1.x,p1.y,cloud_z));
      points->push_back(Point(p2.x,p2.y,cloud_z));
      points->push_back(Point(p3.x,p3.y,cloud_z));

      Eigen::Matrix4d matrix_reverse;
      getRotationMatrix(axis, -angle, matrix_reverse);

      Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
      pcl::copyPointCloud(*points,points_temp);
      pcl::transformPointCloud (points_temp, *points, matrix_transform_reverse);

      MyPointCloud mpc;
      PointCloud2MyPointCloud(points, mpc);

      new_rect_clouds.push_back(mpc);
    }

    return;
  }

  std::vector<Eigen::Vector3d> normals;
  std::vector<pcl::ModelCoefficients::Ptr> coefficientsPtr_vector;
  std::vector<Eigen::Vector3d> new_normals;
  std::vector<pcl::ModelCoefficients::Ptr> new_coefficientsPtr_vector;
  std::vector<MyPointCloud> rect_clouds;

  for(int i=0;i<coefficients_vector.size();i++){
    Eigen::Vector3d plane_normal;
    plane_normal << coefficients_vector.at(i).values[0], coefficients_vector.at(i).values[1], coefficients_vector.at(i).values[2];
    plane_normal.normalize();
    normals.push_back(plane_normal);
  }

  for(int i=0;i<coefficients_vector.size();i++){
    pcl::ModelCoefficients::Ptr mcf(new pcl::ModelCoefficients ());
    for(int j=0;j<coefficients_vector.at(i).values.size();j++){
      mcf->values.push_back(coefficients_vector.at(i).values[j]);
    }
    coefficientsPtr_vector.push_back(mcf);
  }

  //===========For redundant and approximately parallel rectagles that intersects with each other, merge them by least squares.
  int cout = coefficientsPtr_vector.size();
  std::cout<<"&&&&&&&&&&&&&&&&&******cout"<<cout<<std::endl;

  while(cout >0){
    PointCloud_RGB cloud_temp;
    PointCloud points_temp;
    std::vector<MyPointCloud> horizontal_points;
    std::vector<int> horizontal_index;

    int index=0;
    for(int i=0;i<coefficientsPtr_vector.size();i++){
      if(coefficientsPtr_vector.at(i)->values.size()!=0){
        index=i;
        break;
      }
    }

    PointCloudPtr_RGB cloud_projected0(new PointCloud_RGB());
    //pcl::copyPointCloud(plane_clouds.at(index),*cloud_projected0);
    MyPointCloud_RGB2PointCloud(plane_clouds.at(index), cloud_projected0);
    std::cout<<"plane_clouds.at(index).mypoints.size:"<<plane_clouds.at(index).mypoints.size()<<std::endl;
    std::cout<<"cloud_projected0->size:"<<cloud_projected0->size()<<std::endl;
    //showPointCloud (cloud_projected0,"test");

    double angle0=acos(normals[index].dot(Eigen::Vector3d(0,0,1)));
    Eigen::Vector3d axis0=normals[index].cross(Eigen::Vector3d(0,0,1));
    axis0.normalize();

    Eigen::Matrix4d matrix0;
    getRotationMatrix(axis0, angle0, matrix0);

    Eigen::Matrix4f matrix_transform0 = matrix0.cast<float>();
    pcl::copyPointCloud(*cloud_projected0,cloud_temp);
    pcl::transformPointCloud (cloud_temp, *cloud_projected0, matrix_transform0);

    cv::Point2f p0_0;
    cv::Point2f p1_0;
    cv::Point2f p2_0;
    cv::Point2f p3_0;

    find_min_rect(cloud_projected0, p0_0,p1_0,p2_0,p3_0);

    float min_z0,max_z0,avg_z0;
    com_max_and_min_and_avg_z(cloud_projected0,&min_z0,&max_z0,&avg_z0);

    float cloud_z0=min_z0;

    if(max_z0-avg_z0<avg_z0-min_z0){
      cloud_z0=max_z0;
    }

    std::cout<<"cloud_z0:"<<cloud_z0<<std::endl;
    std::cout<<"cloud_projected0->points[0].z:"<<cloud_projected0->points[0].z<<std::endl;

    PointCloudPtr points0(new PointCloud());
    points0->push_back(Point(p0_0.x,p0_0.y,cloud_z0));
    points0->push_back(Point(p1_0.x,p1_0.y,cloud_z0));
    points0->push_back(Point(p2_0.x,p2_0.y,cloud_z0));
    points0->push_back(Point(p3_0.x,p3_0.y,cloud_z0));

    Eigen::Matrix4d matrix0_reverse;
    getRotationMatrix(axis0, -angle0, matrix0_reverse);

    Eigen::Matrix4f matrix_transform0_reverse = matrix0_reverse.cast<float>();
    pcl::copyPointCloud(*points0,points_temp);
    pcl::transformPointCloud (points_temp, *points0, matrix_transform0_reverse);

    MyPointCloud mpc0;
    PointCloud2MyPointCloud(points0, mpc0);
    horizontal_points.push_back(mpc0);
    horizontal_index.push_back(index);

    MyPointCloud_RGB pit_tem;

    for(int i=index+1;i<coefficientsPtr_vector.size();i++){

      if(coefficientsPtr_vector.at(i)->values.size()!=0){

        //horizontal
        if(std::abs(normals[index].dot(normals[i])-1)<0.03){

          horizontal_index.push_back(i);

          PointCloudPtr_RGB cloud_projected_h(new PointCloud_RGB());
          //pcl::copyPointCloud(plane_clouds.at(i),*cloud_projected_h);
          MyPointCloud_RGB2PointCloud(plane_clouds.at(i), cloud_projected_h);

          double angle_h=acos(normals[i].dot(Eigen::Vector3d(0,0,1)));
          Eigen::Vector3d axis_h=normals[i].cross(Eigen::Vector3d(0,0,1));
          axis_h.normalize();

          Eigen::Matrix4d matrix_h;
          getRotationMatrix(axis_h, angle_h, matrix_h);

          Eigen::Matrix4f matrix_transform_h = matrix_h.cast<float>();
          pcl::copyPointCloud(*cloud_projected_h,cloud_temp);
          pcl::transformPointCloud (cloud_temp, *cloud_projected_h, matrix_transform_h);

          cv::Point2f p0_h;
          cv::Point2f p1_h;
          cv::Point2f p2_h;
          cv::Point2f p3_h;

          find_min_rect(cloud_projected_h, p0_h,p1_h,p2_h,p3_h);

          float min_z_h,max_z_h,avg_z_h;
          com_max_and_min_and_avg_z(cloud_projected_h,&min_z_h,&max_z_h,&avg_z_h);

          float cloud_z_h=min_z_h;

          if(max_z_h-avg_z_h<avg_z_h-min_z_h){
            cloud_z_h=max_z_h;
          }

          PointCloudPtr points_h(new PointCloud());
          points_h->push_back(Point(p0_h.x,p0_h.y,cloud_z_h));
          points_h->push_back(Point(p1_h.x,p1_h.y,cloud_z_h));
          points_h->push_back(Point(p2_h.x,p2_h.y,cloud_z_h));
          points_h->push_back(Point(p3_h.x,p3_h.y,cloud_z_h));

          Eigen::Matrix4d matrix_h_reverse;
          getRotationMatrix(axis_h, -angle_h, matrix_h_reverse);

          Eigen::Matrix4f matrix_transform_h_reverse = matrix_h_reverse.cast<float>();
          pcl::copyPointCloud(*points_h,points_temp);
          pcl::transformPointCloud (points_temp, *points_h, matrix_transform_h_reverse);

          MyPt p0_0={points0->points[0].x,points0->points[0].y,points0->points[0].z};
          MyPt p0_1={points0->points[1].x,points0->points[1].y,points0->points[1].z};
          MyPt p0_2={points0->points[2].x,points0->points[2].y,points0->points[2].z};
          MyPt p0_3={points0->points[3].x,points0->points[3].y,points0->points[3].z};
          MyPt p1_0={points_h->points[0].x,points_h->points[0].y,points_h->points[0].z};
          MyPt p1_1={points_h->points[1].x,points_h->points[1].y,points_h->points[1].z};
          MyPt p1_2={points_h->points[2].x,points_h->points[2].y,points_h->points[2].z};
          MyPt p1_3={points_h->points[3].x,points_h->points[3].y,points_h->points[3].z};

          if(testIntrRectangle3Rectangle3(p0_0,p0_1,p0_2,p0_3, p1_0,p1_1,p1_2,p1_3)){
            MyPointCloud mpc_h;
            PointCloud2MyPointCloud(points_h, mpc_h);
            horizontal_points.push_back(mpc_h);

            for(int k=0;k<plane_clouds.at(i).mypoints.size();k++){
              pit_tem.mypoints.push_back(plane_clouds.at(i).mypoints.at(k));
            }
          }
          else{
            new_plane_clouds.push_back(plane_clouds.at(i));
          }
        }
      }
    }

    if(pit_tem.mypoints.size()>0){
      for(int k=0;k<plane_clouds.at(index).mypoints.size();k++){
        pit_tem.mypoints.push_back(plane_clouds.at(index).mypoints.at(k));
      }

      new_plane_clouds.push_back(pit_tem);
    }
    else{
      new_plane_clouds.push_back(plane_clouds.at(index));
    }


    if(horizontal_points.size()>1){
      PointCloudPtr_RGB cloud_all_in_plane(new PointCloud_RGB());

      //cout<<"horizontal_points2.size():"<<horizontal_points2.size()<<endl;
      for(int i=0;i<horizontal_points.size();i++){

        //cout<<"plane_clouds.at(horizontal_index.at(i)).points.size():"<<plane_clouds.at(horizontal_index.at(i)).points.size()<<endl;
        for(int j=0;j<plane_clouds.at(horizontal_index.at(i)).mypoints.size();j++){

          Point_RGB point_tem;
          point_tem.x=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).x;
          point_tem.y=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).y;
          point_tem.z=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).z;
          point_tem.r=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).r;
          point_tem.g=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).g;
          point_tem.b=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).b;

          cloud_all_in_plane->points.push_back(point_tem);
        }
      }

      //showPointClound (cloud_all_in_plane,"cloud_all_in_plane");

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      getPlaneByLeastSquare(cloud_all_in_plane , coefficients);

      //cout<<"coefficients*******************:"<<*coefficients<<endl;

      Eigen::Vector3d new_plane_normal;
      new_plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
      new_plane_normal.normalize();

      double new_angle=acos(new_plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d new_axis=new_plane_normal.cross(Eigen::Vector3d(0,0,1));
      new_axis.normalize();

      Eigen::Matrix4d new_matrix;
      getRotationMatrix(new_axis, new_angle, new_matrix);

      Eigen::Matrix4f new_matrix_transform = new_matrix.cast<float>();
      pcl::copyPointCloud(*cloud_all_in_plane,cloud_temp);
      pcl::transformPointCloud (cloud_temp, *cloud_all_in_plane, new_matrix_transform);

      cv::Point2f new_p0;
      cv::Point2f new_p1;
      cv::Point2f new_p2;
      cv::Point2f new_p3;

      find_min_rect(cloud_all_in_plane, new_p0,new_p1,new_p2,new_p3);

      float min_z_new,max_z_new,avg_z_new;
      com_max_and_min_and_avg_z(cloud_all_in_plane,&min_z_new,&max_z_new,&avg_z_new);

      float cloud_z_new=min_z_new;

      if(max_z_new-avg_z_new<avg_z_new-min_z_new){
        cloud_z_new=max_z_new;
      }

      PointCloudPtr new_points(new PointCloud());
      new_points->push_back(Point(new_p0.x,new_p0.y,cloud_z_new));
      new_points->push_back(Point(new_p1.x,new_p1.y,cloud_z_new));
      new_points->push_back(Point(new_p2.x,new_p2.y,cloud_z_new));
      new_points->push_back(Point(new_p3.x,new_p3.y,cloud_z_new));

      Eigen::Matrix4d new_matrix_reverse;
      getRotationMatrix(new_axis, -new_angle, new_matrix_reverse);

      Eigen::Matrix4f new_matrix_transform_reverse = new_matrix_reverse.cast<float>();
      pcl::copyPointCloud(*new_points,points_temp);
      pcl::transformPointCloud (points_temp, *new_points, new_matrix_transform_reverse);

      MyPointCloud new_mpc;
      PointCloud2MyPointCloud(new_points, new_mpc);
      rect_clouds.push_back(new_mpc);

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficients->values.size();i++){
        mc->values.push_back(coefficients->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      for(int i=0;i<horizontal_points.size();i++){
        //coefficientsPtr_vector.pop_back(horizontal_index.at(i));
        coefficientsPtr_vector.at(horizontal_index.at(i))->values.clear();
        cout--;
      }

    }
    else{
      rect_clouds.push_back(horizontal_points.at(0));

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficientsPtr_vector.at(index)->values.size();i++){
        mc->values.push_back(coefficientsPtr_vector.at(index)->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      coefficientsPtr_vector.at(index)->values.clear();
      cout--;
    }
  }

  coefficients_vector.clear();
  for(int k=0;k<new_coefficientsPtr_vector.size();k++){
    coefficients_vector.push_back(*(new_coefficientsPtr_vector.at(k)));
  }

  for(int i=0;i<rect_clouds.size();i++){
    new_rect_clouds.push_back(rect_clouds.at(i));
  }
}

//big plane fitting
void big_plane_fitting(PointCloudPtr_RGB sourceCloud, MyPointCloud_RGB &plane_cloud, MyPointCloud &rect_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr_RGB remained_cloud){
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*sourceCloud,*cloud_tem);

  PointCloudPtr_RGB cloud_p (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<Point_RGB> extract;

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_tem);
  seg.segment (*inliers, *plane_coefficients);
  if (inliers->indices.size () < 500)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }

  // Extract the inliers
  extract.setInputCloud (cloud_tem);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  PointCloudPtr rect_cl(new PointCloud);
  getRectForPlaneCloud(cloud_p, plane_coefficients, rect_cl);
  PointCloud2MyPointCloud_RGB(cloud_p, plane_cloud);
  PointCloud2MyPointCloud(rect_cl, rect_cloud);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_tem.swap (cloud_f);

  pcl::copyPointCloud(*cloud_tem,*remained_cloud);
}

//detct floor
void detect_floor_and_walls(PointCloudPtr_RGB cloud, MyPointCloud_RGB& floor_cloud, pcl::ModelCoefficients& floor_coefficients, MyPointCloud& floor_rect_cloud, vector<MyPointCloud_RGB> &wall_clouds, std::vector<MyPointCloud> &wall_rect_clouds, PointCloudPtr_RGB remained_cloud){

  std::vector<pcl::ModelCoefficients> plane_coefficients_vector;
  vector<MyPointCloud_RGB> plane_clouds_tem;

  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *cloud_tem);

  PointCloudPtr_RGB cloud_remaining(new PointCloud_RGB);

  bool is_floor_detected=false;

  //// Create the filtering object
  //pcl::VoxelGrid<Point_RGB> sor;
  //sor.setInputCloud (cloud);
  //sor.setLeafSize (0.02f, 0.02f, 0.02f);
  //sor.filter (*cloud_tem);

  //float times=pow((cloud->size()*0.1)/cloud_filtered->size(),1.0/3);

  while(1){
    PointCloudPtr_RGB remained_tem(new PointCloud_RGB);
    MyPointCloud_RGB plane_cloud;
    MyPointCloud plane_cloud_n;
    MyPointCloud rect_cloud;

    float result=0;

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    big_plane_fitting(cloud_tem, plane_cloud, rect_cloud, plane_coefficients, remained_tem);

    MyPointCloud_RGB2MyPointCloud(plane_cloud, plane_cloud_n);

    /* PointCloudPtr_RGB test_cloud(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud(plane_cloud, test_cloud);
    showPointCloud (test_cloud,"test_cloud");*/

    if(plane_cloud.mypoints.size()==0){
      printf("Can't fit any more\n");
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    float l=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(1).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(1).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(1).z, 2));
    float w=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(3).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(3).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(3).z, 2));

    cout<<"l=====================:"<<l<<endl;
    cout<<"w=====================:"<<w<<endl;

    if(l<1.5&&w<1.5){
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, 0.025, &result);

    cout<<"result=====================:"<<result<<endl;

    float thresdhold=0.12;

    if(result>thresdhold){
      Eigen::Vector3d plane_normal;
      plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
      plane_normal.normalize();

      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<<endl;
      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(0).z):"<<std::abs(rect_cloud.mypoints.at(0).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(1).z):"<<std::abs(rect_cloud.mypoints.at(1).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(2).z):"<<std::abs(rect_cloud.mypoints.at(2).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(3).z):"<<std::abs(rect_cloud.mypoints.at(3).z)<<endl;

      if(std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<0.045&&std::abs(rect_cloud.mypoints.at(0).z)<0.6&&std::abs(rect_cloud.mypoints.at(1).z)<0.6&&std::abs(rect_cloud.mypoints.at(2).z)<0.6&&std::abs(rect_cloud.mypoints.at(3).z)<0.6){
        pcl::copyPointCloud(*remained_tem, *cloud_tem);

        //it is floor
        if(!is_floor_detected){
          CopyMyPointCloud(rect_cloud, floor_rect_cloud);
          CopyMyPointCloud_RGB(plane_cloud, floor_cloud);

          floor_coefficients=*plane_coefficients;

          is_floor_detected=true;

          /*PointCloudPtr_RGB test_cloud(new PointCloud_RGB);
          MyPointCloud_RGB2PointCloud(plane_cloud, test_cloud);
          showPointCloud (test_cloud,"test_cloud_floor");*/
        }
        else{
          PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
          MyPointCloud_RGB2PointCloud(plane_cloud, plane_cloud_tem);
          appendCloud_RGB(plane_cloud_tem,cloud_remaining);
        }
      }
      else if(std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<0.25){
        //it is a wall
        pcl::copyPointCloud(*remained_tem, *cloud_tem);
        plane_clouds_tem.push_back(plane_cloud);
        plane_coefficients_vector.push_back(*plane_coefficients);

        /* PointCloudPtr_RGB test_cloud(new PointCloud_RGB);
        MyPointCloud_RGB2PointCloud(plane_cloud, test_cloud);
        showPointCloud (test_cloud,"test_cloud_wall");*/
      }
      else{
        pcl::copyPointCloud(*remained_tem, *cloud_tem);
        PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
        MyPointCloud_RGB2PointCloud(plane_cloud, plane_cloud_tem);
        appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      }
    }
    else{
      appendCloud_RGB(cloud_tem,cloud_remaining);
      printf("Can't fit any more\n");
      break;
    }
  }

  if(plane_clouds_tem.size()>0){
    merge_Plane(plane_clouds_tem, plane_coefficients_vector, wall_clouds, wall_rect_clouds);
  }

  pcl::copyPointCloud(*cloud_remaining, *remained_cloud);
}

//detct support plane
void detect_support_plane(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &support_clouds, std::vector<MyPointCloud> &support_rect_clouds, PointCloudPtr_RGB remained_cloud){
  std::vector<pcl::ModelCoefficients> plane_coefficients_vector;
  vector<MyPointCloud_RGB> plane_clouds_tem;

  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *cloud_tem);

  PointCloudPtr_RGB cloud_remaining(new PointCloud_RGB);

  bool is_floor_detected=false;

  while(1){
    PointCloudPtr_RGB remained_tem(new PointCloud_RGB);
    MyPointCloud_RGB plane_cloud;
    MyPointCloud plane_cloud_n;
    MyPointCloud rect_cloud;

    float result=0;

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    big_plane_fitting(cloud_tem, plane_cloud, rect_cloud, plane_coefficients, remained_tem);

    /*PointCloudPtr_RGB test_cloud(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud(plane_cloud, test_cloud);
    showPointCloud (test_cloud,"test_cloud");*/

    MyPointCloud_RGB2MyPointCloud(plane_cloud, plane_cloud_n);

    if(plane_cloud.mypoints.size()==0){
      printf("Can't fit any more\n");
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    float l=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(1).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(1).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(1).z, 2));
    float w=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(3).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(3).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(3).z, 2));

    cout<<"l=====================:"<<l<<endl;
    cout<<"w=====================:"<<w<<endl;
    cout<<"l*w:"<<l*w<<endl;

    if(l*w<0.6){
      cout<<"===========***==========="<<endl;
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, 0.025, &result);

    cout<<"result=====================:"<<result<<endl;

    float thresdhold=0.12;

    if(result>thresdhold){
      Eigen::Vector3d plane_normal;
      plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));

      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<<endl;
      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(0).z):"<<std::abs(rect_cloud.mypoints.at(0).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(1).z):"<<std::abs(rect_cloud.mypoints.at(1).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(2).z):"<<std::abs(rect_cloud.mypoints.at(2).z)<<endl;
      cout<<"&std::abs(rect_cloud.mypoints.at(3).z):"<<std::abs(rect_cloud.mypoints.at(3).z)<<endl;

      if(angle<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<0.03
        &&std::abs(rect_cloud.mypoints.at(0).z)<1&&std::abs(rect_cloud.mypoints.at(1).z)<1
        &&std::abs(rect_cloud.mypoints.at(2).z)<1&&std::abs(rect_cloud.mypoints.at(3).z)<1
        &&std::abs(rect_cloud.mypoints.at(0).z)>=0&&std::abs(rect_cloud.mypoints.at(1).z)>=0
        &&std::abs(rect_cloud.mypoints.at(2).z)>=0&&std::abs(rect_cloud.mypoints.at(3).z)>=0){
          pcl::copyPointCloud(*remained_tem, *cloud_tem);

          //it is a support plane
          plane_clouds_tem.push_back(plane_cloud);
          plane_coefficients_vector.push_back(*plane_coefficients);
      }
      else{
        pcl::copyPointCloud(*remained_tem, *cloud_tem);
        PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
        MyPointCloud_RGB2PointCloud(plane_cloud, plane_cloud_tem);
        appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      }
    }
    else{
      appendCloud_RGB(cloud_tem,cloud_remaining);
      printf("Can't fit any more\n");
      break;
    }
  }

  if(plane_clouds_tem.size()>0){
    merge_Plane(plane_clouds_tem, plane_coefficients_vector, support_clouds, support_rect_clouds);
  }

  pcl::copyPointCloud(*cloud_remaining, *remained_cloud);
}


//mark remaining cloud by bounding box
void mark_remaining_cloud(PointCloudPtr_RGB sourceCloud, PointCloudPtr cloud){
  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(sourceCloud, p0,p1,p2,p3);

  float min_x,min_y,min_z, max_x, max_y, max_z;

  com_bounding_box(sourceCloud, &min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  cloud->push_back(Point(p0.x,p0.y,min_z));
  cloud->push_back(Point(p1.x,p1.y,min_z));
  cloud->push_back(Point(p2.x,p2.y,min_z));
  cloud->push_back(Point(p3.x,p3.y,min_z));

  cloud->push_back(Point(p0.x,p0.y,max_z));
  cloud->push_back(Point(p1.x,p1.y,max_z));
  cloud->push_back(Point(p2.x,p2.y,max_z));
  cloud->push_back(Point(p3.x,p3.y,max_z));
}

void getColorByValue(float val, float min, float max, float *r, float *g, float *b)
{
  int tmp = static_cast<int>((val - min) / (max - min) * 255);
  *r = g_color_table[tmp][0] * 255;
  *g = g_color_table[tmp][1] * 255;
  *b = g_color_table[tmp][2] * 255;
}


//compute gaussian curvature
void compute_gaussian_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored){
  printf("0000000000000000000\n");
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.02);

  normalEstimation.compute (*cloudWithNormals);
  printf("111111111111111111\n");
  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud (cloud);

  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(cloudWithNormals);
  printf("2222222222222222222\n");
  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);
  principalCurvaturesEstimation.setRadiusSearch(0.05);

  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principalCurvaturesEstimation.compute (*pcs);
  printf("3333333333333333333\n");
  float min_curv_val = 10000;
  float max_curv_val = -10000;

  for (unsigned int i = 0; i < pcs->points.size(); i++) {
    /* if (pcs.points[i].pc1 > max_curv_val){
    max_curv_val = pcs.points[i].pc1;
    }

    if (pcs.points[i].pc2 < min_curv_val){
    min_curv_val = pcs.points[i].pc2;
    }*/
    float curv_val=pcs->points[i].pc1*pcs->points[i].pc2;
    if (curv_val > max_curv_val){
      max_curv_val = curv_val;
    }
    if (curv_val < min_curv_val){
      min_curv_val = curv_val;
    }
  }

  printf("444444444444444444444\n");

  cout<<"pcs->points.size():"<<pcs->points.size()<<endl;
  cout<<"cloud->size():"<<cloud->size()<<endl;

  for (unsigned int i = 0; i < pcs->points.size(); i++) {
    float r=0;
    float g=0;
    float b=0;

    Point_Cur_RGB cur;
    /* if(cur.curvature>0.01){
    cout<<"cur.curvature:"<<cur.curvature<<endl;
    }*/

    //gaussian curvature
    /*color = (((pcs.points[i].pc2 - min_curv_val) * 255.0
    / raw_range) * ((pcs.points[i].pc1 - min_curv_val)
    / raw_range));*/

    //cout<<"i:"<<i<<endl;
    /*cout<<"i:"<<i<<endl;
    cout<<"cur.curvature:"<<cur.curvature<<endl;
    cout<<"pcs->points[i].pc1:"<<pcs->points[i].pc1<<endl;
    cout<<"pcs->points[i].pc2:"<<pcs->points[i].pc2<<endl;*/

    if(!(_isnan(pcs->points[i].pc1)||_isnan(pcs->points[i].pc2))){
      cur.curvature=pcs->points[i].pc1*pcs->points[i].pc2;
      //cout<<"cur.curvature:"<<cur.curvature<<endl;
      getColorByValue(cur.curvature, min_curv_val, max_curv_val, &r, &g, &b);
    }

    else{
      cur.curvature=0;
      //cout<<"============================"<<endl;
    }

    //cout<<"====================="<<endl;

    /*if(cur.curvature>0.01){
    cout<<"color:"<<color<<endl;
    }*/

    cur.x=cloud->at(i).x;
    cur.y=cloud->at(i).y;
    cur.z=cloud->at(i).z;
    cur.r=r;
    cur.g=g;
    cur.b=b;

    curvatures.push_back(cur);

    Point_RGB p;
    p.x=cloud->at(i).x;
    p.y=cloud->at(i).y;
    p.z=cloud->at(i).z;
    p.r=r;
    p.g=g;
    p.b=b;

    cloud_colored->push_back(p);
  }

  printf("555555555555555555555555\n");
}

//compute gaussian curvature
void compute_mean_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored){
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.02);

  normalEstimation.compute (*cloudWithNormals);

  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud (cloud);

  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(cloudWithNormals);

  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);
  principalCurvaturesEstimation.setRadiusSearch(0.05);

  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principalCurvaturesEstimation.compute (*pcs);

  float min_curv_val = 10000;
  float max_curv_val = -10000;

  for (unsigned int i = 0; i < pcs->points.size(); i++) {
    float curv_val=(pcs->points[i].pc1+pcs->points[i].pc2)/2;
    if(!(_isnan(curv_val))){
      if (curv_val > max_curv_val){
        max_curv_val = curv_val;
      }
      if (curv_val < min_curv_val){
        min_curv_val = curv_val;
      }
    }
  }

  for (unsigned int i = 0; i < pcs->points.size(); i++) {
    float r=0;
    float g=0;
    float b=0;

    Point_Cur_RGB cur;

    if(!(_isnan(pcs->points[i].pc1)||_isnan(pcs->points[i].pc2))){
      cur.curvature=(pcs->points[i].pc1+pcs->points[i].pc2)/2;
      getColorByValue(cur.curvature, min_curv_val, max_curv_val, &r, &g, &b);
    }
    else{
      cur.curvature=0;
    }

    cur.x=cloud->at(i).x;
    cur.y=cloud->at(i).y;
    cur.z=cloud->at(i).z;
    cur.r=r;
    cur.g=g;
    cur.b=b;

    curvatures.push_back(cur);

    Point_RGB p;
    p.x=cloud->at(i).x;
    p.y=cloud->at(i).y;
    p.z=cloud->at(i).z;
    p.r=r;
    p.g=g;
    p.b=b;

    cloud_colored->push_back(p);
  }
}

//projecte curvature to x_y plane
void curvature_projected(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& projected_curvatures){
  cloud_colored->clear();
  projected_curvatures.clear();

  PointCloudPtr_RGB projected_cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *projected_cloud_tem);

  for (unsigned int i = 0; i < projected_cloud_tem->size(); i++) {
    projected_cloud_tem->at(i).z=0;
  }

  pcl::KdTreeFLANN<Point_RGB> kdtree;
  kdtree.setInputCloud (projected_cloud_tem);

  float min_curv_val = 10000;
  float max_curv_val = -10000;

  vector<bool> flags(projected_cloud_tem->size());
  for(unsigned int i = 0; i < flags.size(); i++){
    flags[i]=true;
  }

  for (unsigned int i = 0; i < projected_cloud_tem->size(); i++) {
    float curvature=0;

    Point_RGB searchPoint=projected_cloud_tem->at(i);

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 0.00000001;

    if(flags.at(i)){
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      if(pointIdxRadiusSearch.size ()>1){
        cout<<"pointIdxRadiusSearch.size ():"<<pointIdxRadiusSearch.size ()<<endl;
      }
      

      for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j){
        curvature+=curvatures.at(pointIdxRadiusSearch[j]).curvature;
        flags.at(pointIdxRadiusSearch[j])=false;
      }

      if (curvature > max_curv_val){
        max_curv_val = curvature;
      }
      if (curvature < min_curv_val){
        min_curv_val = curvature;
      }

      Point_Cur_RGB c;
      c.x=cloud->at(i).x;
      c.y=cloud->at(i).y;
      c.z=0;
      c.curvature=curvature;
      projected_curvatures.push_back(c);
    }
    }
  }

  for (unsigned int i = 0; i < projected_curvatures.size(); i++) {
    float r=0;
    float g=0;
    float b=0;
    getColorByValue(projected_curvatures.at(i).curvature, min_curv_val, max_curv_val, &r, &g, &b);

    projected_curvatures.at(i).r=r;
    projected_curvatures.at(i).g=g;
    projected_curvatures.at(i).b=b;

    Point_RGB p;
    p.x=projected_curvatures.at(i).x;
    p.y=projected_curvatures.at(i).y;
    p.z=projected_curvatures.at(i).z;
    p.r=r;
    p.g=g;
    p.b=b;

    cloud_colored->push_back(p);
  }
}

//void GaussianBlur();
//cv::GaussianBlur( src, dst, Size( i, i ), 0, 0 )
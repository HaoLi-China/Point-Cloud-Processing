//#pragma once
//#include <iostream>
//#include <string>
//
////#include "common_type.h"
//#include "new_icp.h"
//
//int main(int argc, char* argv[])
//{
//  // The point clouds we will be using
//  PointCloudPtr_RGB cloud0 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud1 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud2 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud3 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud4 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud5 (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud_out (new PointCloud_RGB);
//
//  //loadPointCloud_ply("data/room_tf/scan1_out.ply", cloud1);
//  //loadPointCloud_ply("data/room_tf/scan2_out.ply", cloud2);
//  loadPointCloud_ply("data/frame0_out.ply", cloud0);
//  loadPointCloud_ply("data/frame1_out.ply", cloud1);
//  loadPointCloud_ply("data/frame2_out.ply", cloud2);
//
//
//
//  //vector<MyPointCloud_RGB> clouds0;
//  //MyPointCloud_RGB mpc0;
//  //PointCloud_RGB2MyPointCloud_RGB(cloud0, mpc0);
//  //clouds0.push_back(mpc0);
//  //PointCloud_RGB2MyPointCloud_RGB(cloud1, mpc0);
//  //clouds0.push_back(mpc0);
//  //PointCloud_RGB2MyPointCloud_RGB(cloud2, mpc0);
//  //clouds0.push_back(mpc0);
//  ///* PointCloud_RGB2MyPointCloud_RGB(cloud3, mpc);
//  //clouds.push_back(mpc);
//  //PointCloud_RGB2MyPointCloud_RGB(cloud4, mpc);
//  //clouds.push_back(mpc);*/
//
//  //meargePointClouds(clouds0, cloud_out, 100, 1000);
//
//  ////do_icp(cloud_in0, cloud_in1, cloud_out, 100, 1000);
//
//  //pcl::io::savePLYFileASCII("data/cloud_out1.ply", *cloud_out);
//
//  //showPointCloud(cloud_out, "cloud_out");
//
//  //return 0;
//
//
//
//  /*Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
//
//  PointCloudPtr_RGB cloud_sum (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
//  pcl::transformPointCloud(*cloud1, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud1);
//
//  pcl::io::savePLYFileASCII("data/room_tf/cloud1.ply", *cloud1);
//
//  mat(1,3)-=1.7;
//  pcl::transformPointCloud(*cloud2, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud2);
//
//  pcl::io::savePLYFileASCII("data/room_tf/cloud2.ply", *cloud2);*/
//
//  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
//
//  PointCloudPtr_RGB cloud_sum (new PointCloud_RGB);
//  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
//  pcl::transformPointCloud(*cloud0, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud0);
//
//  //pcl::io::savePLYFileASCII("data/cloud1.ply", *cloud1);
//
//  mat(1,3)-=0.4;
//  pcl::transformPointCloud(*cloud1, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud1);
//
//  //pcl::io::savePLYFileASCII("data/room_tf/cloud2.ply", *cloud2);
//
//  mat(1,3)-=0.4;
//  pcl::transformPointCloud(*cloud2, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud2);
//  
//
// /* mat(1,3)-=0.4;
//  pcl::transformPointCloud(*cloud2, *cloud_tem, mat);
//  pcl::copyPointCloud(*cloud_tem, *cloud2);
// pcl::io::savePLYFileASCII("cloud2.ply", *cloud2);*/
//
//
//  appendCloud_RGB(cloud0, cloud_sum);
//  appendCloud_RGB(cloud1, cloud_sum);
//  appendCloud_RGB(cloud2, cloud_sum);
//  //appendCloud_RGB(cloud3, cloud_sum);
//
//  showPointCloud(cloud_sum, "cloud_sum");
//
//  pcl::io::savePLYFileASCII("data/cloud_sum.ply", *cloud_sum);
//
// /* loadPointCloud_ply("data/frame3.ply", cloud3);
//  loadPointCloud_ply("data/frame4.ply", cloud4);*/
//
//  vector<MyPointCloud_RGB> clouds;
//  MyPointCloud_RGB mpc;
//  PointCloud_RGB2MyPointCloud_RGB(cloud0, mpc);
//  clouds.push_back(mpc);
//  PointCloud_RGB2MyPointCloud_RGB(cloud1, mpc);
//  clouds.push_back(mpc);
//  PointCloud_RGB2MyPointCloud_RGB(cloud2, mpc);
//  clouds.push_back(mpc);
//  /* PointCloud_RGB2MyPointCloud_RGB(cloud3, mpc);
//  clouds.push_back(mpc);
//  PointCloud_RGB2MyPointCloud_RGB(cloud4, mpc);
//  clouds.push_back(mpc);*/
//
//  meargePointClouds(clouds, cloud_out, 10, 1000);
//
//  //do_icp(cloud_in0, cloud_in1, cloud_out, 100, 1000);
//
//  pcl::io::savePLYFileASCII("data/cloud_out.ply", *cloud_out);
//
//  showPointCloud(cloud_out, "cloud_out");
//
//
//
//  return (0);
//}
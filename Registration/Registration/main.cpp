#pragma once
#include <iostream>
#include <string>

//#include "common_type.h"
#include "new_icp.h"

int main(int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudPtr_RGB_NORMAL cloud0 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud1 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud2 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud3 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud4 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud5 (new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL cloud_out (new PointCloud_RGB_NORMAL);

  /*loadPointCloud_normal_ply("data/rotation/in_out/cloud_in0.ply", cloud0);
  loadPointCloud_normal_ply("data/rotation/in_out/cloud_in1.ply", cloud1);*/
  loadPointCloud_normal_ply("data/rotation/r_10_0.ply", cloud0);
  loadPointCloud_normal_ply("data/rotation/r_10_1.ply", cloud1);
  loadPointCloud_normal_ply("data/rotation/r_10_2.ply", cloud2);
  loadPointCloud_normal_ply("data/rotation/r_10_3.ply", cloud3);
  loadPointCloud_normal_ply("data/rotation/r_10_4.ply", cloud4);
  loadPointCloud_normal_ply("data/rotation/r_10_5.ply", cloud5);

  /*loadPointCloud_normal_ply("data/rotation/r_10_0_tf.ply", cloud0);
  loadPointCloud_normal_ply("data/rotation/r_10_1_tf.ply", cloud1);
  loadPointCloud_normal_ply("data/rotation/r_10_2_tf.ply", cloud2);
  loadPointCloud_normal_ply("data/rotation/r_10_3_tf.ply", cloud3);
  loadPointCloud_normal_ply("data/rotation/r_10_4_tf.ply", cloud4);
  loadPointCloud_normal_ply("data/rotation/r_10_5_tf.ply", cloud5);*/

  //Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

  //PointCloudPtr_RGB_NORMAL cloud_sum (new PointCloud_RGB_NORMAL);
  //PointCloudPtr_RGB_NORMAL cloud_tem (new PointCloud_RGB_NORMAL);
  //pcl::transformPointCloud(*cloud0, *cloud_tem, mat);
  //pcl::copyPointCloud(*cloud_tem, *cloud0);

  //mat(1,3)-=0.4;
  //pcl::transformPointCloud(*cloud1, *cloud_tem, mat);
  //pcl::copyPointCloud(*cloud_tem, *cloud1);

  ////showPointCloud_RGB_NORMAL (cloud0, "cloud0");
  ////showPointCloud_RGB_NORMAL (cloud1, "cloud1");

  //mat(1,3)-=0.4;
  //pcl::transformPointCloud(*cloud2, *cloud_tem, mat);
  //pcl::copyPointCloud(*cloud_tem, *cloud2);
  
  vector<MyPointCloud_RGB_NORMAL> clouds;
  MyPointCloud_RGB_NORMAL mpc;
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud0, mpc);
  clouds.push_back(mpc);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud1, mpc);
  clouds.push_back(mpc);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud2, mpc);
  clouds.push_back(mpc);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud3, mpc);
  clouds.push_back(mpc);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud4, mpc);
  clouds.push_back(mpc);
  PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud5, mpc);
  clouds.push_back(mpc);

  meargePointCloudsWithNormal(clouds, cloud_out, 10, 1000);

  //appendCloud_RGB_NORMAL(cloud0, cloud_out);
  //appendCloud_RGB_NORMAL(cloud1, cloud_out);
  //appendCloud_RGB_NORMAL(cloud2, cloud_out);

  //pcl::io::savePLYFileASCII("data/rotation/in_out/cloud_out.ply", *cloud_out);
  pcl::io::savePLYFileASCII("data/rotation/in_out/test1111.ply", *cloud_out);

  showPointCloud_RGB_NORMAL (cloud_out, "cloud_out");

  return (0);
}
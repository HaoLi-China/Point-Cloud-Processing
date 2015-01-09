#include "visualizer.h"
#include "color_op.h"
#include "file_io.h"
#include "scene_seg.h"

int main (int argc, char *argv[])
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
  //loadPointCloud_normal_ply("data/big_table_normal.ply", cloud);
  loadPointCloud_normal_ply("data/table0.ply", cloud);

  showPointCloud_RGB_NORMAL(cloud, "cloud");

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;
  //detect_table_plane(cloud, planeCloud, tabletopCloud);

  detect_table(cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  showPointCloud_RGB_NORMAL(planeCloud, "planeCloud");

  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<planeCloud->size();i++){
    Point_RGB pr;
    pr.x=planeCloud->at(i).x;
    pr.y=planeCloud->at(i).y;
    pr.z=planeCloud->at(i).z;
    pr.r=planeCloud->at(i).r;
    pr.g=planeCloud->at(i).g;
    pr.b=planeCloud->at(i).b;
    pc->push_back(pr);
  }

  vs.viewer->addPointCloud (pc, "table_cloud");

  
  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  printf("000000000\n");

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);
  printf("111111111\n");
  showPointCloud_RGB_NORMAL(remainingCloud, "remainingCloud");

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  showPointCloud_RGB_NORMAL(tabletopCloud, "tabletopCloud");


  //showPointClound(planeCloud,"planeCloud");

  //pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  //pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  //detect_table(cloud, coefficients_plane, inliers_plane);

  //
  //PointCloudPtr_RGB_NORMAL table_cloud(new PointCloud_RGB_NORMAL());

  //pcl::ExtractIndices<Point_RGB_NORMAL> extract0;// Create the filtering object
  //// Extract the inliers
  //extract0.setInputCloud (cloud);
  //extract0.setIndices (inliers_plane);
  //extract0.setNegative (false);
  //extract0.filter (*table_cloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.06f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<PointCloudPtr_RGB_NORMAL> cluster_points;

  vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
  vector<Normal> vecPatcNormal;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i)->size()<200){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);
    VCCS_over_segmentation(cluster_points.at(i),voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    std::stringstream str;
    str<<"colored_voxel_cloud"<<i;
    std::string id_pc=str.str();
    vs.viewer->addPointCloud (colored_cloud, id_pc);

    if(i==2){
      for(int j=0;j<1;j++){
        double sum_x=0;
        double sum_y=0;
        double sum_z=0;

        MyPointCloud_RGB_NORMAL mpc=patch_clouds.at(j);

        for(int k = 0; k < mpc.mypoints.size(); k++){
          sum_x += mpc.mypoints.at(k).x;
          sum_y += mpc.mypoints.at(k).y;
          sum_z += mpc.mypoints.at(k).z;
        }

        float new_x = sum_x / mpc.mypoints.size();
        float new_y = sum_y / mpc.mypoints.size();
        float new_z = sum_z / mpc.mypoints.size();

        std::stringstream str;
        str<<"point"<<i<<j;
        std::string id=str.str();

        vs.viewer->addSphere(Point(new_x, new_y, new_z), 0.01, id);

        cout<<"point:"<<new_x<<" "<<new_y<<" "<<new_z<<" "<<endl;
        cout<<"normal:"<<normal_cloud->at(j).normal_x<<" "<<normal_cloud->at(j).normal_y<<" "<<normal_cloud->at(j).normal_z<<" "<<endl;

      }
    }
  }

  vs.show();

  return 0;
}

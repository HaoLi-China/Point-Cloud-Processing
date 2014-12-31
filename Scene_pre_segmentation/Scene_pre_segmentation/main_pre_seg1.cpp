#include "visualizer.h"
#include "file_io.h"
#include "utility.h"
#include "bigScene_preSeg.h"

int main (int argc, char *argv[])
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB cloud(new PointCloud_RGB);
  NormalCloudTPtr normals(new NormalCloudT);

  //loadPointCloud_ply("data/scene0.ply", cloud);
  //loadPointCloud_ply("data/scene1.ply", cloud);
  //loadPointCloud_ply("data/big_table_after.ply", cloud);
  loadPointCloud_ply("data/two_tables.ply", cloud);
  //loadPointCloud_ply("data/hui_room.ply", cloud);

  /******************detect floor and wall************************/
  MyPointCloud_RGB floor_cloud;
  pcl::ModelCoefficients floor_coefficients;
  MyPointCloud floor_rect_cloud;
  vector<MyPointCloud_RGB> wall_clouds;
  std::vector<MyPointCloud> wall_rect_clouds;
  PointCloudPtr_RGB remained_cloud(new PointCloud_RGB);

  detect_floor_and_walls(cloud, floor_cloud, floor_coefficients, floor_rect_cloud, wall_clouds, wall_rect_clouds, remained_cloud);

  if(floor_cloud.mypoints.size()>0){
    Eigen::Matrix4f matrix_transform;
    Eigen::Matrix4f matrix_translation_r;
    Eigen::Matrix4f matrix_transform_r;
    getTemTransformMatrix(floor_coefficients, floor_rect_cloud, matrix_transform, matrix_translation_r, matrix_transform_r);

    PointCloudPtr_RGB filter_remained_cloud(new PointCloud_RGB);
    remove_outliers(remained_cloud, floor_rect_cloud, wall_rect_clouds, matrix_transform, matrix_translation_r, matrix_transform_r, filter_remained_cloud, vs);

    //vs.viewer->addPointCloud(filter_remained_cloud,"filter_remained_cloud");

    PointCloudPtr_RGB new_remained_cloud(new PointCloud_RGB);
    PointCloud_RGB ct;
    pcl::copyPointCloud(*filter_remained_cloud,ct);
    pcl::transformPointCloud (ct, *new_remained_cloud, matrix_transform);

    /******************pre-segment scene************************/
    std::vector<MyPointCloud> sum_support_clouds;
    std::vector<MyPointCloud> sum_separation_rect_clouds;
    pre_segment_scene(new_remained_cloud, sum_support_clouds, sum_separation_rect_clouds);


    /******************segment scene************************/
    vector<MyPointCloud> clustering_cloud;
    segment_scene(new_remained_cloud, sum_support_clouds, sum_separation_rect_clouds, clustering_cloud, vs);

    for(int i = 0; i < clustering_cloud.size(); i++){
      PointCloudPtr_RGB clustering_color_cloud(new PointCloud_RGB);
      MyPointCloud mpc=clustering_cloud.at(i);

      for(int j = 0;j < mpc.mypoints.size(); j++){
        Point_RGB pt;
        pt.x=mpc.mypoints.at(j).x;
        pt.y=mpc.mypoints.at(j).y;
        pt.z=mpc.mypoints.at(j).z;
        pt.r=new_color_table[i%30][0];
        pt.g=new_color_table[i%30][1];
        pt.b=new_color_table[i%30][2];
        clustering_color_cloud->push_back(pt);
      }

      std::stringstream st;
      st<<"clustering_color_cloud"<<i;
      std::string id_str=st.str();
      vs.viewer->addPointCloud(clustering_color_cloud,id_str);
    }

    vs.show();
  }

  return 0;
}

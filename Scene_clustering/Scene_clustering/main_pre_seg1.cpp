#include "visualizer.h"
#include "file_io.h"
#include "utility.h"
#include "bigScene_preSeg.h"
#include "common_func.h"

int main (int argc, char *argv[])
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB cloud(new PointCloud_RGB);

  //loadPointCloud_ply("data/scene0.ply", cloud);
  //loadPointCloud_ply("data/scene1.ply", cloud);
  //loadPointCloud_ply("data/big_table_after.ply", cloud);
  loadPointCloud_ply("data/two_tables.ply", cloud);
  //loadPointCloud_ply("data/big_room.ply", cloud);

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

    /******************pre-segment scene************************/
    vector<MyPointCloud_RGB> cluster_projected_pcs;
    vector<MyPointCloud_RGB> cluster_origin_pcs;
    PointCloudPtr_RGB colored_projected_pc(new PointCloud_RGB);
    PointCloudPtr_RGB colored_origin_pc(new PointCloud_RGB);
    pre_segment_scene(filter_remained_cloud, matrix_transform, matrix_translation_r, matrix_transform_r, cluster_projected_pcs, cluster_origin_pcs, colored_projected_pc, colored_origin_pc);

    //vs.viewer->addPointCloud(filter_remained_cloud, "c1");
    //vs.viewer->addPointCloud(colored_projected_pc, "c1");
    //vs.viewer->addPointCloud(colored_origin_pc, "c1");

    /******************Set Priority for Clusters************************/
    vector<int> priority_vec;
    setPriorityforClusters(cluster_projected_pcs, cluster_origin_pcs ,priority_vec);

    PointCloudPtr_RGB colored_pc(new PointCloud_RGB);
    for(int i=priority_vec.size()-1; i>=0; i--){

      cout<<"priority_vec.at(i):"<<priority_vec.at(i)<<endl;
      PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);

      MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(priority_vec.at(i)), cloud_tem);
      float r, g, b;
      getColorByValue(i*1.0, 0, (cluster_origin_pcs.size()-1)*1.0, &r, &g, &b);
      for(int j=0; j<cloud_tem->size(); j++){
        /*cloud_tem->at(j).r=color_table2[i%6][0];
        cloud_tem->at(j).g=color_table2[i%6][1];
        cloud_tem->at(j).b=color_table2[i%6][2];*/

        cloud_tem->at(j).r=r;
        cloud_tem->at(j).g=g;
        cloud_tem->at(j).b=b;
      }

      appendCloud_RGB(cloud_tem, colored_pc);
    }

    vs.viewer->addPointCloud(colored_pc, "c1");

    Point position;
    PointCloudPtr_RGB cl(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(priority_vec.at(priority_vec.size()-1)), cl);
    getRobotPosition1(cl, wall_rect_clouds, matrix_transform, matrix_translation_r, matrix_transform_r, position, vs);

    //vs.viewer->addSphere(position, 0.1, "pos");



    //for(int i=0; i<cluster_origin_pcs.size() ; i++){
    //  PointCloudPtr box_cloud(new PointCloud);
    //  PointCloudPtr_RGB pc(new PointCloud_RGB);
    //  MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(i), pc);
    //  mark_cloud(pc, matrix_transform, matrix_translation_r, matrix_transform_r, box_cloud);

    //  std::stringstream st;
    //  st<<"box"<<i<<"-";
    //  std::string id_str=st.str();
    //  const char* id=id_str.c_str();

    //  draw_box(box_cloud, vs, 0, 255, 255, id);
    //}

    vs.show();
  }

  return 0;
}

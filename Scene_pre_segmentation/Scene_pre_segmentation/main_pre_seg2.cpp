//#include "visualizer.h"
//#include "file_io.h"
//#include "utility.h"
//#include "PreSegmentation/bigScene_preSeg.h"
//
//int main (int argc, char *argv[])
//{
//  Visualizer vs;
//  vs.viewer->removeAllPointClouds();
//  vs.viewer->removeCoordinateSystem();
//  vs.viewer->setBackgroundColor(0,0,0);
//
//  PointCloudPtr_RGB cloud(new PointCloud_RGB);
//  NormalCloudTPtr normals(new NormalCloudT);
//
//  //loadPointCloud_ply("data/scene0.ply", cloud);
//  loadPointCloud_ply("data/scene1.ply", cloud);
//  //loadPointCloud_ply("data/big_table_after.ply", cloud);
//  //loadPointCloud_ply("data/two_tables.ply", cloud);
//  //loadPointCloud_ply("data/hui_room.ply", cloud);
// 
//
//  /******************detect floor and wall************************/
//  MyPointCloud_RGB floor_cloud;
//  pcl::ModelCoefficients floor_coefficients;
//  MyPointCloud floor_rect_cloud;
//  vector<MyPointCloud_RGB> wall_clouds;
//  std::vector<MyPointCloud> wall_rect_clouds;
//  PointCloudPtr_RGB remained_cloud(new PointCloud_RGB);
//
//  detect_floor_and_walls(cloud, floor_cloud, floor_coefficients, floor_rect_cloud, wall_clouds, wall_rect_clouds, remained_cloud);
//
//  if(floor_cloud.mypoints.size()>0){
//    //draw floor rect
//    if(floor_cloud.mypoints.size()>0)
//    {
//      PointCloudPtr pc(new PointCloud);
//      MyPointCloud2PointCloud(floor_rect_cloud, pc);
//
//      draw_rect(pc, vs, 255, 0, 0, "floor_rect");
//    }
//
//    //draw wall rects
//    for(int k=0;k<wall_rect_clouds.size();k++){
//      PointCloudPtr pcp(new PointCloud);
//      MyPointCloud2PointCloud(wall_rect_clouds.at(k), pcp);
//
//      std::stringstream st;
//      st<<"wall_rect"<<k;
//      std::string id_str=st.str();
//      const char* id=id_str.c_str();
//
//      draw_rect(pcp, vs, 0, 255, 0, id);
//    }
//
//    Eigen::Matrix4f matrix_transform;
//    Eigen::Matrix4f matrix_translation_r;
//    Eigen::Matrix4f matrix_transform_r;
//    getTemTransformMatrix(floor_coefficients, floor_rect_cloud, matrix_transform, matrix_translation_r, matrix_transform_r);
//
//    PointCloudPtr_RGB filter_remained_cloud(new PointCloud_RGB);
//    remove_outliers(remained_cloud, floor_rect_cloud, wall_rect_clouds, matrix_transform, matrix_translation_r, matrix_transform_r, filter_remained_cloud, vs);
//
//    vs.viewer->addPointCloud(filter_remained_cloud,"filter_remained_cloud");
//
//    PointCloudPtr_RGB new_remained_cloud(new PointCloud_RGB);
//    PointCloud_RGB ct;
//    pcl::copyPointCloud(*filter_remained_cloud,ct);
//    pcl::transformPointCloud (ct, *new_remained_cloud, matrix_transform);
//
//    cout<<"new_remained_cloud->size():"<<new_remained_cloud->size()<<endl;
//
//    /******************Euclidean Cluster Extraction************************/
//    std::vector<PointCloudPtr_RGB> cluster_points;
//    big_object_seg_ECE(new_remained_cloud, cluster_points);
//
//    //showPointCloud(new_remained_cloud, "new_remained_cloud");
//
//    //cout<<"cluster_points.size():"<<cluster_points.size()<<endl;
//    for(int i=0;i<cluster_points.size();i++){
//      //cout<<"cluster_points.at(i)->size():"<<cluster_points.at(i)->size()<<endl;
//
//      if(cluster_points.at(i)->size()>1000){
//        //detect support plane
//        vector<MyPointCloud_RGB> support_clouds;
//        std::vector<MyPointCloud> support_rect_clouds;
//        vector<MyPointCloud_RGB> separation_clouds;
//        std::vector<MyPointCloud> separation_rect_clouds;
//        PointCloudPtr_RGB cluster_remained_cloud(new PointCloud_RGB);
//
//        //detect_support_separation_plane(cluster_points.at(i), support_clouds, support_rect_clouds, separation_clouds, separation_rect_clouds, remained_cloud);
//        //detct support plane
//
//        cout<<"=====================detect_support_plane==========================="<<endl;
//        detect_support_plane(cluster_points.at(i), support_clouds, support_rect_clouds, cluster_remained_cloud);
//        
//        if(support_rect_clouds.size()>0){
//        float min_z = 3.0;
//        for (int j = 0; j < support_rect_clouds.size(); j++){
//          for(int k = 0; k < support_rect_clouds.at(j).mypoints.size(); k++){
//            if(support_rect_clouds.at(j).mypoints[k].z < min_z){
//              min_z = support_rect_clouds.at(j).mypoints[k].z;
//            }
//          }
//        }
//
//        PointCloudPtr_RGB table_top_cloud(new PointCloud_RGB);
//        
//        for (int j = 0; j < cluster_remained_cloud->size(); j++){
//          if(cluster_remained_cloud->at(j).z > min_z){
//            table_top_cloud->push_back(cluster_remained_cloud->at(j));
//          }
//        }
//
//        //showPointCloud(table_top_cloud, "table_top_cloud");
//
//        std::vector<PointCloudPtr_RGB> tabletop_cluster_points;
//        big_object_seg_ECE(table_top_cloud, tabletop_cluster_points);
//        for(int m=0; m<tabletop_cluster_points.size(); m++){
//          cout<<"=====================detect_separation_plane==========================="<<endl;
//          detect_separation_plane(tabletop_cluster_points.at(m), separation_clouds, separation_rect_clouds, cluster_remained_cloud);
//
//          if(separation_rect_clouds.size()>0){
//            //draw support rects
//            for(int k=0;k<separation_rect_clouds.size();k++){
//              PointCloudPtr pcp(new PointCloud);
//              MyPointCloud2PointCloud(separation_rect_clouds.at(k), pcp);
//
//              PointCloud pcp_temp;
//              pcl::transformPointCloud (*pcp, pcp_temp, matrix_translation_r);
//              pcl::transformPointCloud (pcp_temp, *pcp, matrix_transform_r);
//
//              std::stringstream st;
//              st<<"separation_rect"<<i<<m<<k<<"-";
//              std::string id_str=st.str();
//              const char* id=id_str.c_str();
//
//              draw_rect(pcp, vs, 255, 0, 255, id);
//            }
//          }
//        }
//        
//      }
//
//        if(support_rect_clouds.size()>0){
//          //draw support rects
//          for(int k=0;k<support_rect_clouds.size();k++){
//            PointCloudPtr pcp(new PointCloud);
//            MyPointCloud2PointCloud(support_rect_clouds.at(k), pcp);
//
//            PointCloud pcp_temp;
//            pcl::transformPointCloud (*pcp, pcp_temp, matrix_translation_r);
//            pcl::transformPointCloud (pcp_temp, *pcp, matrix_transform_r);
//
//            std::stringstream st;
//            st<<"support_rect"<<i<<k<<"-";
//            std::string id_str=st.str();
//            const char* id=id_str.c_str();
//
//            draw_rect(pcp, vs, 0, 0, 255, id);
//
//            /*PointCloudPtr_RGB pcp_RGB(new PointCloud_RGB);
//            MyPointCloud_RGB2PointCloud(support_clouds.at(k), pcp_RGB);
//            showPointCloud(pcp_RGB, "test");*/
//          }
//
//          PointCloudPtr box_cloud(new PointCloud);
//          mark_remaining_cloud(cluster_points.at(i), box_cloud);
//
//          PointCloud box_cloud_temp;
//          pcl::transformPointCloud (*box_cloud, box_cloud_temp, matrix_translation_r);
//          pcl::transformPointCloud (box_cloud_temp, *box_cloud, matrix_transform_r);
//
//          std::stringstream st;
//          st<<"box"<<i<<"-";
//          std::string id_str=st.str();
//          const char* id=id_str.c_str();
//
//          draw_box(box_cloud, vs, 0, 255, 255, id);
//        }
//        else{
//          PointCloudPtr box_cloud(new PointCloud);
//
//          mark_remaining_cloud(cluster_points.at(i), box_cloud);
//
//          std::stringstream st;
//          st<<"box"<<i<<"-";
//          std::string id_str=st.str();
//          const char* id=id_str.c_str();
//
//          PointCloud box_cloud_temp;
//          pcl::transformPointCloud (*box_cloud, box_cloud_temp, matrix_translation_r);
//          pcl::transformPointCloud (box_cloud_temp, *box_cloud, matrix_transform_r);
//
//          draw_box(box_cloud, vs, 255, 255, 0, id);
//        }
//      }
//    }
//
//    vs.show();
//  }
//
//  return 0;
//}
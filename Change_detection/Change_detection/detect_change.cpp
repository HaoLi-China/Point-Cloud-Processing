#include "detect_change.h"

//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//Euclidean Cluster Extraction
void object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.010); // 1cm
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

//detect change of two point cloud
void detect_change(PointCloudPtr_RGB cloud0, PointCloudPtr_RGB cloud1, PointCloudPtr_RGB result0, PointCloudPtr_RGB result1){
   PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);

  // Octree resolution - side length of octree voxels
  float resolution = 0.01f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<Point_RGB> octree0 (resolution);

  // Add points from cloudA to octree
  octree0.setInputCloud (cloud0);
  octree0.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree0.switchBuffers ();

  // Add points from cloudB to octree
  octree0.setInputCloud (cloud1);
  octree0.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector0;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree0.getPointIndicesFromNewVoxels (newPointIdxVector0);

  for (size_t i = 0; i < newPointIdxVector0.size (); ++i){
    cloud_tem->push_back(cloud1->points[newPointIdxVector0[i]]);
  }

  std::vector<PointCloudPtr_RGB> cluster_points0;
  object_seg_ECE(cloud_tem, cluster_points0);

  for(int i=0; i<cluster_points0.size(); i++){
    if(cluster_points0.at(i)->size()>200){
      appendCloud_RGB(cluster_points0.at(i), result0);
    }
  }

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<Point_RGB> octree1 (resolution);

  // Add points from cloudA to octree
  octree1.setInputCloud (cloud1);
  octree1.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree1.switchBuffers ();

  // Add points from cloudB to octree
  octree1.setInputCloud (cloud0);
  octree1.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector1;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree1.getPointIndicesFromNewVoxels (newPointIdxVector1);

  cloud_tem->clear();

  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector1.size (); ++i){
    cloud_tem->push_back(cloud0->points[newPointIdxVector1[i]]);
  }

  std::vector<PointCloudPtr_RGB> cluster_points1;
  object_seg_ECE(cloud_tem, cluster_points1);

  for(int i=0; i<cluster_points1.size(); i++){
    if(cluster_points1.at(i)->size()>200){
      appendCloud_RGB(cluster_points1.at(i), result1);
    }
  }
}

//merge plane in objects
void merge_Plane(std::vector<MyPointCloud_RGB>& plane_clouds, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& new_plane_clouds, std::vector<MyPointCloud>& new_rect_clouds){

  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  if(coefficients_vector.size()<2){
    if(coefficients_vector.size()==1){
      PointCloudPtr_RGB cloud_temp(new PointCloud_RGB);
      PointCloud points_temp;

      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());

      MyPointCloud_RGB2PointCloud_RGB(plane_clouds.at(0), cloud_in_plane);
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
      MyPointCloud_RGB2PointCloud_RGB(plane_clouds.at(0), cloud_temp);
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
    MyPointCloud_RGB2PointCloud_RGB(plane_clouds.at(index), cloud_projected0);
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
          MyPointCloud_RGB2PointCloud_RGB(plane_clouds.at(i), cloud_projected_h);

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

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  PointCloudPtr_RGB cloud_p (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);
  pcl::ExtractIndices<Point_RGB> extract;

  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.015);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_tem);
  seg.segment (*inliers, *plane_coefficients);

  if (inliers->indices.size () < 1000)
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
  PointCloud_RGB2MyPointCloud_RGB(cloud_p, plane_cloud);
  PointCloud2MyPointCloud(rect_cl, rect_cloud);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_tem.swap (cloud_f);

  pcl::copyPointCloud(*cloud_tem,*remained_cloud);
}

//detect separation plane
void detect_separation_plane(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &separation_clouds, std::vector<MyPointCloud> &separation_rect_clouds, PointCloudPtr_RGB remained_cloud){
  separation_clouds.clear();
  separation_rect_clouds.clear();

  std::vector<pcl::ModelCoefficients> separation_plane_coefficients_vector;
  vector<MyPointCloud_RGB> separation_plane_clouds_tem;

  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *cloud_tem);

  PointCloudPtr_RGB cloud_remaining(new PointCloud_RGB);

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

    if(plane_cloud.mypoints.size()<1000){
      printf("Can't fit any more\n");
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    float l=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(1).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(1).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(1).z, 2));
    float w=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(3).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(3).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(3).z, 2));

    cout<<"l=====================:"<<l<<endl;
    cout<<"w=====================:"<<w<<endl;
    cout<<"l*w:"<<l*w<<endl;

    if(l<0.5&&w<0.5){
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    if(l<0.7&&w<0.7){

      pcl::copyPointCloud(*remained_tem, *cloud_tem);
      PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
      MyPointCloud_RGB2PointCloud_RGB(plane_cloud, plane_cloud_tem);
      appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      continue;

      /*cout<<"===========***==========="<<endl;
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;*/
    }

    computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, 0.025, &result);

    cout<<"result=====================:"<<result<<endl;

    float thresdhold=0.12;

    if(result>thresdhold){
      Eigen::Vector3d plane_normal;
      plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
      plane_normal.normalize();

      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(0).z):"<<std::abs(rect_cloud.mypoints.at(0).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(1).z):"<<std::abs(rect_cloud.mypoints.at(1).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(2).z):"<<std::abs(rect_cloud.mypoints.at(2).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(3).z):"<<std::abs(rect_cloud.mypoints.at(3).z)<<endl;

      if(std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<0.25){
        //it is a separation plane
        PointCloudPtr_RGB pc(new PointCloud_RGB);
        MyPointCloud_RGB2PointCloud_RGB(plane_cloud, pc);
        //showPointCloud(pc, "plane_cloud");

        pcl::copyPointCloud(*remained_tem, *cloud_tem);
        //showPointCloud(remained_tem, "test");

        separation_plane_clouds_tem.push_back(plane_cloud);
        separation_plane_coefficients_vector.push_back(*plane_coefficients);
      }
      else{
        pcl::copyPointCloud(*remained_tem, *cloud_tem);
        PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
        MyPointCloud_RGB2PointCloud_RGB(plane_cloud, plane_cloud_tem);
        appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      }
    }
    else{
      /*appendCloud_RGB(cloud_tem,cloud_remaining);
      printf("Can't fit any more\n");
      break;*/
      pcl::copyPointCloud(*remained_tem, *cloud_tem);
      PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
      MyPointCloud_RGB2PointCloud_RGB(plane_cloud, plane_cloud_tem);
      appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      //continue;
    }
  }

  if(separation_plane_clouds_tem.size()>0){
    merge_Plane(separation_plane_clouds_tem, separation_plane_coefficients_vector, separation_clouds, separation_rect_clouds);
  }

  pcl::copyPointCloud(*cloud_remaining, *remained_cloud);
}

//detect table
void detect_table(PointCloudPtr_RGB sourceCloud, pcl::ModelCoefficients& plane_coefficients, PointCloudPtr_RGB planeCloud, PointCloudPtr rect_cloud, PointCloudPtr_RGB remainingCloud){
  pcl::ExtractIndices<Point_RGB> extract;// Create the filtering object
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  PointCloudPtr_RGB cloud_p(new PointCloud_RGB);

  pcl::copyPointCloud(*sourceCloud, *cloud_tem);

  while(1){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
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
    seg.segment (*inliers, plane_coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    //showPointCloud(cloud_tem, "cloud_tem");

    // Extract the inliers
    extract.setInputCloud (cloud_tem);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

    //showPointCloud(planeCloud, "planeCloud");

    getRectForPlaneCloud(planeCloud, boost::make_shared<pcl::ModelCoefficients>(plane_coefficients), rect_cloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_p);
    //showPointCloud(cloud_p, "cloud_p");

    pcl::copyPointCloud(*cloud_p, *cloud_tem);

    Eigen::Vector3d normal;
    normal << plane_coefficients.values[0], plane_coefficients.values[1], plane_coefficients.values[2];
    normal.normalize();

    if(std::abs(normal.dot(Eigen::Vector3d(0,0,1)))>0.7){
      break;
    }
  }

  pcl::copyPointCloud(*cloud_tem, *remainingCloud);
}

//get transform matrix between plane and x_y plane 
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r){
  Eigen::Vector3d normal;
  normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  normal.normalize();

  if(normal.dot(Eigen::Vector3d(0,0,1))<0){
    normal = -normal;
  }

  double angle=acos(normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);
  matrix_transform = matrix.cast<float>();

  getRotationMatrix(axis, -angle, matrix);
  matrix_transform_r = matrix.cast<float>();
}

//get points cloud on the table rect
void getCloudOnTable(PointCloudPtr_RGB cloud, PointCloudPtr rect_cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr_RGB resultCloud){
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  PointCloudPtr rect_cloud_tem(new PointCloud);
  
  pcl::transformPointCloud (*cloud, *cloud_tem, matrix_transform);
  pcl::transformPointCloud (*rect_cloud, *rect_cloud_tem, matrix_transform);
 

    for(int k=0; k<cloud_tem->size(); k++){
      Point_RGB p;
      p.x = cloud_tem->at(k).x;
      p.y = cloud_tem->at(k).y;
      p.z = cloud_tem->at(k).z;
      p.r = cloud_tem->at(k).r;
      p.g = cloud_tem->at(k).g;
      p.b = cloud_tem->at(k).b;

      if(cloud_tem->at(k).z < rect_cloud_tem->at(0).z){
        continue;
      }

      bool flag = true;

      for(int i=0; i<rect_cloud_tem->size(); i++){
        Eigen::Vector3f normal0;
        normal0 << p.x-rect_cloud_tem->at(i).x, p.y-rect_cloud_tem->at(i).y, 0;
        normal0.normalize();

        Eigen::Vector3f normal1;
        normal1 << rect_cloud_tem->at((i+1)%4).x-rect_cloud_tem->at(i).x, rect_cloud_tem->at((i+1)%4).y-rect_cloud_tem->at(i).y, 0;
        normal1.normalize();

        Eigen::Vector3f normal2;
        normal2 << rect_cloud_tem->at((i+3)%4).x-rect_cloud_tem->at(i).x, rect_cloud_tem->at((i+3)%4).y-rect_cloud_tem->at(i).y, 0;
        normal2.normalize();

        if(normal0.dot(normal1)<0||normal0.dot(normal2)<0){
          flag = false;
          break;
        }
      }

      if(flag){
        resultCloud->push_back(p);
      }
    }

    /******************detect separation plane************************/
    vector<MyPointCloud_RGB> separation_clouds;
    std::vector<MyPointCloud> separation_rect_clouds;
    PointCloudPtr_RGB remained_cloud(new PointCloud_RGB);
    detect_separation_plane(resultCloud, separation_clouds, separation_rect_clouds, remained_cloud);

    pcl::transformPointCloud (*remained_cloud, *resultCloud, matrix_transform_r);
}
#include "bigScene_preSeg.h"

//Euclidean Cluster Extraction
void big_object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.015); // 1.5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1500000);
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

//Euclidean Cluster Extraction
void object_seg_ECE2(PointCloudPtr cloud, std::vector<PointCloudPtr> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance (0.015); // 1.5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudPtr cloud_cluster (new PointCloud);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    }
    cluster_points.push_back(cloud_cluster);

    j++;
  }
}

//Euclidean Cluster Extraction
void ECE_for_plane(PointCloudPtr_RGB cloud, std::vector<MyPointCloud_RGB> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.015); // 1.5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1500000);
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

    MyPointCloud_RGB mc;
    PointCloud_RGB2MyPointCloud_RGB(cloud_cluster, mc);
    cluster_points.push_back(mc);

    j++;
  }
}

//get transform matrix between plane and x_y plane 
void getTemTransformMatrix(pcl::ModelCoefficients& coefficients, MyPointCloud& rect_cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r){
  Eigen::Vector3d floor_normal;
  floor_normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  floor_normal.normalize();

  if(floor_normal.dot(Eigen::Vector3d(0,0,1))<0){
    floor_normal = -floor_normal;
  }

  double angle=acos(floor_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=floor_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);
  matrix_transform = matrix.cast<float>();

  PointCloudPtr rect_cl(new PointCloud);
  PointCloudPtr rect_cl_tem(new PointCloud);
  MyPointCloud2PointCloud(rect_cloud, rect_cl_tem);
  pcl::transformPointCloud (*rect_cl_tem, *rect_cl, matrix_transform);

  matrix_transform(2,3)-=rect_cl->at(0).z;

  matrix_translation_r = Eigen::Matrix4f::Identity();
  matrix_translation_r(2,3) += rect_cl->at(0).z;

  getRotationMatrix(axis, -angle, matrix);
  matrix_transform_r = matrix.cast<float>();
}

//remove floor_bottom and wall_back
void remove_outliers(PointCloudPtr_RGB remained_cloud, MyPointCloud& floor_rect_cloud, std::vector<MyPointCloud>& wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr_RGB new_remained_cloud, Visualizer& vs){

  PointCloud_RGB sourceCloud_temp;
  pcl::copyPointCloud(*remained_cloud,sourceCloud_temp);
  pcl::transformPointCloud (sourceCloud_temp, *remained_cloud, matrix_transform);

  PointCloudPtr floor_rect(new PointCloud);
  MyPointCloud2PointCloud(floor_rect_cloud, floor_rect);
  PointCloud floor_rect_temp;
  pcl::copyPointCloud(*floor_rect,floor_rect_temp);
  pcl::transformPointCloud (floor_rect_temp, *floor_rect, matrix_transform);

  Point floor_rect_cen((floor_rect->at(0).x+floor_rect->at(2).x)/2, (floor_rect->at(0).y+floor_rect->at(2).y)/2, (floor_rect->at(0).z+floor_rect->at(2).z)/2);

  //showPointCloud(remained_cloud, "remained_cloud");

  for(int i=0; i < remained_cloud->size(); i++ ){
    if(remained_cloud->at(i).z>0.01){
      new_remained_cloud->push_back(remained_cloud->at(i));
    }
  }

  PointCloudPtr_RGB new_remained_cloud_temp(new PointCloud_RGB);
  pcl::transformPointCloud (*new_remained_cloud, *new_remained_cloud_temp, matrix_translation_r);
  pcl::transformPointCloud (*new_remained_cloud_temp, *new_remained_cloud, matrix_transform_r);

  //showPointCloud(new_remained_cloud_temp, "test");

  //vs.viewer->addPointCloud(new_remained_cloud,"remained_cloud");

  for(int i=0; i < wall_rect_clouds.size(); i++ ){
    Point p0_tem(wall_rect_clouds.at(i).mypoints.at(0).x, wall_rect_clouds.at(i).mypoints.at(0).y, wall_rect_clouds.at(i).mypoints.at(0).z);
    Point p1_tem(wall_rect_clouds.at(i).mypoints.at(1).x, wall_rect_clouds.at(i).mypoints.at(1).y, wall_rect_clouds.at(i).mypoints.at(1).z);
    Point p2_tem(wall_rect_clouds.at(i).mypoints.at(2).x, wall_rect_clouds.at(i).mypoints.at(2).y, wall_rect_clouds.at(i).mypoints.at(2).z);
    Point p3_tem(wall_rect_clouds.at(i).mypoints.at(3).x, wall_rect_clouds.at(i).mypoints.at(3).y, wall_rect_clouds.at(i).mypoints.at(3).z);

    Eigen::Vector3d normal_0;
    normal_0 << p0_tem.x - p2_tem.x, p0_tem.y - p2_tem.y, p0_tem.z - p2_tem.z;
    normal_0.normalize();

    Eigen::Vector3d normal_1;
    normal_1 << p3_tem.x - p1_tem.x, p3_tem.y - p1_tem.y, p3_tem.z - p1_tem.z;
    normal_1.normalize();

    wall_rect_clouds.at(i).mypoints.at(0).x += 0.1*normal_0[0];
    wall_rect_clouds.at(i).mypoints.at(0).y += 0.1*normal_0[1];
    wall_rect_clouds.at(i).mypoints.at(0).z += 0.1*normal_0[2];

    wall_rect_clouds.at(i).mypoints.at(2).x -= 0.1*normal_0[0];
    wall_rect_clouds.at(i).mypoints.at(2).y -= 0.1*normal_0[1];
    wall_rect_clouds.at(i).mypoints.at(2).z -= 0.1*normal_0[2];

    wall_rect_clouds.at(i).mypoints.at(3).x += 0.1*normal_1[0];
    wall_rect_clouds.at(i).mypoints.at(3).y += 0.1*normal_1[1];
    wall_rect_clouds.at(i).mypoints.at(3).z += 0.1*normal_1[2];

    wall_rect_clouds.at(i).mypoints.at(1).x -= 0.1*normal_1[0];
    wall_rect_clouds.at(i).mypoints.at(1).y -= 0.1*normal_1[1];
    wall_rect_clouds.at(i).mypoints.at(1).z -= 0.1*normal_1[2];

    Point p0(wall_rect_clouds.at(i).mypoints.at(0).x, wall_rect_clouds.at(i).mypoints.at(0).y, wall_rect_clouds.at(i).mypoints.at(0).z);
    Point p1(wall_rect_clouds.at(i).mypoints.at(1).x, wall_rect_clouds.at(i).mypoints.at(1).y, wall_rect_clouds.at(i).mypoints.at(1).z);
    Point p2(wall_rect_clouds.at(i).mypoints.at(2).x, wall_rect_clouds.at(i).mypoints.at(2).y, wall_rect_clouds.at(i).mypoints.at(2).z);
    Point p3(wall_rect_clouds.at(i).mypoints.at(3).x, wall_rect_clouds.at(i).mypoints.at(3).y, wall_rect_clouds.at(i).mypoints.at(3).z);

    Eigen::Vector3d normal0;
    normal0 << p0.x - p1.x, p0.y - p1.y, p0.z - p1.z;
    normal0.normalize();

    Eigen::Vector3d normal1;
    normal1 << p0.x - p3.x, p0.y - p3.y, p0.z - p3.z;
    normal1.normalize();

    Eigen::Vector3d normal2;
    normal2 <<floor_rect_cen.x - p0.x, floor_rect_cen.y - p0.y, floor_rect_cen.z - p0.z;
    normal2.normalize();

    Eigen::Vector3d wall_normal = normal0.cross(normal1);
    wall_normal.normalize();

    if(wall_normal.dot(normal2) < 0){
      wall_normal = -wall_normal;
    }

    double angle_t = acos(wall_normal.dot(Eigen::Vector3d(0,1,0)));
    Eigen::Vector3d axis_t = wall_normal.cross(Eigen::Vector3d(0,1,0));
    axis_t.normalize();

    Eigen::Matrix4d matrix_t;
    getRotationMatrix(axis_t, angle_t, matrix_t);
    Eigen::Matrix4f matrix_transform_t = matrix_t.cast<float>();

    getRotationMatrix(axis_t, -angle_t, matrix_t);
    Eigen::Matrix4f matrix_transform_r_t = matrix_t.cast<float>();

    PointCloudPtr rect_cl_t(new PointCloud);
    PointCloudPtr rect_cl_tem_t(new PointCloud);
    MyPointCloud2PointCloud(wall_rect_clouds.at(i), rect_cl_tem_t);
    pcl::transformPointCloud (*rect_cl_tem_t, *rect_cl_t, matrix_transform_t);


    PointCloud_RGB cloud_temp_t;
    pcl::copyPointCloud(*new_remained_cloud, cloud_temp_t);
    pcl::transformPointCloud (cloud_temp_t, *new_remained_cloud, matrix_transform_t);

    //showPointCloud(new_remained_cloud, "test");

    pcl::ExtractIndices<Point_RGB> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    PointCloudPtr_RGB new_remained_cloud_t(new PointCloud_RGB);
    for(int j=0; j < new_remained_cloud->size(); j++ ){
      if(new_remained_cloud->at(j).y <= rect_cl_t->at(0).y+0.02){
        bool flag = true;

        for(int i=0; i<rect_cl_t->size(); i++){
          Eigen::Vector3f normal0;
          normal0 << new_remained_cloud->at(j).x-rect_cl_t->at(i).x, 0, new_remained_cloud->at(j).z-rect_cl_t->at(i).z;
          normal0.normalize();

          Eigen::Vector3f normal1;
          normal1 << rect_cl_t->at((i+1)%4).x-rect_cl_t->at(i).x, 0, rect_cl_t->at((i+1)%4).z-rect_cl_t->at(i).z;
          normal1.normalize();

          Eigen::Vector3f normal2;
          normal2 << rect_cl_t->at((i+3)%4).x-rect_cl_t->at(i).x, 0, rect_cl_t->at((i+3)%4).z-rect_cl_t->at(i).z;
          normal2.normalize();

          if(normal0.dot(normal1)<0||normal0.dot(normal2)<0){
            flag=false;
            break;
          }
        }

        if(flag){
          inliers->indices.push_back(j);
        }
      }
    }

    //PointCloudPtr_RGB new_remained_cloud_t(new PointCloud_RGB);
    //for(int j=0; j < new_remained_cloud->size(); j++ ){
    //  if(new_remained_cloud->at(j).y < rect_cl_t->at(0).y-0.02){
    //    new_remained_cloud_t->push_back(new_remained_cloud->at(j));
    //  }
    //}

    PointCloudPtr_RGB cloud_p(new PointCloud_RGB);
    // Extract the inliers
    extract.setInputCloud (new_remained_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_p);

    //showPointCloud(cloud_p, "test");

    PointCloud_RGB new_remained_cloud_temp_t;
    pcl::copyPointCloud(*cloud_p,new_remained_cloud_temp_t);
    pcl::transformPointCloud (new_remained_cloud_temp_t, *cloud_p, matrix_transform_r_t);
    pcl::copyPointCloud(*cloud_p, *new_remained_cloud);

    /*if(new_remained_cloud_t1->size() < new_remained_cloud_t0->size()){
    PointCloud_RGB new_remained_cloud_temp_t;
    pcl::copyPointCloud(*new_remained_cloud_t0,new_remained_cloud_temp_t);
    pcl::transformPointCloud (new_remained_cloud_temp_t, *new_remained_cloud_t0, matrix_transform_r_t);
    pcl::copyPointCloud(*new_remained_cloud_t0, *new_remained_cloud);
    }
    else{
    PointCloud_RGB new_remained_cloud_temp_t;
    pcl::copyPointCloud(*new_remained_cloud_t1,new_remained_cloud_temp_t);
    pcl::transformPointCloud (new_remained_cloud_temp_t, *new_remained_cloud_t1, matrix_transform_r_t);
    pcl::copyPointCloud(*new_remained_cloud_t1, *new_remained_cloud);
    }*/
  }
}

//get points cloud on the table rect
void getCloudOnTable(PointCloudPtr_RGB tableTopCloud, vector<MyPointCloud_RGB>& support_clouds, vector<MyPointCloud>& rect_clouds, vector<MyPointCloud>& resultClouds){
  for(int m=0; m<rect_clouds.size(); m++){
    MyPointCloud resultCloud;

    MyPointCloud_RGB2MyPointCloud(support_clouds.at(m), resultCloud);

    for(int i=0; i<resultCloud.mypoints.size(); i++){
      resultCloud.mypoints.at(i).z = 0;
    }

    MyPointCloud rect_cloud = rect_clouds.at(m);

    for(int k=0; k<tableTopCloud->size(); k++){
      MyPt p;
      p.x = tableTopCloud->at(k).x;
      p.y = tableTopCloud->at(k).y;
      p.z = 0;

      if(tableTopCloud->at(k).z < rect_cloud.mypoints.at(0).z){
        continue;
      }

      bool flag = true;

      for(int i=0; i<rect_cloud.mypoints.size(); i++){
        Eigen::Vector3f normal0;
        normal0 << p.x-rect_cloud.mypoints.at(i).x, p.y-rect_cloud.mypoints.at(i).y, 0;
        normal0.normalize();

        Eigen::Vector3f normal1;
        normal1 << rect_cloud.mypoints.at((i+1)%4).x-rect_cloud.mypoints.at(i).x, rect_cloud.mypoints.at((i+1)%4).y-rect_cloud.mypoints.at(i).y, 0;
        normal1.normalize();

        Eigen::Vector3f normal2;
        normal2 << rect_cloud.mypoints.at((i+3)%4).x-rect_cloud.mypoints.at(i).x, rect_cloud.mypoints.at((i+3)%4).y-rect_cloud.mypoints.at(i).y, 0;
        normal2.normalize();

        if(normal0.dot(normal1)<0||normal0.dot(normal2)<0){
          flag = false;
          break;
        }
      }

      if(flag){
        resultCloud.mypoints.push_back(p);
      }
    }

    PointCloudPtr cloud(new PointCloud);
    MyPointCloud2PointCloud(resultCloud, cloud);

    std::vector<PointCloudPtr> cluster_points;
    object_seg_ECE2(cloud, cluster_points);

    for(int k=0; k<cluster_points.size(); k++){
      if(cluster_points.at(k)->size() > resultCloud.mypoints.size()/2){
        MyPointCloud mc;
        PointCloud2MyPointCloud(cluster_points.at(k),  mc);
        resultClouds.push_back(mc);
      }
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

  pcl::NormalEstimation<Point_RGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<Point_RGB, pcl::Normal> seg; 
  pcl::ExtractIndices<Point_RGB> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  PointCloudPtr_RGB cloud_p (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_tem);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_tem);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
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

//detct floor
void detect_floor_and_walls(PointCloudPtr_RGB cloud, MyPointCloud_RGB& floor_cloud, pcl::ModelCoefficients& floor_coefficients, MyPointCloud& floor_rect_cloud, vector<MyPointCloud_RGB> &wall_clouds, vector<MyPointCloud> &wall_rect_clouds, PointCloudPtr_RGB remained_cloud){
  PointCloudPtr_RGB cloud_tem (new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *cloud_tem);

  //// Create the filtering object
  //pcl::VoxelGrid<Point_RGB> sor;
  //sor.setInputCloud (cloud);
  //sor.setLeafSize (0.002f, 0.002f, 0.002f);
  //sor.filter (*cloud_tem);

  //cout<<"cloud_tem.size:"<<cloud_tem->size()<<endl;

  std::vector<pcl::ModelCoefficients> plane_coefficients_vector;
  vector<MyPointCloud_RGB> plane_clouds_tem;

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

    MyPointCloud_RGB2MyPointCloud(plane_cloud, plane_cloud_n);

    /* PointCloudPtr_RGB test_cloud(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud(plane_cloud, test_cloud);
    showPointCloud (test_cloud,"test_cloud");*/

    if(plane_cloud.mypoints.size()<40000){
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

      /* PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
      MyPointCloud_RGB2PointCloud(plane_cloud, plane_cloud_tem);
      appendCloud_RGB(plane_cloud_tem,cloud_remaining);
      continue;*/
    }

    computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, 0.020, &result);

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
          MyPointCloud_RGB2PointCloud_RGB(plane_cloud, plane_cloud_tem);
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
      continue;
    }
  }

  if(plane_clouds_tem.size()>0){
    merge_Plane(plane_clouds_tem, plane_coefficients_vector, wall_clouds, wall_rect_clouds);
  }

  pcl::copyPointCloud(*cloud_remaining, *remained_cloud);
}

//detct support plane
void detect_support_plane(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &support_clouds, std::vector<MyPointCloud> &support_rect_clouds, PointCloudPtr_RGB remained_cloud){
  support_clouds.clear();
  support_rect_clouds.clear();

  std::vector<pcl::ModelCoefficients> support_plane_coefficients_vector;
  vector<MyPointCloud_RGB> support_plane_clouds_tem;

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

    if(plane_cloud.mypoints.size()<10000){
      printf("Can't fit any more\n");
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    float l=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(1).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(1).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(1).z, 2));
    float w=sqrt(pow(rect_cloud.mypoints.at(0).x-rect_cloud.mypoints.at(3).x, 2)+pow(rect_cloud.mypoints.at(0).y-rect_cloud.mypoints.at(3).y, 2)+pow(rect_cloud.mypoints.at(0).z-rect_cloud.mypoints.at(3).z, 2));

    cout<<"l=====================:"<<l<<endl;
    cout<<"w=====================:"<<w<<endl;
    cout<<"l*w:"<<l*w<<endl;

    if(l*w<0.5){
      cout<<"===========***==========="<<endl;
      appendCloud_RGB(cloud_tem,cloud_remaining);
      break;
    }

    computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, 0.02, &result);

    cout<<"result=====================:"<<result<<endl;

    float thresdhold=0.12;

    if(result>thresdhold){
      Eigen::Vector3d plane_normal;
      plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
      plane_normal.normalize();

      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<<endl;
      cout<<"std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))):"<<std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1)))<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(0).z):"<<std::abs(rect_cloud.mypoints.at(0).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(1).z):"<<std::abs(rect_cloud.mypoints.at(1).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(2).z):"<<std::abs(rect_cloud.mypoints.at(2).z)<<endl;
      cout<<"std::abs(rect_cloud.mypoints.at(3).z):"<<std::abs(rect_cloud.mypoints.at(3).z)<<endl;

      if(std::abs(plane_normal.dot(Eigen::Vector3d(0,0,1))-1)<0.045
        &&std::abs(rect_cloud.mypoints.at(0).z)<1&&std::abs(rect_cloud.mypoints.at(1).z)<1
        &&std::abs(rect_cloud.mypoints.at(2).z)<1&&std::abs(rect_cloud.mypoints.at(3).z)<1
        &&std::abs(rect_cloud.mypoints.at(0).z)>=0.25&&std::abs(rect_cloud.mypoints.at(1).z)>=0.25
        &&std::abs(rect_cloud.mypoints.at(2).z)>=0.25&&std::abs(rect_cloud.mypoints.at(3).z)>=0.25){
          pcl::copyPointCloud(*remained_tem, *cloud_tem);

          PointCloudPtr_RGB pc(new PointCloud_RGB);
          MyPointCloud_RGB2PointCloud_RGB(plane_cloud, pc);
          /*showPointCloud(pc, "new_remained_cloud");

          PointCloudPtr rect_pc(new PointCloud);
          MyPointCloud2PointCloud(rect_cloud, rect_pc);
          showPointCloud2(rect_pc, "rect_pc");*/

          std::vector<PointCloudPtr_RGB> cluster_points;
          big_object_seg_ECE(pc, cluster_points);

          PointCloudPtr_RGB new_plane_cloud(new PointCloud_RGB);
          for(int k=0; k<cluster_points.size(); k++){
            if(cluster_points.at(k)->size()>50){
              appendCloud_RGB(cluster_points.at(k), new_plane_cloud);
            }
          }

          MyPointCloud_RGB my_new_plane_cloud;
          PointCloud_RGB2MyPointCloud_RGB(new_plane_cloud, my_new_plane_cloud);

          //it is a support plane
          support_plane_clouds_tem.push_back(my_new_plane_cloud);
          support_plane_coefficients_vector.push_back(*plane_coefficients);
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

  if(support_plane_clouds_tem.size()>0){
    merge_Plane(support_plane_clouds_tem, support_plane_coefficients_vector, support_clouds, support_rect_clouds);
  }

  pcl::copyPointCloud(*cloud_remaining, *remained_cloud);
}



//detct separation plane
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

        std::vector<PointCloudPtr_RGB> cluster_points;
        big_object_seg_ECE(pc, cluster_points);

        PointCloudPtr_RGB new_plane_cloud(new PointCloud_RGB);

        for(int k=0; k<cluster_points.size(); k++){
          if(cluster_points.at(k)->size()>10){
            appendCloud_RGB(cluster_points.at(k), new_plane_cloud);
          }
        }

        PointCloudPtr rect(new PointCloud);
        getRectForPlaneCloud(new_plane_cloud, plane_coefficients, rect);

        float ll=sqrt(pow(rect->at(0).x-rect->at(1).x, 2)+pow(rect->at(0).y-rect->at(1).y, 2)+pow(rect->at(0).z-rect->at(1).z, 2));
        float ww=sqrt(pow(rect->at(0).x-rect->at(3).x, 2)+pow(rect->at(0).y-rect->at(3).y, 2)+pow(rect->at(0).z-rect->at(3).z, 2));

        if(ll>0.7||ww>0.7){
          MyPointCloud_RGB my_new_plane_cloud;
          PointCloud_RGB2MyPointCloud_RGB(new_plane_cloud, my_new_plane_cloud);

          separation_plane_clouds_tem.push_back(my_new_plane_cloud);
          separation_plane_coefficients_vector.push_back(*plane_coefficients);
        }
        else{
          PointCloudPtr_RGB plane_cloud_tem (new PointCloud_RGB);
          MyPointCloud_RGB2PointCloud_RGB(plane_cloud, plane_cloud_tem);
          appendCloud_RGB(plane_cloud_tem,cloud_remaining);
          continue;
        }
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

//pre-segment scene
void pre_segment_scene(PointCloudPtr_RGB transformed_cloud,  vector<MyPointCloud>& sum_support_clouds, vector<MyPointCloud>& sum_separation_rect_clouds){
  /******************Euclidean Cluster Extraction************************/
  std::vector<PointCloudPtr_RGB> cluster_points;
  big_object_seg_ECE(transformed_cloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){

    if(cluster_points.at(i)->size()>1000){
      //detect support plane
      vector<MyPointCloud_RGB> support_clouds;
      std::vector<MyPointCloud> support_rect_clouds;
      vector<MyPointCloud_RGB> separation_clouds;
      std::vector<MyPointCloud> separation_rect_clouds;
      PointCloudPtr_RGB cluster_remained_cloud(new PointCloud_RGB);
      PointCloudPtr_RGB table_top_remained_cloud(new PointCloud_RGB);

      //detct support plane
      detect_support_plane(cluster_points.at(i), support_clouds, support_rect_clouds, cluster_remained_cloud);

      if(support_rect_clouds.size()>0){
        float min_z = 3.0;
        for (int j = 0; j < support_rect_clouds.size(); j++){
          for(int k = 0; k < support_rect_clouds.at(j).mypoints.size(); k++){
            if(support_rect_clouds.at(j).mypoints[k].z < min_z){
              min_z = support_rect_clouds.at(j).mypoints[k].z;
            }
          }
        }

        PointCloudPtr_RGB table_top_cloud(new PointCloud_RGB);

        for (int j = 0; j < cluster_remained_cloud->size(); j++){
          if(cluster_remained_cloud->at(j).z > min_z){
            table_top_cloud->push_back(cluster_remained_cloud->at(j));
          }
        }

        std::vector<PointCloudPtr_RGB> tabletop_cluster_points;
        big_object_seg_ECE(table_top_cloud, tabletop_cluster_points);

        for(int m=0; m<tabletop_cluster_points.size(); m++){
          detect_separation_plane(tabletop_cluster_points.at(m), separation_clouds, separation_rect_clouds, cluster_remained_cloud);

          for(int j = 0; j < separation_rect_clouds.size(); j++){
            sum_separation_rect_clouds.push_back(separation_rect_clouds.at(j));
          }

          appendCloud_RGB(cluster_remained_cloud, table_top_remained_cloud);
        }

        //get points cloud on the table
        getCloudOnTable(table_top_remained_cloud, support_clouds, support_rect_clouds, sum_support_clouds);
      }
    }
  }

  //debug
  /*for(int i=0; i<sum_support_clouds.size(); i++){
    PointCloudPtr pc(new PointCloud);
    MyPointCloud2PointCloud(sum_support_clouds.at(i), pc);
    showPointCloud2(pc, "test");
  }*/
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
  if(max>1.0){
    max=1.0;
  }

  int tmp = static_cast<int>((val - min) / (max - min) * 255);
  *r = g_color_table[tmp][0] * 255;
  *g = g_color_table[tmp][1] * 255;
  *b = g_color_table[tmp][2] * 255;
}


//compute gaussian curvature
void compute_gaussian_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& curvatures, PointCloudPtr_RGB cloud_colored){
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

  cout<<"pcs->points.size():"<<pcs->points.size()<<endl;
  cout<<"cloud->size():"<<cloud->size()<<endl;

  for (unsigned int i = 0; i < pcs->points.size(); i++) {
    float r=0;
    float g=0;
    float b=0;

    Point_Cur_RGB cur;

    if(!(_isnan(pcs->points[i].pc1)||_isnan(pcs->points[i].pc2))){
      cur.curvature=pcs->points[i].pc1*pcs->points[i].pc2;
      //cout<<"cur.curvature:"<<cur.curvature<<endl;
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

  cout<<"min_curv_val:"<<min_curv_val<<endl;
  cout<<"max_curv_val:"<<max_curv_val<<endl;

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

    float radius = 0.002;

    if(flags.at(i)){
      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
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

//normalize projected curvature
void normalize_projected_curvature(PointCloudPtr_RGB cloud, vector<Point_Cur_RGB>& projected_curvatures, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& new_curvatures, int *rows, int *cols){
  /*PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  for(int i=0;i<projected_curvatures.size();i++){
  Point_RGB  
  }*/

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(cloud, p0,p1,p2,p3);

  MyPointCloud rect_mpt;
  MyPointCloud sample_mpt;

  MyPt pt0={p0.x,p0.y,0};
  MyPt pt1={p1.x,p1.y,0};
  MyPt pt2={p2.x,p2.y,0};
  MyPt pt3={p3.x,p3.y,0};
  rect_mpt.mypoints.push_back(pt0);
  rect_mpt.mypoints.push_back(pt1);
  rect_mpt.mypoints.push_back(pt2);
  rect_mpt.mypoints.push_back(pt3);

  float grid_len=0.02;

  samplePlane(rect_mpt, grid_len, sample_mpt, rows, cols);

  cout<<"cloud.size():"<<cloud->size()<<endl;
  cout<<"sample_mpt.mypoints.size():"<<sample_mpt.mypoints.size()<<endl;

  pcl::KdTreeFLANN<Point_RGB> kdtree;
  kdtree.setInputCloud (cloud);

  float min_curv_val = 10000;
  float max_curv_val = -10000;

  for (unsigned int i = 0; i < sample_mpt.mypoints.size(); i++) {
    float curvature=0;

    Point_RGB searchPoint;
    searchPoint.x=sample_mpt.mypoints.at(i).x;
    searchPoint.y=sample_mpt.mypoints.at(i).y;
    searchPoint.z=sample_mpt.mypoints.at(i).z;

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = grid_len/2;

    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j){
        curvature+=projected_curvatures.at(pointIdxRadiusSearch[j]).curvature;
      }
      curvature/=pointIdxRadiusSearch.size();
    }

    if (curvature > max_curv_val){
      max_curv_val = curvature;
    }
    if (curvature < min_curv_val){
      min_curv_val = curvature;
    }

    Point_Cur_RGB c;
    c.x=sample_mpt.mypoints.at(i).x;
    c.y=sample_mpt.mypoints.at(i).y;
    c.z=0;
    c.curvature=curvature;
    new_curvatures.push_back(c);
  }

  cout<<"min_curv_val:"<<min_curv_val<<endl;
  cout<<"max_curv_val:"<<max_curv_val<<endl;

  for (unsigned int i = 0; i < new_curvatures.size(); i++) {
    float r=0;
    float g=0;
    float b=0;
    getColorByValue(new_curvatures.at(i).curvature, min_curv_val, max_curv_val, &r, &g, &b);

    new_curvatures.at(i).r=r;
    new_curvatures.at(i).g=g;
    new_curvatures.at(i).b=b;

    Point_RGB p;
    p.x=new_curvatures.at(i).x;
    p.y=new_curvatures.at(i).y;
    p.z=new_curvatures.at(i).z;
    p.r=r;
    p.g=g;
    p.b=b;

    cloud_colored->push_back(p);
  }

}

//Gaussian Blur
void gaussian_blur(vector<Point_Cur_RGB>& curvatures, int rows, int cols, PointCloudPtr_RGB cloud_colored, vector<Point_Cur_RGB>& new_curvatures){
  cv::Mat src(rows, cols, CV_32FC1);
  cv::Mat dst; 

  int index=0;
  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      src.at<float>(i,j)=curvatures.at(index).curvature;
      index++;
    }
  }

  cv::GaussianBlur( src, dst, cv::Size(3, 3), 0, 0 );

  float min_curv_val = 10000;
  float max_curv_val = -10000;

  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      if (dst.at<float>(i,j) > max_curv_val){
        max_curv_val = dst.at<float>(i,j);
      }
      if (dst.at<float>(i,j) < min_curv_val){
        min_curv_val = dst.at<float>(i,j);
      }
    }
  }

  index=0;
  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      float r=0;
      float g=0;
      float b=0;

      new_curvatures.push_back(curvatures.at(index));
      new_curvatures.at(index).curvature=dst.at<float>(i,j);

      getColorByValue(new_curvatures.at(index).curvature, min_curv_val, max_curv_val, &r, &g, &b);

      new_curvatures.at(index).r=r;
      new_curvatures.at(index).g=g;
      new_curvatures.at(index).b=b;

      index++;
    }
  }

  for(int i=0;i<new_curvatures.size();i++){
    Point_RGB p;
    p.x=new_curvatures.at(i).x;
    p.y=new_curvatures.at(i).y;
    p.z=new_curvatures.at(i).z;
    p.r=new_curvatures.at(i).r;
    p.g=new_curvatures.at(i).g;
    p.b=new_curvatures.at(i).b;

    cloud_colored->push_back(p);
  }
}

//gaussian kernel function
float gauss(float x)  
{  
  return exp(-x);  
}  

//if a point in a rect
bool isInRect(Point p, MyPointCloud& rect_cloud){

  for(int i=0; i<rect_cloud.mypoints.size(); i++){
    Eigen::Vector3f normal0;
    normal0 << p.x-rect_cloud.mypoints.at(i).x, p.y-rect_cloud.mypoints.at(i).y, p.z-rect_cloud.mypoints.at(i).z;
    normal0.normalize();

    Eigen::Vector3f normal1;
    normal1 << rect_cloud.mypoints.at((i+1)%4).x-rect_cloud.mypoints.at(i).x, rect_cloud.mypoints.at((i+1)%4).y-rect_cloud.mypoints.at(i).y, rect_cloud.mypoints.at((i+1)%4).z-rect_cloud.mypoints.at(i).z;
    normal1.normalize();

    Eigen::Vector3f normal2;
    normal2 << rect_cloud.mypoints.at((i+3)%4).x-rect_cloud.mypoints.at(i).x, rect_cloud.mypoints.at((i+3)%4).y-rect_cloud.mypoints.at(i).y, rect_cloud.mypoints.at((i+3)%4).z-rect_cloud.mypoints.at(i).z;
    normal2.normalize();

    if(normal0.dot(normal1)<0||normal0.dot(normal2)<0){
      return false;
    }
  }

  return true;
}


//if two points in the same rect
bool isInSameRect(Point p1, Point p2, vector<MyPointCloud>& rect_clouds){
  for(int i=0; i<rect_clouds.size(); i++){
    if(isInRect(p1, rect_clouds.at(i))&&isInRect(p2, rect_clouds.at(i))){
      return true;
    }
  }

  return false;
}

//if a point in a plane
bool isInPlane(Point p, pcl::KdTreeFLANN<Point> tree){

  // Neighbors containers  
  vector<int> k_indices;  
  vector<float> k_distances;  

  tree.radiusSearch (p, 0.005, k_indices, k_distances);  

  if(k_indices.size()>0){
    return true;
  }

  return false;
}


//if two points in the same plane
bool isInSamePlane(Point p1, Point p2, vector<pcl::KdTreeFLANN<Point>> trees){


  for(int i=0; i<trees.size(); i++){
    if(isInPlane(p1, trees.at(i))&&isInPlane(p2, trees.at(i))){
      return true;
    }
  }

  return false;
}

//Wm5IntrLine2Line2
bool testIntrLine2Line2(Point p0, Point p1 , Point p2, Point p3){

  //Triangle3<float> triangle(Vector3<float>(p0_0.x,p0_0.y,p0_0.z),Vector3<float>(p0_1.x,p0_1.y,p0_1.z),Vector3<float>(p0_2.x,p0_2.y,p0_2.z));
  Segment2<float> segment0(Vector2<float>(p0.x, p0.y), Vector2<float>(p1.x, p1.y));
  Segment2<float> segment1(Vector2<float>(p2.x, p2.y), Vector2<float>(p3.x, p3.y));

  IntrSegment2Segment2<float> intrSegment2Segment2(segment0, segment1);

  bool bo=intrSegment2Segment2.Test();

  return bo;
}

//if two points are separated by separation plane 
bool isSeparated(Point p0, Point p1, vector<MyLine>& lines){
  for(int i=0; i<lines.size(); i++){
    if(testIntrLine2Line2(p0, p1 , Point(lines.at(i).p0.x, lines.at(i).p0.y, 0), Point(lines.at(i).p1.x, lines.at(i).p1.y, 0))){
      return true;
    }
  }

  return false;
}

//execute Mean Shift For Each Point
void execMeanShiftForEachPoint(PointCloudPtr in_cloud, vector<MyLine>& lines, float radius, float scene_scale, PointCloudPtr out_cloud)  
{  
  pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point>);  
  tree->setInputCloud (in_cloud);  

  cout<<"in_cloud->size():"<<in_cloud->size()<<endl;

  for (int i = 0; i < in_cloud->size(); i++)  
  { 
    //cout<<"i:"<<i<<endl;

    if(i%500==0){
      cout<<"execMeanShiftForEachPoint progress:"<<i*100.0/in_cloud->size()<<"%"<<endl;
    }

    //cout<<"i:"<<i<<endl;
    Point pnt = in_cloud->at(i);  

    for(int j = 0; j < 1000; j++)
    {  
      // Neighbors containers  
      vector<int> k_indices;  
      vector<float> k_distances;  

      float sum_weigh = 0;  
      float x = 0.0f, y = 0.0f, z = 0.0f;  
      float dist_pnts = 0.0f;  

      tree->radiusSearch (pnt, radius, k_indices, k_distances);  

      for (int k = 0; k < k_indices.size(); ++k)  
      {  
        int index = k_indices[k];  
        Point nbhd_pnt = in_cloud->points[index];  

        float w=0;

        if(isSeparated(pnt, nbhd_pnt, lines)){
          w=0;
        }
        else{
          float sqr_dist = k_distances[k];  
          float gauss_param = sqr_dist / (0.0025*scene_scale*scene_scale);  
          w= gauss(gauss_param);  
        }

        /* if(w==0){
        cout<<"w:"<<w<<endl;
        }*/


        x += nbhd_pnt.x * w;  
        y += nbhd_pnt.y * w;  
        z += nbhd_pnt.z * w;  
        sum_weigh += w;  
      }  
      x = x / sum_weigh;  
      y = y / sum_weigh;  
      z = z / sum_weigh;  

      float diff_x = x - pnt.x, diff_y = y - pnt.y, diff_z = z - pnt.z;  

      dist_pnts = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);  

      if (dist_pnts <= 0.002)  
      {  
        break;  
      }  
      pnt.x = x;  
      pnt.y = y;  
      pnt.z = z;  
    }

    out_cloud->push_back(pnt); 
  }

  cout<<"execMeanShiftForEachPoint progress:"<<100.0<<"%"<<endl;
}  

//mean shift cluster
void mean_shift_cluster(PointCloudPtr in_cloud, vector<MyLine>& lines, float radius, float scene_scale, vector<MyPointCloud>& clustering_cloud){
  PointCloudPtr out_cloud(new PointCloud);

  execMeanShiftForEachPoint(in_cloud, lines, radius, scene_scale, out_cloud);

  vector<bool> flags(out_cloud->size());
  for(unsigned int i = 0; i < flags.size(); i++){
    flags[i]=true;
  }

  cout<<"out_cloud->size():"<<out_cloud->size()<<endl;

  for (int i = 0; i < out_cloud->size(); i++)  
  {  
    if(!flags[i]){
      continue;
    }

    MyPointCloud mpc;

    MyPt mp;
    mp.x=in_cloud->at(i).x;
    mp.y=in_cloud->at(i).y;
    mp.z=in_cloud->at(i).z;

    mpc.mypoints.push_back(mp);
    flags[i]=false;

    for (int j = i + 1; j < out_cloud->size(); j++)  
    {  
      if(!flags[j]){
        continue;
      }

      Point mode1=out_cloud->at(i);
      Point mode2=out_cloud->at(j);

      float dis=sqrt(pow(mode1.x-mode2.x, 2)+pow(mode1.y-mode2.y, 2)+pow(mode1.z-mode2.z, 2));

      if (dis <= 0.04)  
      {  
        MyPt mp;
        mp.x=in_cloud->at(j).x;
        mp.y=in_cloud->at(j).y;
        mp.z=in_cloud->at(j).z;

        mpc.mypoints.push_back(mp);
        flags[j]=false;
      }
    }  //  for j  

    clustering_cloud.push_back(mpc);
  }  //  for i  
}

//segment scene
void segment_scene(PointCloudPtr_RGB cloud, vector<MyPointCloud>& support_plane_clouds, vector<MyPointCloud>& separation_rect_clouds, vector<MyPointCloud>& clustering_cloud, Visualizer& vs){
  vector<pcl::KdTreeFLANN<Point>> trees;  

  for(int i=0; i<support_plane_clouds.size(); i++){
    for(int j=0; j<support_plane_clouds.at(i).mypoints.size(); j++){
      support_plane_clouds.at(i).mypoints.at(j).z=0;
    }

    PointCloudPtr cloud_p(new PointCloud);
    MyPointCloud2PointCloud(support_plane_clouds.at(i), cloud_p);

    //vs.viewer->addPointCloud(cloud_p, "121");

    //showPointCloud2(cloud_p, "test");

    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud (cloud_p);

    trees.push_back(tree);
  }

  vector<MyLine> lines;

  for(int i=0; i<separation_rect_clouds.size(); i++){
    Point p0(separation_rect_clouds.at(i).mypoints[0].x, separation_rect_clouds.at(i).mypoints[0].y, 0);
    Point p1(separation_rect_clouds.at(i).mypoints[1].x, separation_rect_clouds.at(i).mypoints[1].y, 0);
    Point p2(separation_rect_clouds.at(i).mypoints[2].x, separation_rect_clouds.at(i).mypoints[2].y, 0);
    Point p3(separation_rect_clouds.at(i).mypoints[3].x, separation_rect_clouds.at(i).mypoints[3].y, 0);

    float dis0 = sqrt(pow(p0.x-p1.x, 2)+pow(p0.y-p1.y, 2));
    float dis1 = sqrt(pow(p0.x-p3.x, 2)+pow(p0.y-p3.y, 2));

    MyLine ml;

    if(dis0 < dis1){

      ml.p0.x = (p0.x + p1.x) / 2.0;
      ml.p0.y = (p0.y + p1.y) / 2.0;
      ml.p1.x = (p2.x + p3.x) / 2.0;
      ml.p1.y = (p2.y + p3.y) / 2.0;
    }
    else{
      ml.p0.x = (p0.x + p3.x) / 2.0;
      ml.p0.y = (p0.y + p3.y) / 2.0;
      ml.p1.x = (p1.x + p2.x) / 2.0;
      ml.p1.y = (p1.y + p2.y) / 2.0;
    }

    float len = sqrt(pow(ml.p0.x-ml.p1.x, 2)+pow(ml.p0.y-ml.p1.y, 2));
    float vct_x=(ml.p1.x-ml.p0.x)/len;
    float vct_y=(ml.p1.y-ml.p0.y)/len;

    ml.p0.x -= vct_x*0.05;
    ml.p0.y -= vct_y*0.05;
    ml.p1.x += vct_x*0.05;
    ml.p1.y += vct_y*0.05;

    lines.push_back(ml);

    std::stringstream st;
    st<<"line"<<i;
    std::string id=st.str();

    vs.viewer->addLine(Point(ml.p0.x, ml.p0.y, 0), Point(ml.p1.x, ml.p1.y, 0), 0, 255, 0, id);
  }

  //compute mean curvature
  vector<Point_Cur_RGB> curvatures;
  PointCloudPtr_RGB cloud_colored(new PointCloud_RGB);
  compute_mean_curvature(cloud, curvatures, cloud_colored);

  //projecte curvature to x_y plane
  vector<Point_Cur_RGB> projected_curvatures;
  curvature_projected(cloud, curvatures, cloud_colored, projected_curvatures);

  PointCloudPtr_RGB new_cloud_colored(new PointCloud_RGB);
  vector<Point_Cur_RGB> new_curvatures;
  int rows=0;
  int cols=0;
  normalize_projected_curvature(cloud_colored, projected_curvatures, new_cloud_colored, new_curvatures, &rows, &cols);

  PointCloudPtr_RGB cloud_blur(new PointCloud_RGB);
  vector<Point_Cur_RGB> curvatures_blur;
  gaussian_blur(new_curvatures, rows, cols, cloud_blur, curvatures_blur);

  PointCloudPtr in_cloud(new PointCloud);
  //float radius=0.9;
  float radius=0.5;
  float scene_scale=4.0;

  for(int i=0; i < curvatures_blur.size(); i++){
    if(curvatures_blur.at(i).curvature > 0.3){
      in_cloud->push_back(Point(curvatures_blur.at(i).x, curvatures_blur.at(i).y, curvatures_blur.at(i).z));
    }
  }

  showPointCloud(cloud_blur, "cloud_blur");

  PointCloudPtr cloud_tem(new PointCloud);
  vector<bool> flags(in_cloud->size());
  for(unsigned int i = 0; i < flags.size(); i++){
    flags[i]=true;
  }

  cout<<"in_cloud->size():"<<in_cloud->size()<<endl;

  vector<int> labels(in_cloud->size());
  for(unsigned int i = 0; i < labels.size(); i++){
    labels[i]=-1;
  }

  for (int i = 0; i < in_cloud->size(); i++)  
  { 
    // Neighbors containers  
    vector<int> k_indices;  
    vector<float> k_distances;  

    for(int j=0; j<trees.size(); j++){
      trees.at(j).radiusSearch (in_cloud->at(i), 0.02, k_indices, k_distances);  

      if(k_indices.size()>0){
        labels[i]=j;
      }
    }
  }


  for (int i = 0; i < in_cloud->size(); i++)  
  {  
    if(i%500==0){
      cout<<"progress:"<<i*100.0/in_cloud->size()<<"%"<<endl;
    }

    if(!flags[i]){
      continue;
    }

    if(labels[i]==-1){
      cloud_tem->push_back(in_cloud->at(i));
      flags[i]=false;
      continue;
    }

    MyPointCloud mpc;

    MyPt mp;
    mp.x=in_cloud->at(i).x;
    mp.y=in_cloud->at(i).y;
    mp.z=in_cloud->at(i).z;

    mpc.mypoints.push_back(mp);
    flags[i]=false;

    for (int j = i + 1; j < in_cloud->size(); j++)  
    {  
      if(!flags[j]){
        continue;
      }

      if(labels[j]==-1){
        cloud_tem->push_back(in_cloud->at(j));
        flags[j]=false;
        continue;
      }

      if(labels[i]==labels[j]&&!isSeparated(in_cloud->at(i), in_cloud->at(j), lines)){
        MyPt mp;
        mp.x=in_cloud->at(j).x;
        mp.y=in_cloud->at(j).y;
        mp.z=in_cloud->at(j).z;

        mpc.mypoints.push_back(mp);
        flags[j]=false;
      }
    }  //  for j  

    if(mpc.mypoints.size()>1){
      clustering_cloud.push_back(mpc);
    }
    else{
      cloud_tem->push_back(in_cloud->at(i));
    }
  }  //  for i  

  pcl::copyPointCloud(*cloud_tem, *in_cloud);

  mean_shift_cluster(in_cloud, lines, radius, scene_scale, clustering_cloud);
}
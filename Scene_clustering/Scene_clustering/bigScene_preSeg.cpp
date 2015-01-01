#include "bigScene_preSeg.h"

//Euclidean Cluster Extraction
void big_object_seg_ECE(PointCloudPtr_RGB cloud, vector<pcl::PointIndices>& cluster_indices){
  cluster_indices.clear();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (5000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
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

    wall_rect_clouds.at(i).mypoints.at(0).x += 0.2*normal_0[0];
    wall_rect_clouds.at(i).mypoints.at(0).y += 0.2*normal_0[1];
    wall_rect_clouds.at(i).mypoints.at(0).z += 0.2*normal_0[2];

    wall_rect_clouds.at(i).mypoints.at(2).x -= 0.2*normal_0[0];
    wall_rect_clouds.at(i).mypoints.at(2).y -= 0.2*normal_0[1];
    wall_rect_clouds.at(i).mypoints.at(2).z -= 0.2*normal_0[2];

    wall_rect_clouds.at(i).mypoints.at(3).x += 0.2*normal_1[0];
    wall_rect_clouds.at(i).mypoints.at(3).y += 0.2*normal_1[1];
    wall_rect_clouds.at(i).mypoints.at(3).z += 0.2*normal_1[2];

    wall_rect_clouds.at(i).mypoints.at(1).x -= 0.2*normal_1[0];
    wall_rect_clouds.at(i).mypoints.at(1).y -= 0.2*normal_1[1];
    wall_rect_clouds.at(i).mypoints.at(1).z -= 0.2*normal_1[2];

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
      if(new_remained_cloud->at(j).y <= rect_cl_t->at(0).y+0.025){
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

//pre-segment scene
void pre_segment_scene(PointCloudPtr_RGB cloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, vector<MyPointCloud_RGB>& cluster_projected_pcs, vector<MyPointCloud_RGB>& cluster_origin_pcs, PointCloudPtr_RGB colored_projected_pc, PointCloudPtr_RGB colored_origin_pc){
  PointCloudPtr_RGB new_cloud(new PointCloud_RGB);
  pcl::transformPointCloud (*cloud, *new_cloud, matrix_transform);

  // All the objects needed
  pcl::PassThrough<Point_RGB> pass;
  // Datasets
  PointCloudPtr_RGB cloud_filtered(new PointCloud_RGB);
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (new_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 2.0);
  pass.filter (*cloud_filtered);

  /******************Euclidean Cluster Extraction************************/
  vector<pcl::PointIndices> cluster_indices;
  big_object_seg_ECE(cloud_filtered, cluster_indices);

  pcl::ExtractIndices<Point_RGB> extract;
  extract.setInputCloud (cloud_filtered);

  PointCloudPtr_RGB cloud_filtered_tem(new PointCloud_RGB);
  for(int i=0; i<cluster_indices.size(); i++){
    extract.setIndices (boost::make_shared<pcl::PointIndices>(cluster_indices.at(i)));
    extract.setNegative (false);
    extract.filter(*cloud_tem);

    float min_x,min_y,min_z, max_x, max_y, max_z;
    com_bounding_box(cloud_tem, &min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

    if(min_z < 1.5){
      appendCloud_RGB(cloud_tem, cloud_filtered_tem);
    } 
  }

  pcl::copyPointCloud(*cloud_filtered_tem, *cloud_filtered);

  PointCloudPtr_RGB projected_cloud(new PointCloud_RGB);
  pcl::copyPointCloud(*cloud_filtered, *projected_cloud);

  for(int i=0; i<projected_cloud->size(); i++){
    projected_cloud->at(i).z = 0;
  }

  big_object_seg_ECE(projected_cloud, cluster_indices);

  extract.setInputCloud (projected_cloud);

  for(int i=0; i<cluster_indices.size(); i++){
    if(cluster_indices.at(i).indices.size() > 500){
      extract.setIndices (boost::make_shared<pcl::PointIndices>(cluster_indices.at(i)));
      extract.setNegative (false);
      extract.filter(*cloud_tem);

      MyPointCloud_RGB mc;
      PointCloud_RGB2MyPointCloud_RGB(cloud_tem, mc);
      cluster_projected_pcs.push_back(mc);

      for(int j=0; j<cloud_tem->size(); j++){
        cloud_tem->at(j).r=new_color_table[i%30][0];
        cloud_tem->at(j).g=new_color_table[i%30][1];
        cloud_tem->at(j).b=new_color_table[i%30][2];
      }

      appendCloud_RGB(cloud_tem, colored_projected_pc);
    }
  }

  extract.setInputCloud (cloud_filtered);

  for(int i=0; i<cluster_indices.size(); i++){
    if(cluster_indices.at(i).indices.size() > 500){
      extract.setIndices (boost::make_shared<pcl::PointIndices>(cluster_indices.at(i)));
      extract.setNegative (false);
      extract.filter(*cloud_tem);

      PointCloud_RGB c_temp;
      pcl::transformPointCloud (*cloud_tem, c_temp, matrix_translation_r);
      pcl::transformPointCloud (c_temp, *cloud_tem, matrix_transform_r);

      MyPointCloud_RGB mc;
      PointCloud_RGB2MyPointCloud_RGB(cloud_tem, mc);
      cluster_origin_pcs.push_back(mc);

      for(int j=0; j<cloud_tem->size(); j++){
        cloud_tem->at(j).r=new_color_table[i%30][0];
        cloud_tem->at(j).g=new_color_table[i%30][1];
        cloud_tem->at(j).b=new_color_table[i%30][2];
      }

      appendCloud_RGB(cloud_tem, colored_origin_pc);
    }
  }
}


//mark cloud by bounding box
void mark_cloud(PointCloudPtr_RGB sourceCloud, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, PointCloudPtr box_cloud){
  PointCloudPtr_RGB new_cloud(new PointCloud_RGB);
  pcl::transformPointCloud (*sourceCloud, *new_cloud, matrix_transform);


  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(new_cloud, p0,p1,p2,p3);

  float min_x,min_y,min_z, max_x, max_y, max_z;

  com_bounding_box(new_cloud, &min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  box_cloud->push_back(Point(p0.x,p0.y,min_z));
  box_cloud->push_back(Point(p1.x,p1.y,min_z));
  box_cloud->push_back(Point(p2.x,p2.y,min_z));
  box_cloud->push_back(Point(p3.x,p3.y,min_z));

  box_cloud->push_back(Point(p0.x,p0.y,max_z));
  box_cloud->push_back(Point(p1.x,p1.y,max_z));
  box_cloud->push_back(Point(p2.x,p2.y,max_z));
  box_cloud->push_back(Point(p3.x,p3.y,max_z));

  PointCloud box_cloud_temp;
  pcl::transformPointCloud (*box_cloud, box_cloud_temp, matrix_translation_r);
  pcl::transformPointCloud (box_cloud_temp, *box_cloud, matrix_transform_r);
}


//compute mean curvature
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

//bubble Sort
void bubbleSort(vector<int>& a, vector<int>& priority_vec)
{
  int temp;
  int size=a.size();

  for(int pass=0; pass<size; pass++)
  {
    priority_vec.push_back(pass);
  }

  for(int pass=1; pass<size; pass++)
  {
    for(int k=0;k<size-pass;k++){ 
      if(a.at(k)>a.at(k+1))
      {
        //cout<<"============"<<endl;

        temp=a.at(k);
        a.at(k)=a.at(k+1);
        a.at(k+1)=temp; 

        temp=priority_vec.at(k);
        priority_vec.at(k)=priority_vec.at(k+1);
        priority_vec.at(k+1)=temp;
      }
    }
  }
}

//Set Priority for Clusters
void setPriorityforClusters(vector<MyPointCloud_RGB>& cluster_projected_pcs, vector<MyPointCloud_RGB>& cluster_origin_pcs ,vector<int>& priority_vec){
  vector<int> num;

  for(int i=0; i<cluster_origin_pcs.size(); i++){
    PointCloudPtr_RGB pc(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(i), pc);

    vector<Point_Cur_RGB> curvatures;
    PointCloudPtr_RGB cloud_colored(new PointCloud_RGB);
    compute_mean_curvature(pc, curvatures, cloud_colored);

    int count=0;
    for(int j=0; j<curvatures.size(); j++){
      if(curvatures.at(j).curvature > 0.1){
        count++;
      }
    }

    num.push_back(count);

    //cout<<"count:"<<count<<endl;

    //showPointCloud(pc, "test");
  }

  bubbleSort(num, priority_vec);
}

//get position that robot should go
void getRobotPosition(PointCloudPtr_RGB sourceCloud, vector<MyPointCloud> &wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, Point& position, Visualizer& vs){
  PointCloudPtr_RGB new_cloud(new PointCloud_RGB);
  pcl::transformPointCloud (*sourceCloud, *new_cloud, matrix_transform);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(new_cloud, p0,p1,p2,p3);

  float min_x,min_y,min_z, max_x, max_y, max_z;

  com_bounding_box(new_cloud, &min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  PointCloudPtr box_cloud(new PointCloud);

  box_cloud->push_back(Point(p0.x,p0.y,min_z));
  box_cloud->push_back(Point(p1.x,p1.y,min_z));
  box_cloud->push_back(Point(p2.x,p2.y,min_z));
  box_cloud->push_back(Point(p3.x,p3.y,min_z));

  box_cloud->push_back(Point(p0.x,p0.y,max_z));
  box_cloud->push_back(Point(p1.x,p1.y,max_z));
  box_cloud->push_back(Point(p2.x,p2.y,max_z));
  box_cloud->push_back(Point(p3.x,p3.y,max_z));

  PointCloud box_cloud_temp;
  pcl::transformPointCloud (*box_cloud, box_cloud_temp, matrix_translation_r);
  pcl::transformPointCloud (box_cloud_temp, *box_cloud, matrix_transform_r);

  draw_box(box_cloud, vs, 0, 255, 255, "box");

  Point p0_bottom = box_cloud->at(0);
  Point p1_bottom = box_cloud->at(1);
  Point p2_bottom = box_cloud->at(2);
  Point p3_bottom = box_cloud->at(3);
  Point p0_top = box_cloud->at(4);
  Point p1_top = box_cloud->at(5);
  Point p2_top = box_cloud->at(6);
  Point p3_top = box_cloud->at(7);

  int first_id=-1;
  int second_id=-1;
  float max=0;
  float second_max=0;

  for(int i=0; i<wall_rect_clouds.size(); i++){
    Eigen::Vector3d normal0;
    normal0 << wall_rect_clouds.at(i).mypoints.at(0).x - wall_rect_clouds.at(i).mypoints.at(1).x, wall_rect_clouds.at(i).mypoints.at(0).y - wall_rect_clouds.at(i).mypoints.at(1).y, wall_rect_clouds.at(i).mypoints.at(0).z - wall_rect_clouds.at(i).mypoints.at(1).z;
    normal0.normalize();

    Eigen::Vector3d normal1;
    normal1 << wall_rect_clouds.at(i).mypoints.at(0).x - wall_rect_clouds.at(i).mypoints.at(3).x, wall_rect_clouds.at(i).mypoints.at(0).y - wall_rect_clouds.at(i).mypoints.at(3).y, wall_rect_clouds.at(i).mypoints.at(0).z - wall_rect_clouds.at(i).mypoints.at(3).z;
    normal1.normalize();

    Eigen::Vector3d wall_normal = normal0.cross(normal1);
    wall_normal.normalize();

    float A = wall_normal[0];
    float B = wall_normal[1];
    float C = wall_normal[2];
    float D = -(A*wall_rect_clouds.at(i).mypoints.at(0).x+B*wall_rect_clouds.at(i).mypoints.at(0).y+C*wall_rect_clouds.at(i).mypoints.at(0).z);

    cout<<"A=====:"<<A<<endl;
    cout<<"B=====:"<<B<<endl;
    cout<<"C=====:"<<C<<endl;
    cout<<"D=====:"<<D<<endl;

    cout<<"std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D):"<<std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D)<<endl;
    cout<<"std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D):"<<std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D)<<endl;
    cout<<"std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D):"<<std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D)<<endl;
    cout<<"std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D):"<<std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D)<<endl;

    if(max < std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D)){
      second_max = max;
      second_id = first_id;
      
      max = std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D);
      first_id = 0;
    }
    else if(second_max < std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D)){
      second_max = std::abs(A*p0_bottom.x+B*p0_bottom.y+C*p0_bottom.z+D);
      second_id = 0;
    }

    cout<<"max:"<<max<<endl;

    if(max < std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D)){
      second_max = max;
      second_id = first_id;

      max = std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D);
      first_id = 1;
    }
    else if(second_max < std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D)){
      second_max = std::abs(A*p1_bottom.x+B*p1_bottom.y+C*p1_bottom.z+D);
      second_id = 1;
    }

    cout<<"max:"<<max<<endl;

    if(max < std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D)){
      second_max = max;
      second_id = first_id;

      max = std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D);
      first_id = 2;
    }
    else if(second_max < std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D)){
      second_max = std::abs(A*p2_bottom.x+B*p2_bottom.y+C*p2_bottom.z+D);
      second_id = 2;
    }

    cout<<"max:"<<max<<endl;

    if(max < std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D)){
      second_max = max;
      second_id = first_id;

      max = std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D);
      first_id = 3;
    }
    else if(second_max < std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D)){
      second_max = std::abs(A*p3_bottom.x+B*p3_bottom.y+C*p3_bottom.z+D);
      second_id = 3;
    }

    cout<<"max:"<<max<<endl;
  }

  int first_id_r=0;
  int second_id_r=0;

  switch(first_id){
  case 0:
    switch(second_id){
    case 1:
      first_id_r=3;
      second_id_r=2;
      break;
    case 3:
      first_id_r=1;
      second_id_r=2;
      break;

    }
    break;
  case 1:
    switch(second_id){
    case 0:
      first_id_r=2;
      second_id_r=3;
      break;
    case 2:
      first_id_r=0;
      second_id_r=3;
      break;

    }
    break;
  case 2:
    switch(second_id){
    case 1:
      first_id_r=3;
      second_id_r=0;
      break;
    case 3:
      first_id_r=1;
      second_id_r=0;
      break;

    }
    break;
  case 3:
    switch(second_id){
    case 0:
      first_id_r=2;
      second_id_r=1;
      break;
    case 2:
      first_id_r=0;
      second_id_r=1;
      break;

    }
    break;
  }


  cout<<"first_id=============:"<<first_id<<endl;
  cout<<"second_id=============:"<<second_id<<endl;
  cout<<"first_id_r=============:"<<first_id_r<<endl;
  cout<<"second_id_r=============:"<<second_id_r<<endl;

  vs.viewer->addSphere(box_cloud->at(first_id), 0.025, "1");
  vs.viewer->addSphere(box_cloud->at(second_id), 0.05, "2");

  vs.viewer->addSphere(box_cloud->at(first_id_r), 0.1, "3");
  vs.viewer->addSphere(box_cloud->at(second_id_r), 0.15, "4");

  Eigen::Vector3d r_normal0;
  r_normal0 << box_cloud->at(first_id).x - box_cloud->at(first_id_r).x, box_cloud->at(first_id).y - box_cloud->at(first_id_r).y, box_cloud->at(first_id).z - box_cloud->at(first_id_r).z;
  r_normal0.normalize();

  Point pos((box_cloud->at(first_id).x+box_cloud->at(second_id).x)/2, (box_cloud->at(first_id).y+box_cloud->at(second_id).y)/2, (box_cloud->at(first_id).z+box_cloud->at(second_id).z)/2);
  
  vs.viewer->addSphere(pos, 0.175, "5");

  position.x = pos.x + 0.4*r_normal0[0];
  position.y = pos.y + 0.4*r_normal0[1];
  position.z = pos.z + 0.4*r_normal0[2];

  vs.viewer->addSphere(position, 0.2, "6");
}

//get position that robot should go
void getRobotPosition1(PointCloudPtr_RGB sourceCloud, vector<MyPointCloud> &wall_rect_clouds, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_translation_r, Eigen::Matrix4f& matrix_transform_r, Point& position, Visualizer& vs){
  PointCloudPtr_RGB new_cloud(new PointCloud_RGB);
  pcl::transformPointCloud (*sourceCloud, *new_cloud, matrix_transform);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(new_cloud, p0,p1,p2,p3);

  float min_x,min_y,min_z, max_x, max_y, max_z;

  com_bounding_box(new_cloud, &min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  PointCloudPtr box_cloud(new PointCloud);

  box_cloud->push_back(Point(p0.x,p0.y,min_z));
  box_cloud->push_back(Point(p1.x,p1.y,min_z));
  box_cloud->push_back(Point(p2.x,p2.y,min_z));
  box_cloud->push_back(Point(p3.x,p3.y,min_z));

  box_cloud->push_back(Point(p0.x,p0.y,max_z));
  box_cloud->push_back(Point(p1.x,p1.y,max_z));
  box_cloud->push_back(Point(p2.x,p2.y,max_z));
  box_cloud->push_back(Point(p3.x,p3.y,max_z));

  PointCloud box_cloud_temp;
  pcl::transformPointCloud (*box_cloud, box_cloud_temp, matrix_translation_r);
  pcl::transformPointCloud (box_cloud_temp, *box_cloud, matrix_transform_r);

  draw_box(box_cloud, vs, 0, 255, 255, "box");

  int first_id0=-1;
  int second_id0=-1;
  int first_id1=-1;
  int second_id1=-1;

  float len0 = sqrt(pow(box_cloud->at(0).x-box_cloud->at(1).x, 2)+pow(box_cloud->at(0).y-box_cloud->at(1).y, 2)+pow(box_cloud->at(0).z-box_cloud->at(1).z, 2));
  float len1 = sqrt(pow(box_cloud->at(0).x-box_cloud->at(3).x, 2)+pow(box_cloud->at(0).y-box_cloud->at(3).y, 2)+pow(box_cloud->at(0).z-box_cloud->at(3).z, 2));

  if(len0 > len1){
    first_id0 = 0;
    second_id0 = 1;
    first_id1 = 3;
    second_id1 = 2;
  }
  else{
    first_id0 = 0;
    second_id0 = 3;
    first_id1 = 1;
    second_id1 = 4;
  }


  float sum_dis0 = sqrt(pow(box_cloud->at(first_id0).x, 2)+pow(box_cloud->at(first_id0).y, 2)+pow(box_cloud->at(first_id0).z, 2)) + sqrt(pow(box_cloud->at(second_id0).x, 2)+pow(box_cloud->at(second_id0).y, 2)+pow(box_cloud->at(second_id0).z, 2));
  float sum_dis1 = sqrt(pow(box_cloud->at(first_id1).x, 2)+pow(box_cloud->at(first_id1).y, 2)+pow(box_cloud->at(first_id1).z, 2)) + sqrt(pow(box_cloud->at(second_id1).x, 2)+pow(box_cloud->at(second_id1).y, 2)+pow(box_cloud->at(second_id1).z, 2));
  
  Eigen::Vector3d r_normal0;

  int first_id = -1;
  int second_id = -1;

  if(sum_dis0 < sum_dis1){
    first_id = first_id0;
    second_id = second_id0;

    r_normal0 << box_cloud->at(first_id0).x - box_cloud->at(first_id1).x, box_cloud->at(first_id0).y - box_cloud->at(first_id1).y, box_cloud->at(first_id0).z - box_cloud->at(first_id1).z;
    r_normal0.normalize();
  }
  else{
    first_id = first_id1;
    second_id = second_id1;

    r_normal0 << box_cloud->at(first_id1).x - box_cloud->at(first_id0).x, box_cloud->at(first_id1).y - box_cloud->at(first_id0).y, box_cloud->at(first_id1).z - box_cloud->at(first_id0).z;
    r_normal0.normalize();
  }
  
  Point pos((box_cloud->at(first_id).x+box_cloud->at(second_id).x)/2, (box_cloud->at(first_id).y+box_cloud->at(second_id).y)/2, (box_cloud->at(first_id).z+box_cloud->at(second_id).z)/2);

  vs.viewer->addSphere(pos, 0.15, "0");

  position.x = pos.x + 0.4*r_normal0[0];
  position.y = pos.y + 0.4*r_normal0[1];
  position.z = pos.z + 0.4*r_normal0[2];

  vs.viewer->addSphere(position, 0.15, "1");
}
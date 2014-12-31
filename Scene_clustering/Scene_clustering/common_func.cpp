#include "common_func.h"

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

  pointCloud_RGBPopUp(cloud_in_plane);

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
void samplePlane(MyPointCloud& rect_mpc, float grid_length, MyPointCloud& sample_mpt, int *rows, int *cols){
  sample_mpt.mypoints.clear();

  float w=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(1).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(1).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(1).z, 2));
  float h=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(3).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(3).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(3).z, 2));

  int num_w=(int)(w/grid_length);
  int num_h=(int)(h/grid_length);

  *rows=num_w;
  *cols=num_h;

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

//sample cylinder
void sampleCylinder(Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, MyPointCloud& mpt){
  float height=sqrt(pow(cenPoint0.x-cenPoint1.x, 2)+pow(cenPoint0.y-cenPoint1.y, 2)+pow(cenPoint0.z-cenPoint1.z, 2));

  int num_cir=(int)((2*PI*r)/grid_length);
  int num_h=(int)(height/grid_length);

  Eigen::Vector3d direction_normal;
  direction_normal << direction[0], direction[1], direction[2];
  direction_normal.normalize();

  double angle0=acos(direction_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis0=direction_normal.cross(Eigen::Vector3d(0,0,1));
  axis0.normalize();

  Eigen::Matrix4d matrix0;
  getRotationMatrix(axis0, angle0, matrix0);

  PointCloud cloud;
  cloud.push_back(cenPoint0);
  cloud.push_back(cenPoint1);

  PointCloudPtr cloud_up(new PointCloud);
  PointCloudPtr cloud_temp(new PointCloud);

  Eigen::Matrix4f matrix_transform0 = matrix0.cast<float>();
  pcl::copyPointCloud(cloud,*cloud_temp);
  pcl::transformPointCloud (*cloud_temp, cloud, matrix_transform0);

  Point firstPoint0(cloud.at(0).x,cloud.at(0).y+r,cloud.at(0).z);
  Point firstPoint1(cloud.at(1).x,cloud.at(1).y+r,cloud.at(1).z);

  float ang=grid_length/r;
  int pos_neg_flag=1;

  if(firstPoint0.z>firstPoint1.z){
    pos_neg_flag=-1;
  }

  for(int i=0;i<num_cir;i++){
    float new_x=0;
    float new_y=0;
    rotatePoint2ByPoint2(firstPoint0.x,firstPoint0.y,cloud.at(0).x,cloud.at(0).y,ang*i,&new_x,&new_y);
    for(int j=0;j<num_h;j++){
      cloud_up->push_back(Point(new_x,new_y,cloud.at(0).z+pos_neg_flag*(j+1)*grid_length));
    }
  }

  Eigen::Matrix4d matrix1;
  getRotationMatrix(axis0, -angle0, matrix1);

  Eigen::Matrix4f matrix_transform1 = matrix1.cast<float>();
  pcl::copyPointCloud(*cloud_up,*cloud_temp);
  pcl::transformPointCloud (*cloud_temp, *cloud_up, matrix_transform1);


  PointCloud2MyPointCloud(cloud_up, mpt);
}

//sample sphere
void sampleSphere(Point cenPoint, float r, float grid_length, MyPointCloud& mpt){

  int woof_num=(int)((PI*r)/grid_length);

  PointCloudPtr sample_clound(new PointCloud);
  PointCloudPtr cloud_woof_points(new PointCloud);
  PointCloudPtr cloud_cen_points(new PointCloud);

  Point startPoint(cenPoint.x,cenPoint.y,cenPoint.z+r);

  sample_clound->push_back(startPoint);

  /*Eigen::Vector3d normal0;
  normal0 << 0, 0, r;
  normal0.normalize();

  Eigen::Vector3d normal1;
  normal1 << 0, 1, 0;
  normal1.normalize();*/

  float ang=grid_length/r;

  for(int i=0;i<woof_num;i++){
    float new_z=0;
    float new_x=0;
    rotatePoint2ByPoint2(startPoint.z,startPoint.x,cenPoint.z,cenPoint.x,ang*(i+1),&new_z,&new_x);

    cloud_woof_points->push_back(Point(new_x,cenPoint.y,new_z));
    cloud_cen_points->push_back(Point(cenPoint.x,cenPoint.y,new_z));
  }

  for(int i=0;i<woof_num;i++){
    Point cen_cir(cloud_cen_points->at(i).x,cloud_cen_points->at(i).y,cloud_cen_points->at(i).z);
    Point first_point(cloud_woof_points->at(i).x,cloud_woof_points->at(i).y,cloud_woof_points->at(i).z);
    float cir_r=sqrt(pow(first_point.x-cen_cir.x, 2)+pow(first_point.y-cen_cir.y, 2)+pow(first_point.z-cen_cir.z, 2));
    int num=(int)((2*PI*cir_r)/grid_length);
    float ang_tem=grid_length/cir_r;

    for(int j=0;j<num;j++){
      float new_x=0;
      float new_y=0;
      rotatePoint2ByPoint2(first_point.x,first_point.y,cen_cir.x,cen_cir.y,ang_tem*j,&new_x,&new_y);

      sample_clound->push_back(Point(new_x,new_y,cen_cir.z));
    }
  }

  PointCloud2MyPointCloud(sample_clound, mpt);
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
  int rows=0;
  int cols=0;
  samplePlane(rect_mpc, grid_length, sample_mpc, &rows, &cols);

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

//compute Cylinder Jaccard Index
void computeCylinderJaccardIndex(MyPointCloud& source_mpc, Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, float *result){
  MyPointCloud sample_mpc;
  sampleCylinder(cenPoint0, cenPoint1, direction,r,grid_length,sample_mpc);

  /*PointCloudPtr pc(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc,pc);
  showPointCloud2(pc,"cylinder");*/

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_cylinder.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_cylinder.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_cylinder:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_cylinder>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  if(rate>0.05){
    computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);
  }
  else{
    *result=0;
  }
}

//compute Sphere Jaccard Index
void computeSphereJaccardIndex(MyPointCloud& source_mpc, Point cenPoint, float r, float grid_length, float *result){
  MyPointCloud sample_mpc;
  sampleSphere(cenPoint, r, grid_length, sample_mpc);

  /*PointCloudPtr pc(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc,pc);
  showPointCloud2(pc,"sphere");*/

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_sphere.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_sphere.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_sphere:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_sphere>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);

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

//append a cloud to another cloud
void appendCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL sourceCloud,PointCloudPtr_RGB_NORMAL targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//rotate a 2d point by a 2d point
void rotatePoint2ByPoint2(float p_x,float p_y,float cen_x,float cen_y,float ang,float *new_x,float *new_y){
  *new_x=cen_x+(p_x-cen_x)*cos(ang)-(p_y-cen_y)*sin(ang);
  *new_y=cen_y+(p_x-cen_x)*sin(ang)+(p_y-cen_y)*cos(ang);
}

//pcl pointCloud pop up
void pointCloud_RGBPopUp(PointCloudPtr_RGB cloud){
  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<cloud->size()-1;i++){
    pc->push_back(cloud->at(i));
  }

  cloud->clear();

  pcl::copyPointCloud(*pc,*cloud);
}

//pcl pointCloud pop up
void pointCloud_RGB_NORMAPopUp(PointCloudPtr_RGB_NORMAL cloud){
  PointCloudPtr_RGB_NORMAL pc(new PointCloud_RGB_NORMAL);

  for(int i=0;i<cloud->size()-1;i++){
    pc->push_back(cloud->at(i));
  }

  cloud->clear();

  pcl::copyPointCloud(*pc,*cloud);
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

//Get Color By Value
void getColorByValue(float val, float min, float max, float *r, float *g, float *b)
{
  int tmp = static_cast<int>((val - min) / (max - min) * 255);
  *r = g_color_table[tmp][0] * 255;
  *g = g_color_table[tmp][1] * 255;
  *b = g_color_table[tmp][2] * 255;
}
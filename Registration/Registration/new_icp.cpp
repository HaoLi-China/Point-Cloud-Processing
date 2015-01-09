#include "new_icp.h"
#include "SparseICP.h"
////transform should be 3*4 matrix
//void computeICPNoNormal(CMesh *moving_mesh, CMesh *static_mesh, CMesh *noised)
//{
//  cout<<"******************************Begin SICP***********************"<<endl;
//  Eigen::MatrixXd SrCloud;
//  Eigen::MatrixXd TgCloud;
//  Eigen::MatrixXd noisedSrcCloud;
//  Eigen::MatrixXd verterMap;//点之间的对应
//
//  const int srVerNum = moving_mesh->vert.size(); 
//  const int tgVerNum = static_mesh->vert.size();
//  SrCloud.resize(3,srVerNum);
//  TgCloud.resize(3,tgVerNum);
//  verterMap.resize(1,srVerNum);
//
//  for(int i = 0; i < srVerNum; i++){
//    CVertex &v = moving_mesh->vert[i];
//    SrCloud(0,i) = v.P()[0];
//    SrCloud(1,i) = v.P()[1];
//    SrCloud(2,i) = v.P()[2];
//  }
//
//  for(int i = 0; i < tgVerNum; i++){
//    CVertex &v = static_mesh->vert[i];
//    TgCloud(0,i) = v.P()[0];
//    TgCloud(1,i) = v.P()[1];
//    TgCloud(2,i) = v.P()[2];
//  }
//
//  int noiseVerNum = 0;
//  if(noised != NULL){
//    noiseVerNum = noised->vert.size();
//    noisedSrcCloud.resize(3, noiseVerNum);
//    for(int i = 0; i < noiseVerNum; ++i)
//    {
//      CVertex &v = noised->vert[i];
//      noisedSrcCloud(0, i) = v.P()[0];
//      noisedSrcCloud(1, i) = v.P()[1];
//      noisedSrcCloud(2, i) = v.P()[2];
//    }
//  }
//
//  double error;
//  SparseICP::SICP::Parameters pa;
//  SparseICP::SICP::point_to_point(SrCloud, TgCloud, verterMap, noisedSrcCloud, error, pa);
//  //update the moved points
//  for(int i = 0; i < srVerNum; i++)
//  {
//    CVertex& v = moving_mesh->vert[i];
//    v.P()[0] = SrCloud(0,i);
//    v.P()[1] = SrCloud(1,i);
//    v.P()[2] = SrCloud(2,i);
//  }
//
//  if(noised != NULL){
//    for(int i = 0; i < noiseVerNum; ++i){
//      CVertex &v = noised->vert[i];
//      v.P()[0] = noisedSrcCloud(0, i);
//      v.P()[1] = noisedSrcCloud(1, i);
//      v.P()[2] = noisedSrcCloud(2, i);
//    }
//  }
//  cout<<"******************************End SICP***********************"<<endl;
//}
//
//void GlobalFun::computeICPNoNormal(CMesh *moving_mesh, CMesh *static_mesh, double &error)
//{
//  cout<<"******************************Begin SICP GETTING ERROR******************"<<endl;
//  Eigen::MatrixXd SrCloud;
//  Eigen::MatrixXd TgCloud;
//  Eigen::MatrixXd noisedSrcCloud;
//  Eigen::MatrixXd verterMap;//点之间的对应
//
//  const int srVerNum = moving_mesh->vert.size(); 
//  const int tgVerNum = static_mesh->vert.size();
//  SrCloud.resize(3,srVerNum);
//  TgCloud.resize(3,tgVerNum);
//  verterMap.resize(1,srVerNum);
//
//  for(int i = 0; i < srVerNum; i++){
//    CVertex &v = moving_mesh->vert[i];
//    SrCloud(0,i) = v.P()[0];
//    SrCloud(1,i) = v.P()[1];
//    SrCloud(2,i) = v.P()[2];
//  }
//
//  for(int i = 0; i < tgVerNum; i++){
//    CVertex &v = static_mesh->vert[i];
//    TgCloud(0,i) = v.P()[0];
//    TgCloud(1,i) = v.P()[1];
//    TgCloud(2,i) = v.P()[2];
//  }
//
//  SparseICP::SICP::Parameters pa;
//  pa.max_icp = 30;
//  pa.use_penalty = true;
//  pa.max_inner = 2; //use ALM
//  clock_t start, finish;
//  start = clock();
//  SparseICP::SICP::point_to_point(SrCloud, TgCloud, verterMap, noisedSrcCloud, error, pa);
//  finish = clock();
//  double duration = double (finish - start) / CLOCKS_PER_SEC;
//  cout<<"Compute ICP used: "<<duration <<endl;
//  //update the moved points
//  for(int i = 0; i < srVerNum; i++)
//  {
//    CVertex& v = moving_mesh->vert[i];
//    v.P()[0] = SrCloud(0,i);
//    v.P()[1] = SrCloud(1,i);
//    v.P()[2] = SrCloud(2,i);
//  }
//  cout<<"pa.alpha: " <<pa.alpha<<endl;
//  cout<<"pa.max_icp: " <<pa.max_icp<<endl;
//  cout<<"pa.max_inner: " <<pa.max_inner<<endl;
//  cout<<"pa.max_mu: "<<pa.max_mu<<endl;
//  cout<<"pa.max_outer: " <<pa.max_outer<<endl;
//  cout<<"pa.p: " <<pa.p<<endl;
//  cout<<"pa.stop"<<pa.stop<<endl;
//  cout<<"pa.use_penalty: " <<pa.use_penalty<<endl;
//  cout<<"******************************End SICP GETTING ERROR******************"<<endl;
//}

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

bool loadPointCloud_ply(char* fileName, PointCloudPtr_RGB cloud){

  std::ifstream input(fileName) ;
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return false;
  }

  int num_header = 14;
  int num_points = 0;
  // this line contains point number
  int line_num = 3;

  for (int i=0; i<num_header; ++i) {
    std::string line;
    getline(input, line);

    if (i==line_num) {
      std::istringstream line_input(line);
      std::string dummy1;
      std::string dummy2;
      line_input >> dummy1 >> dummy2 >>num_points;

      printf("num_points:%d\n",num_points);
    }
  }

  std::cout<< "===========================" <<std::endl;

  for (int i=0; i<num_points; ++i) {
    Point_RGB point_tem;
    int alpha=0;
    int r,g,b;

    input >> point_tem.x >> point_tem.y >> point_tem.z >> r >> g >> b >> alpha;

    point_tem.r=r;
    point_tem.g=g;
    point_tem.b=b;

    cloud->push_back(point_tem);
  }

  return true;
}

bool loadPointCloud_normal_ply(char* fileName, PointCloudPtr_RGB_NORMAL cloud){

  std::ifstream input(fileName) ;
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return false;
  }

  int num_header = 17;
  int num_points = 0;
  // this line contains point number
  int line_num = 3;

  for (int i=0; i<num_header; ++i) {
    std::string line;
    getline(input, line);

    if (i==line_num) {
      std::istringstream line_input(line);
      std::string dummy1;
      std::string dummy2;
      line_input >> dummy1 >> dummy2 >>num_points;

      printf("num_points:%d\n",num_points);
    }
  }

  std::cout<< "===========================" <<std::endl;

  for (int i=0; i<num_points; ++i) {
    Point_RGB_NORMAL point_tem;
    int alpha=0;
    int r,g,b;

    input >> point_tem.x >> point_tem.y >> point_tem.z >>point_tem.normal_x>>point_tem.normal_y>>point_tem.normal_z>>r >> g >> b >> alpha;

    point_tem.r=r;
    point_tem.g=g;
    point_tem.b=b;

    cloud->push_back(point_tem);
  }

  return true;
}


//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}


//show rgb cloud
void showPointCloud_RGB_NORMAL (PointCloudPtr_RGB_NORMAL cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<cloud->size();i++){
    Point_RGB pr;
    pr.x=cloud->at(i).x;
    pr.y=cloud->at(i).y;
    pr.z=cloud->at(i).z;
    pr.r=cloud->at(i).r;
    pr.g=cloud->at(i).g;
    pr.b=cloud->at(i).b;
    pc->push_back(pr);
  }

  viewer.showCloud (pc);
  while (!viewer.wasStopped ())
  {

  }
}


//append a cloud to another cloud
void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//append a cloud to another cloud
void appendCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL sourceCloud,PointCloudPtr_RGB_NORMAL targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

void computerICPWithNormal(PointCloudPtr_RGB_NORMAL cloud_in0, PointCloudPtr_RGB_NORMAL cloud_in1, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations){
  cloud_out->clear();

  cout<<"******************************Begin SICP USING NORMAL***********************"<<endl;
  Eigen::Matrix3Xd SrCloud;
  Eigen::Matrix3Xd TgCloud;
  Eigen::Matrix3Xd NormCloud;//the normal of target cloud

  const int srVerNum = cloud_in1->size(); 
  const int tgVerNum = cloud_in0->size();
  SrCloud.resize(3,srVerNum);
  TgCloud.resize(3,tgVerNum);
  NormCloud.resize(3, tgVerNum);

  for(int i = 0; i < srVerNum; i++){
    Point_RGB_NORMAL &p = cloud_in1->at(i);

    SrCloud(0,i) = p.x;
    SrCloud(1,i) = p.y;
    SrCloud(2,i) = p.z;
  }

  for(int i = 0; i < tgVerNum; i++){
    Point_RGB_NORMAL &p = cloud_in0->at(i);

    TgCloud(0,i) = p.x;
    TgCloud(1,i) = p.y;
    TgCloud(2,i) = p.z;

    NormCloud(0, i) = p.normal_x;
    NormCloud(1, i) = p.normal_y;
    NormCloud(2, i) = p.normal_z;
  }

  SparseICP::SICP::Parameters pa;
  //  pa.max_icp = 30;
  //  pa.use_penalty = true;
  //  pa.max_inner = 2; //use ALM

  //Eigen::Matrix3Xf SrCloud2 = SrCloud.base().cast<float>();
  //Eigen::Matrix3Xf TgCloud2  = TgCloud.base().cast<float>();
  //Eigen::Matrix3Xf NormCloud2 = NormCloud.base().cast<float>();//the normal of target cloud
  SparseICP::SICP::point_to_plane(SrCloud, TgCloud, NormCloud, pa);
  //update the moved points
  for(int i = 0; i < srVerNum; i++)
  {
    cloud_in1->at(i).x = SrCloud(0,i);
    cloud_in1->at(i).y = SrCloud(1,i);
    cloud_in1->at(i).z = SrCloud(2,i);
  }

  appendCloud_RGB_NORMAL(cloud_in0, cloud_out);
  appendCloud_RGB_NORMAL(cloud_in1, cloud_out);
  cout<<"******************************End SICP USING NORMAL***********************"<<endl;
}

void computeIcpPCL(PointCloudPtr_RGB cloud_in0, PointCloudPtr_RGB cloud_in1, PointCloudPtr_RGB cloud_out, int iterations, int max_iterations){
  cloud_out->clear();

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // The Iterative Closest Point algorithm
  pcl::IterativeClosestPoint<Point_RGB, Point_RGB> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_in1);
  icp.setInputTarget (cloud_in0);
  icp.align (*cloud_in1);
  icp.setMaximumIterations (max_iterations); 

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return;
  }

  appendCloud_RGB(cloud_in0, cloud_out);
  appendCloud_RGB(cloud_in1, cloud_out);
}

void meargePointCloudsNoNormal(vector<MyPointCloud_RGB> clouds, PointCloudPtr_RGB cloud_out, int iterations, int max_iterations){
  cloud_out->clear();

  PointCloudPtr_RGB cloud_out_tem(new PointCloud_RGB);

  if(clouds.size()==1){
    MyPointCloud_RGB2PointCloud_RGB(clouds.at(0), cloud_out);
    return;
  }

  if(clouds.size()==2){
    PointCloudPtr_RGB cloud_in0 (new PointCloud_RGB);
    PointCloudPtr_RGB cloud_in1 (new PointCloud_RGB); 
    MyPointCloud_RGB2PointCloud_RGB(clouds.at(0), cloud_in0);
    MyPointCloud_RGB2PointCloud_RGB(clouds.at(1), cloud_in1);

    computeIcpPCL(cloud_in0, cloud_in1, cloud_out, iterations, max_iterations);
    return;
  }

  if(clouds.size()>2){
    PointCloudPtr_RGB cloud_in0 (new PointCloud_RGB);
    PointCloudPtr_RGB cloud_in1 (new PointCloud_RGB); 
    MyPointCloud_RGB2PointCloud_RGB(clouds.at(0), cloud_in0);
    MyPointCloud_RGB2PointCloud_RGB(clouds.at(1), cloud_in1);

    computeIcpPCL(cloud_in0, cloud_in1, cloud_out_tem, iterations, max_iterations);
    cout<<"0"<<endl;
    cout<<"1"<<endl;

    for(int i=2; i<clouds.size(); i++){
      cout<<i<<endl;
      PointCloudPtr_RGB cloud_tem (new PointCloud_RGB); 
      MyPointCloud_RGB2PointCloud_RGB(clouds.at(i), cloud_tem);

      computeIcpPCL(cloud_out_tem, cloud_tem, cloud_out, iterations, max_iterations);

      pcl::copyPointCloud(*cloud_out, *cloud_out_tem);
    }
  }
}

//void meargePointCloudsWithNormal(vector<MyPointCloud_RGB_NORMAL> clouds, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations){
//  cloud_out->clear();
//
//  PointCloudPtr_RGB_NORMAL cloud_out_tem(new PointCloud_RGB_NORMAL);
//
//  if(clouds.size()==1){
//    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_out);
//    return;
//  }
//
//  if(clouds.size()==2){
//    PointCloudPtr_RGB_NORMAL cloud_in0 (new PointCloud_RGB_NORMAL);
//    PointCloudPtr_RGB_NORMAL cloud_in1 (new PointCloud_RGB_NORMAL); 
//    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_in0);
//    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(1), cloud_in1);
//
//    computerICPWithNormal(cloud_in1, cloud_in0, cloud_out, iterations, max_iterations);
//    return;
//  }
//
//  if(clouds.size()>2){
//    PointCloudPtr_RGB_NORMAL cloud_in0 (new PointCloud_RGB_NORMAL);
//    PointCloudPtr_RGB_NORMAL cloud_in1 (new PointCloud_RGB_NORMAL); 
//    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_in0);
//    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(1), cloud_in1);
//
//    computerICPWithNormal(cloud_in1, cloud_in0, cloud_out_tem, iterations, max_iterations);
//    cout<<"0"<<endl;
//    cout<<"1"<<endl;
//
//    for(int i=2; i<clouds.size(); i++){
//      cout<<i<<endl;
//      PointCloudPtr_RGB_NORMAL cloud_tem (new PointCloud_RGB_NORMAL); 
//      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(i), cloud_tem);
//
//      computerICPWithNormal(cloud_tem, cloud_out_tem, cloud_out, iterations, max_iterations);
//
//      pcl::copyPointCloud(*cloud_out, *cloud_out_tem);
//    }
//  }
//}


void meargePointCloudsWithNormal(vector<MyPointCloud_RGB_NORMAL> clouds, PointCloudPtr_RGB_NORMAL cloud_out, int iterations, int max_iterations){
  cloud_out->clear();


  vector<Eigen::Matrix4f> mats;

  PointCloudPtr_RGB_NORMAL cloud_out_tem(new PointCloud_RGB_NORMAL);

  if(clouds.size()==1){
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_out);
    return;
  }

  if(clouds.size()==2){
    PointCloudPtr_RGB_NORMAL cloud_in0 (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL cloud_in1 (new PointCloud_RGB_NORMAL); 
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_in0);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(1), cloud_in1);

    computerICPWithNormal(cloud_in1, cloud_in0, cloud_out, iterations, max_iterations);
    return;
  }

  if(clouds.size()>2){
    PointCloudPtr_RGB_NORMAL cloud_pre (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL cloud_next (new PointCloud_RGB_NORMAL); 
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(0), cloud_pre);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(1), cloud_next);

    PointCloudPtr_RGB_NORMAL cloud_pre_tem (new PointCloud_RGB_NORMAL);
    pcl::copyPointCloud(*cloud_pre, *cloud_pre_tem);

    computerICPWithNormal(cloud_next, cloud_pre, cloud_out_tem, iterations, max_iterations);

    Eigen::Matrix4d mat_d;
    Eigen::Vector3d normal0;
    normal0 << cloud_pre_tem->at(0).x, cloud_pre_tem->at(0).y, cloud_pre_tem->at(0).z;
    normal0.normalize();
    Eigen::Vector3d normal1;
    normal1 << cloud_pre->at(0).x, cloud_pre->at(0).y, cloud_pre->at(0).z;
    normal1.normalize();

    double angle=acos(normal0.dot(normal1));
    Eigen::Vector3d axis=normal0.cross(normal1);
    axis.normalize();

    getRotationMatrix(axis, angle, mat_d);

    Eigen::Matrix4f mat_f = mat_d.cast<float>();

    mats.push_back(mat_f);

    cout<<"0"<<endl;
    cout<<"1"<<endl;

    for(int i=2; i<clouds.size(); i++){
      cout<<i<<endl;
      pcl::copyPointCloud(*cloud_next, *cloud_pre);
      pcl::copyPointCloud(*cloud_pre, *cloud_pre_tem);

      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(i), cloud_next);

      computerICPWithNormal(cloud_next, cloud_pre, cloud_out_tem, iterations, max_iterations);

      Eigen::Matrix4d mat_d1;
      Eigen::Vector3d normal01;
      normal01 << cloud_pre_tem->at(0).x, cloud_pre_tem->at(0).y, cloud_pre_tem->at(0).z;
      normal01.normalize();
      Eigen::Vector3d normal11;
      normal11 << cloud_pre->at(0).x, cloud_pre->at(0).y, cloud_pre->at(0).z;
      normal11.normalize();

      double angle1=acos(normal01.dot(normal11));
      Eigen::Vector3d axis1=normal01.cross(normal11);
      axis.normalize();

      getRotationMatrix(axis1, angle1, mat_d1);

      Eigen::Matrix4f mat_f1 = mat_d1.cast<float>();

      mats.push_back(mat_f1);
    }
    
    PointCloudPtr_RGB_NORMAL cloud_tem(new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL cloud_tem1(new PointCloud_RGB_NORMAL);
   
    for(int i=0; i<clouds.size()-1; i++){
      
      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(i), cloud_tem);

      Eigen::Matrix4f mat_tem = mats.at(i);

      for(int j=i+1; j<mats.size(); j++){
        mat_tem = mat_tem*mats.at(j);
      }
      
      //showPointCloud_RGB_NORMAL(cloud_tem, "cloud_tem");

      pcl::transformPointCloud(*cloud_tem, *cloud_tem1, mat_tem);

      //showPointCloud_RGB_NORMAL(cloud_tem1, "cloud_tem1");

      appendCloud_RGB_NORMAL(cloud_tem1, cloud_out);
  }

    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(clouds.at(clouds.size()-1), cloud_tem);
    appendCloud_RGB_NORMAL(cloud_tem, cloud_out);
  }
}
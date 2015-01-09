//#include <pcl/console/parse.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/conversions.h>
//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_rejection_distance.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//
////typedef pcl::PointXYZ Point;
////typedef pcl::PointCloud<Point> PointCloud;
////typedef PointCloud::Ptr PointCloudPtr;
////typedef PointCloud::ConstPtr PointCloudConstPtr;
////
////typedef pcl::PointXYZRGB Point_RGB;
////typedef pcl::PointCloud<Point_RGB> PointCloud_RGB;
////typedef PointCloud_RGB::Ptr PointCloudPtr_RGB;
////typedef PointCloud_RGB::ConstPtr PointCloudConstPtr_RGB;
////
////typedef pcl::PointXYZRGBA PointT;
////typedef pcl::PointCloud<PointT> PointCloudT;
////typedef pcl::PointNormal PointNT;
////typedef pcl::PointCloud<PointNT> PointNCloudT;
////typedef pcl::PointXYZL PointLT;
////typedef pcl::PointCloud<PointLT> PointLCloudT;
////
////typedef pcl::Normal Normal;
////typedef pcl::PointCloud<Normal> NormalCloudT;
////typedef NormalCloudT::Ptr NormalCloudTPtr;
////typedef NormalCloudT::ConstPtr NormalCloudTConstPtr;
////
////typedef pcl::PointXYZRGBNormal Point_RGB_NORMAL;
////typedef pcl::PointCloud<Point_RGB_NORMAL> PointCloud_RGB_NORMAL;
////typedef PointCloud_RGB_NORMAL::Ptr PointCloudPtr_RGB_NORMAL;
////typedef PointCloud_RGB_NORMAL::ConstPtr PointCloudConstPtr_RGB_NORMAL;
//
//using namespace std;
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//using namespace pcl::registration;
//PointCloud<PointXYZ>::Ptr src, tgt;
//
//////////////////////////////////////////////////////////////////////////////////
//void
//estimateKeypoints (const PointCloud<PointXYZ>::Ptr &src, 
//                   const PointCloud<PointXYZ>::Ptr &tgt,
//                   PointCloud<PointXYZ> &keypoints_src,
//                   PointCloud<PointXYZ> &keypoints_tgt)
//{
//  PointCloud<int> keypoints_src_idx, keypoints_tgt_idx;
//  // Get an uniform grid of keypoints
//  UniformSampling<PointXYZ> uniform;
//  uniform.setRadiusSearch (0.01);  // 1cm
//
//  uniform.setInputCloud (src);
//  uniform.compute (keypoints_src_idx);
//  copyPointCloud<PointXYZ, PointXYZ> (*src, keypoints_src_idx.points, keypoints_src);
//
//  uniform.setInputCloud (tgt);
//  uniform.compute (keypoints_tgt_idx);
//  copyPointCloud<PointXYZ, PointXYZ> (*tgt, keypoints_tgt_idx.points, keypoints_tgt);
//
//  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
//  // pcl_viewer source_pcd keypoints_src.pcd -ps 1 -ps 10
//  savePCDFileBinary ("keypoints_src.pcd", keypoints_src);
//  savePCDFileBinary ("keypoints_tgt.pcd", keypoints_tgt);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//void
//estimateNormals (const PointCloud<PointXYZ>::Ptr &src, 
//                 const PointCloud<PointXYZ>::Ptr &tgt,
//                 PointCloud<Normal> &normals_src,
//                 PointCloud<Normal> &normals_tgt)
//{
//  NormalEstimation<PointXYZ, Normal> normal_est;
//  normal_est.setInputCloud (src);
//  normal_est.setRadiusSearch (0.02);  // 2cm
//  normal_est.compute (normals_src);
//
//  normal_est.setInputCloud (tgt);
//  normal_est.compute (normals_tgt);
//
//  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
//  // pcl_viewer normals_src.pcd
//  PointCloud<PointNormal> s, t;
//  copyPointCloud<PointXYZ, PointNormal> (*src, s);
//  copyPointCloud<Normal, PointNormal> (normals_src, s);
//  copyPointCloud<PointXYZ, PointNormal> (*tgt, t);
//  copyPointCloud<Normal, PointNormal> (normals_tgt, t);
//  savePCDFileBinary ("normals_src.pcd", s);
//  savePCDFileBinary ("normals_tgt.pcd", t);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//void
//estimateFPFH (const PointCloud<PointXYZ>::Ptr &src, 
//              const PointCloud<PointXYZ>::Ptr &tgt,
//              const PointCloud<Normal>::Ptr &normals_src,
//              const PointCloud<Normal>::Ptr &normals_tgt,
//              const PointCloud<PointXYZ>::Ptr &keypoints_src,
//              const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
//              PointCloud<FPFHSignature33> &fpfhs_src,
//              PointCloud<FPFHSignature33> &fpfhs_tgt)
//{
//  FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
//  fpfh_est.setInputCloud (keypoints_src);
//  fpfh_est.setInputNormals (normals_src);
//  fpfh_est.setRadiusSearch (0.01); // 1m
//  fpfh_est.setSearchSurface (src);
//  fpfh_est.compute (fpfhs_src);
//
//  fpfh_est.setInputCloud (keypoints_tgt);
//  fpfh_est.setInputNormals (normals_tgt);
//  fpfh_est.setSearchSurface (tgt);
//  fpfh_est.compute (fpfhs_tgt);
//
//  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
//  // pcl_viewer fpfhs_src.pcd
//  PCLPointCloud2 s, t, out;
//  toPCLPointCloud2 (*keypoints_src, s); toPCLPointCloud2 (fpfhs_src, t); concatenateFields (s, t, out);
//  savePCDFile ("fpfhs_src.pcd", out);
//  toPCLPointCloud2 (*keypoints_tgt, s); toPCLPointCloud2 (fpfhs_tgt, t); concatenateFields (s, t, out);
//  savePCDFile ("fpfhs_tgt.pcd", out);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//void
//findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
//                     const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
//                     Correspondences &all_correspondences)
//{
//  CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
//  est.setInputCloud (fpfhs_src);
//  est.setInputTarget (fpfhs_tgt);
//  est.determineReciprocalCorrespondences (all_correspondences);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//void
//rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
//                          const PointCloud<PointXYZ>::Ptr &keypoints_src,
//                          const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
//                          Correspondences &remaining_correspondences)
//{
//  CorrespondenceRejectorDistance rej;
//  rej.setInputSource<PointXYZ> (keypoints_src);
//  rej.setInputTarget<PointXYZ> (keypoints_tgt);
//  rej.setMaximumDistance (1);    // 1m
//  rej.setInputCorrespondences (all_correspondences);
//  rej.getCorrespondences (remaining_correspondences);
//
//  std::cout<<"all_correspondences:"<<all_correspondences->size()<<std::endl;
//  std::cout<<"remaining_correspondences:"<<remaining_correspondences.size()<<std::endl;
//}
//
//
//////////////////////////////////////////////////////////////////////////////////
//void
//computeTransformation (const PointCloud<PointXYZ>::Ptr &src, 
//                       const PointCloud<PointXYZ>::Ptr &tgt,
//                       Eigen::Matrix4f &transform)
//{
//  // Get an uniform grid of keypoints
//  PointCloud<PointXYZ>::Ptr keypoints_src (new PointCloud<PointXYZ>), 
//                            keypoints_tgt (new PointCloud<PointXYZ>);
//
//  estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
//  print_info ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());
//
//  // Compute normals for all points keypoint
//  PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), 
//                          normals_tgt (new PointCloud<Normal>);
//  estimateNormals (src, tgt, *normals_src, *normals_tgt);
//  print_info ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());
//
//  // Compute FPFH features at each keypoint
//  PointCloud<FPFHSignature33>::Ptr fpfhs_src (new PointCloud<FPFHSignature33>), 
//                                   fpfhs_tgt (new PointCloud<FPFHSignature33>);
//  estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);
//
//  // Copy the data and save it to disk
///*  PointCloud<PointNormal> s, t;
//  copyPointCloud<PointXYZ, PointNormal> (*keypoints_src, s);
//  copyPointCloud<Normal, PointNormal> (normals_src, s);
//  copyPointCloud<PointXYZ, PointNormal> (*keypoints_tgt, t);
//  copyPointCloud<Normal, PointNormal> (normals_tgt, t);*/
//
//  // Find correspondences between keypoints in FPFH space
//  CorrespondencesPtr all_correspondences (new Correspondences), 
//                     good_correspondences (new Correspondences);
//  findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);
//
//  // Reject correspondences based on their XYZ distance
//  rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
//
//  for (int i = 0; i < good_correspondences->size (); ++i)
//    std::cerr << good_correspondences->at (i) << std::endl;
//  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
//  TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
//  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
//}
//
//
//bool loadPointCloud_ply(char* fileName, PointCloud<PointXYZ>::Ptr cloud){
//
//  std::ifstream input(fileName) ;
//  if(input.fail()) {
//    std::cout<<"could not open file!" << std::endl;
//    return false;
//  }
//
//  int num_header = 14;
//  int num_points = 0;
//  // this line contains point number
//  int line_num = 3;
//
//  for (int i=0; i<num_header; ++i) {
//    std::string line;
//    getline(input, line);
//
//    if (i==line_num) {
//      std::istringstream line_input(line);
//      std::string dummy1;
//      std::string dummy2;
//      line_input >> dummy1 >> dummy2 >>num_points;
//
//      printf("num_points:%d\n",num_points);
//    }
//  }
//
//  std::cout<< "===========================" <<std::endl;
//
//  for (int i=0; i<num_points; ++i) {
//    PointXYZ point_tem;
//
//    input >> point_tem.x >> point_tem.y >> point_tem.z;
//
//    cloud->push_back(point_tem);
//  }
//
//  return true;
//}
//
////append a cloud to another cloud
//void appendCloud( PointCloud<PointXYZ>::Ptr sourceCloud, PointCloud<PointXYZ>::Ptr targetCloud){
//  for(int i=0;i<sourceCloud->size();i++){
//    targetCloud->push_back(sourceCloud->at(i));
//  }
//}
//
//int
//main (int argc, char** argv)
//{
//  //// Parse the command line arguments for .pcd files
//  //std::vector<int> p_file_indices;
//  //p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
//  //if (p_file_indices.size () != 2)
//  //{
//  //  print_error ("Need one input source PCD file and one input target PCD file to continue.\n");
//  //  print_error ("Example: %s source.pcd target.pcd\n", argv[0]);
//  //  return (-1);
//  //}
//
//  // Load the files
//  /*print_info ("Loading %s as source and %s as target...\n", argv[p_file_indices[0]], argv[p_file_indices[1]]);
//  src.reset (new PointCloud<PointXYZ>);
//  tgt.reset (new PointCloud<PointXYZ>);
//  if (loadPCDFile (argv[p_file_indices[0]], *src) == -1 || loadPCDFile (argv[p_file_indices[1]], *tgt) == -1)
//  {
//    print_error ("Error reading the input files!\n");
//    return (-1);
//  }*/
//
//  PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
//  loadPointCloud_ply("data/1-0.ply", cloud1);
//
//  PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
//  loadPointCloud_ply("data/2-0.ply", cloud2);
//
//  // Compute the best transformtion
//  Eigen::Matrix4f transform;
//  computeTransformation (cloud1, cloud2, transform);
//
//  std::cerr << transform << std::endl;
//  // Transform the data and write it to disk
//  PointCloud<PointXYZ>::Ptr output(new PointCloud<PointXYZ>);
//  transformPointCloud (*cloud1, *output, transform);
//
//  appendCloud(cloud2, output);
//
//  savePCDFileBinary ("output.pcd", *output);
//}
//

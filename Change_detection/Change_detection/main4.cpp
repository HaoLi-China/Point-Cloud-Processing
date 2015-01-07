//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/octree/octree.h>
//#include <pcl/filters/extract_indices.h>
//
//#include <pcl/console/parse.h>
//
//enum 
//{
//  REDDIFF_MODE,
//  ONLYDIFF_MODE,
//  MODE_COUNT
//};
//
//class OpenNIChangeViewer
//{
//  public:
//    OpenNIChangeViewer (double resolution, int mode, int noise_filter)
//      : viewer ("PCL OpenNI Viewer")
//    {
//      octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA>(resolution);
//      mode_ = mode;
//      noise_filter_ = noise_filter;
//    }
//
//    void 
//    cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
//    {
//      std::cerr << cloud->points.size() << " -- ";
//
//      // assign point cloud to octree
//      octree->setInputCloud (cloud);
//
//      // add points from cloud to octree
//      octree->addPointsFromInputCloud ();
//
//      std::cerr << octree->getLeafCount() << " -- ";
//      boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);
//
//      // get a vector of new points, which did not exist in previous buffer
//      octree->getPointIndicesFromNewVoxels (*newPointIdxVector, noise_filter_);
//
//      std::cerr << newPointIdxVector->size() << std::endl;
//
//      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;
//
//      switch (mode_) 
//      {
//        case REDDIFF_MODE:
//          filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (*cloud));
//          filtered_cloud->points.reserve(newPointIdxVector->size());
//
//          for (std::vector<int>::iterator it = newPointIdxVector->begin (); it != newPointIdxVector->end (); it++)
//            filtered_cloud->points[*it].rgba = 255<<16;
//
//          if (!viewer.wasStopped())
//            viewer.showCloud (filtered_cloud);
//
//          break;
//        case ONLYDIFF_MODE:
//          filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//          filtered_cloud->points.reserve(newPointIdxVector->size());
//
//          for (std::vector<int>::iterator it = newPointIdxVector->begin (); it != newPointIdxVector->end (); it++)
//            filtered_cloud->points.push_back(cloud->points[*it]);
//
//
//          if (!viewer.wasStopped())
//            viewer.showCloud (filtered_cloud);
//          break;
//      }
//      
//      // switch buffers - reset tree
//      octree->switchBuffers ();
//    }
//    
//    void 
//    run ()
//    {
//      pcl::Grabber* interface = new pcl::OpenNIGrabber();
//
//      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
//        boost::bind (&OpenNIChangeViewer::cloud_cb_, this, _1);
//
//      boost::signals2::connection c = interface->registerCallback (f);
//      
//      interface->start ();
//      
//      while (!viewer.wasStopped())
//      {
//        boost::this_thread::sleep(boost::posix_time::seconds(1));
//      }
//
//      interface->stop ();
//    }
//
//    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> *octree;
//    pcl::visualization::CloudViewer viewer;
//
//    int mode_;
//    int noise_filter_;
//};
//
//int 
//main (int argc, char* argv[])
//{
//
//  std::cout << "Syntax is " << argv[0] << " [-r octree resolution] [-d] [-n noise_filter intensity] \n";
//
//  int mode = REDDIFF_MODE;
//  int noise_filter = 7;
//  double resolution = 0.01;
//
//  /*pcl::console::parse_argument (argc, argv, "-r", resolution);
//
//  pcl::console::parse_argument (argc, argv, "-n", noise_filter);
//
//  if (pcl::console::find_argument (argc, argv, "-d")>0) {
//  mode = ONLYDIFF_MODE;
//  }*/
//
//
//  OpenNIChangeViewer v (resolution, mode, noise_filter);
//  v.run ();
//  return 0;
//}

#include "visualizer.h"
#include <boost/thread/thread.hpp>

Visualizer::Visualizer(){
	viewer.reset (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->initCameraParameters();
}

Visualizer::~Visualizer(){

}

void Visualizer::showPoints_RGB(PointCloudPtr_RGB cloud){
	viewer->removeAllPointClouds();
	viewer->removeCoordinateSystem();

	viewer->addPointCloud (cloud, "PointCloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);

	viewer->resetCamera();

	while (!viewer->wasStopped ())
	{
	viewer->spinOnce(100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void Visualizer::show(){
	viewer->addCoordinateSystem(1.0);
	viewer->resetCamera();

	while (!viewer->wasStopped ())
	{
	viewer->spinOnce(100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
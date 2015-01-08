#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "common_type.h"
#include <pcl/visualization/pcl_visualizer.h>

class Visualizer
{
public:
    Visualizer();
    ~Visualizer();

public:
	void showPoints_RGB(PointCloudPtr_RGB cloud);
	void show();

public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif // VISUALIZER_H
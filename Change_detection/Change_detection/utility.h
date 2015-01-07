#ifndef UTILITY_H
#define UTILITY_H

#include "common_type.h"
#include "visualizer.h"

void draw_box(PointCloudPtr box_cloud, Visualizer& vs, float r, float g, float b, const char* id);

#endif // UTILITY_H
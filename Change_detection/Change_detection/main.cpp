#include "common_type.h"
#include "file_io.h"
#include "visualizer.h"
#include "detect_change.h"

int main (int argc, char** argv)
{
  PointCloudPtr_RGB cloudA (new PointCloud_RGB);
  loadPointCloud_ply("data/scan_frame2.ply", cloudA);
  PointCloudPtr_RGB cloudB (new PointCloud_RGB);
  loadPointCloud_ply("data/scan_frame3.ply", cloudB);

  PointCloudPtr_RGB planeCloudA(new PointCloud_RGB());
  PointCloudPtr rect_cloudA(new PointCloud());
  PointCloudPtr_RGB remainingCloudA(new PointCloud_RGB());
  pcl::ModelCoefficients coefficientsA;
  PointCloudPtr_RGB planeCloudB(new PointCloud_RGB());
  PointCloudPtr rect_cloudB(new PointCloud());
  PointCloudPtr_RGB remainingCloudB(new PointCloud_RGB());
  pcl::ModelCoefficients coefficientsB;
  detect_table(cloudA, coefficientsA, planeCloudA, rect_cloudA, remainingCloudA);
  detect_table(cloudB, coefficientsB, planeCloudB, rect_cloudB, remainingCloudB);

  /* showPointCloud(planeCloudA, "planeCloudA");
  showPointCloud(remainingCloudA, "remainingCloudA");
  showPointCloud(planeCloudB, "planeCloudB");
  showPointCloud(remainingCloudB, "remainingCloudB");*/

  Eigen::Matrix4f matrix_transformA;
  Eigen::Matrix4f matrix_transformA_r;
  Eigen::Matrix4f matrix_transformB;
  Eigen::Matrix4f matrix_transformB_r;

  getTemTransformMatrix(coefficientsA, matrix_transformA, matrix_transformA_r);
  getTemTransformMatrix(coefficientsB, matrix_transformB, matrix_transformB_r);

  PointCloudPtr_RGB tabletopCloudA(new PointCloud_RGB());
  PointCloudPtr_RGB tabletopCloudB(new PointCloud_RGB());
  getCloudOnTable(remainingCloudA, rect_cloudA, matrix_transformA, matrix_transformA_r, tabletopCloudA);
  getCloudOnTable(remainingCloudB, rect_cloudB, matrix_transformB, matrix_transformB_r, tabletopCloudB);

  //showPointCloud(tabletopCloudA, "tabletopCloudA");
  //showPointCloud(tabletopCloudB, "tabletopCloudB");

  PointCloudPtr_RGB resultA (new PointCloud_RGB);
  PointCloudPtr_RGB resultB (new PointCloud_RGB);
  detect_change(tabletopCloudA, tabletopCloudB, resultA, resultB);

  showPointCloud(resultA, "resultA");
  showPointCloud(resultB, "resultB");
}
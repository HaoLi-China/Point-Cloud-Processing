#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "supervoxel_clustering.hpp"

#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_iterator.hpp>

template class pcl::SupervoxelClustering<pcl::PointXYZRGBA>;
template class pcl::SupervoxelClustering<pcl::PointXYZRGB>;
template class pcl::SupervoxelClustering<pcl::PointXYZ>;

typedef pcl::SupervoxelClustering<pcl::PointXYZ>::VoxelData VoxelDataT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData VoxelDataRGBT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData VoxelDataRGBAT;

typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT> AdjacencyContainerT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT> AdjacencyContainerRGBT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT> AdjacencyContainerRGBAT;

template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT>;

template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ, AdjacencyContainerT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGB, AdjacencyContainerRGBT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, AdjacencyContainerRGBAT>;

//template class pcl::octree::OctreeBase<AdjacencyContainerRGBT, pcl::octree::OctreeContainerEmpty>;
//template class pcl::octree::OctreeBase<AdjacencyContainerRGBAT, pcl::octree::OctreeContainerEmpty>;

//template class pcl::octree::OctreeBreadthFirstIterator<pcl::octree::OctreeBase<AdjacencyContainerRGBT, pcl::octree::OctreeContainerEmpty> >;
//template class pcl::octree::OctreeBreadthFirstIterator<pcl::octree::OctreeBase<AdjacencyContainerRGBAT, pcl::octree::OctreeContainerEmpty> >;


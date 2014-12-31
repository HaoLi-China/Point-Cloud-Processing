#include "common_type.h"

MyPointCloud::MyPointCloud(){

}

MyPointCloud::~MyPointCloud(){

}

MyPointCloud_RGB::MyPointCloud_RGB(){

}

MyPointCloud_RGB::~MyPointCloud_RGB(){

}

MyPointCloud_RGB_NORMAL::MyPointCloud_RGB_NORMAL(){

}

MyPointCloud_RGB_NORMAL::~MyPointCloud_RGB_NORMAL(){

}

MyLine::MyLine(){

}

MyLine::~MyLine(){

}

//convert MyPointCloud to PointCloud
void MyPointCloud2PointCloud(MyPointCloud& mc, PointCloudPtr pc){
  for(int i=0;i<mc.mypoints.size();i++){
    pc->push_back(Point(mc.mypoints.at(i).x,mc.mypoints.at(i).y,mc.mypoints.at(i).z));
  }
}

//convert PointCloud to MyPointCloud
void PointCloud2MyPointCloud(PointCloudPtr pc, MyPointCloud& mc){
  for(int i=0;i<pc->size();i++){
    MyPt mp={pc->at(i).x,pc->at(i).y,pc->at(i).z};
    mc.mypoints.push_back(mp);
  }
}

//convert MyPointCloud_RGB to PointCloud_RGB
void MyPointCloud_RGB2PointCloud_RGB(MyPointCloud_RGB& mc, PointCloudPtr_RGB pc){
  for(int i=0;i<mc.mypoints.size();i++){
    Point_RGB point_tem;
    point_tem.x=mc.mypoints.at(i).x;
    point_tem.y=mc.mypoints.at(i).y;
    point_tem.z=mc.mypoints.at(i).z;
    point_tem.r=mc.mypoints.at(i).r;
    point_tem.g=mc.mypoints.at(i).g;
    point_tem.b=mc.mypoints.at(i).b;

    pc->push_back(point_tem);
  }
}

//convert PointCloud_RGB to MyPointCloud_RGB
void PointCloud_RGB2MyPointCloud_RGB(PointCloudPtr_RGB pc, MyPointCloud_RGB& mc){
  for(int i=0;i<pc->size();i++){
    MyPt_RGB mp={pc->at(i).x,pc->at(i).y,pc->at(i).z,pc->at(i).r,pc->at(i).g,pc->at(i).b};
    mc.mypoints.push_back(mp);
  }
}

//convert MyPointCloud_RGB_NORMAL to PointCloud_RGB_NORMAL
void MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(MyPointCloud_RGB_NORMAL& mc, PointCloudPtr_RGB_NORMAL pc){
  for(int i=0;i<mc.mypoints.size();i++){
    Point_RGB_NORMAL point_tem;
    point_tem.x=mc.mypoints.at(i).x;
    point_tem.y=mc.mypoints.at(i).y;
    point_tem.z=mc.mypoints.at(i).z;
    point_tem.r=mc.mypoints.at(i).r;
    point_tem.g=mc.mypoints.at(i).g;
    point_tem.b=mc.mypoints.at(i).b;
    point_tem.normal_x=mc.mypoints.at(i).normal_x;
    point_tem.normal_y=mc.mypoints.at(i).normal_y;
    point_tem.normal_z=mc.mypoints.at(i).normal_z;

    pc->push_back(point_tem);
  }
}

//convert PointCloud_RGB_NORMAL to MyPointCloud_RGB_NORMAL
void PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL pc, MyPointCloud_RGB_NORMAL& mc){
  for(int i=0;i<pc->size();i++){
    MyPt_RGB_NORMAL mp={pc->at(i).x,pc->at(i).y,pc->at(i).z,pc->at(i).r,pc->at(i).g,pc->at(i).b,pc->at(i).normal_x,pc->at(i).normal_y,pc->at(i).normal_z};
    mc.mypoints.push_back(mp);
  }
}

//convert MyPointCloud_RGB to MyPointCloud
void MyPointCloud_RGB2MyPointCloud(MyPointCloud_RGB& mc_rgb, MyPointCloud& mc){
  for(int i=0;i<mc_rgb.mypoints.size();i++){
    MyPt mpt;
    mpt.x=mc_rgb.mypoints.at(i).x;
    mpt.y=mc_rgb.mypoints.at(i).y;
    mpt.z=mc_rgb.mypoints.at(i).z;

    mc.mypoints.push_back(mpt);
  }
}

//convert MyPointCloud_RGB_NORMAL to MyPointCloud
void MyPointCloud_RGB_NORMAL2MyPointCloud(MyPointCloud_RGB_NORMAL& mc_rgb, MyPointCloud& mc){
  for(int i=0;i<mc_rgb.mypoints.size();i++){
    MyPt mpt;
    mpt.x=mc_rgb.mypoints.at(i).x;
    mpt.y=mc_rgb.mypoints.at(i).y;
    mpt.z=mc_rgb.mypoints.at(i).z;

    mc.mypoints.push_back(mpt);
  }
}

//Copy MyPointCloud_RGB to MyPointCloud_RGB
void CopyMyPointCloud_RGB(MyPointCloud_RGB& source, MyPointCloud_RGB& target){
  for(int i=0;i<source.mypoints.size();i++){
    MyPt_RGB mpt;
    mpt.x=source.mypoints.at(i).x;
    mpt.y=source.mypoints.at(i).y;
    mpt.z=source.mypoints.at(i).z;
    mpt.r=source.mypoints.at(i).r;
    mpt.g=source.mypoints.at(i).g;
    mpt.b=source.mypoints.at(i).b;

    target.mypoints.push_back(mpt);
  }
}

//convert MyPointCloud_RGB to MyPointCloud
void CopyMyPointCloud(MyPointCloud& source, MyPointCloud& target){
  for(int i=0;i<source.mypoints.size();i++){
    MyPt mpt;
    mpt.x=source.mypoints.at(i).x;
    mpt.y=source.mypoints.at(i).y;
    mpt.z=source.mypoints.at(i).z;

    target.mypoints.push_back(mpt);
  }
}
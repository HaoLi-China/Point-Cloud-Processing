#include "utility.h"

//draw box
void draw_box(PointCloudPtr box_cloud, Visualizer& vs, float r, float g, float b, const char* id){
  
  std::stringstream st0;
  std::stringstream st1;
  std::stringstream st2;
  std::stringstream st3;
  std::stringstream st4;
  std::stringstream st5;
  std::stringstream st6;
  std::stringstream st7;
  std::stringstream st8;
  std::stringstream st9;
  std::stringstream st10;
  std::stringstream st11;

  st0<<id<<"0";
  st1<<id<<"1";
  st2<<id<<"2";
  st3<<id<<"3";
  st4<<id<<"4";
  st5<<id<<"5";
  st6<<id<<"6";
  st7<<id<<"7";
  st8<<id<<"8";
  st9<<id<<"9";
  st10<<id<<"10";
  st11<<id<<"11";

  std::string id0=st0.str();
  std::string id1=st1.str();
  std::string id2=st2.str();
  std::string id3=st3.str();
  std::string id4=st4.str();
  std::string id5=st5.str();
  std::string id6=st6.str();
  std::string id7=st7.str();
  std::string id8=st8.str();
  std::string id9=st9.str();
  std::string id10=st10.str();
  std::string id11=st11.str();

  cout<<"id0:"<<id0<<endl;

  vs.viewer->addLine(box_cloud->at(0),box_cloud->at(1),r,g,b,id0);
  vs.viewer->addLine(box_cloud->at(1),box_cloud->at(2),r,g,b,id1);
  vs.viewer->addLine(box_cloud->at(2),box_cloud->at(3),r,g,b,id2);
  vs.viewer->addLine(box_cloud->at(3),box_cloud->at(0),r,g,b,id3);
  vs.viewer->addLine(box_cloud->at(0),box_cloud->at(4),r,g,b,id4);
  vs.viewer->addLine(box_cloud->at(4),box_cloud->at(5),r,g,b,id5);
  vs.viewer->addLine(box_cloud->at(5),box_cloud->at(6),r,g,b,id6);
  vs.viewer->addLine(box_cloud->at(6),box_cloud->at(7),r,g,b,id7);
  vs.viewer->addLine(box_cloud->at(7),box_cloud->at(4),r,g,b,id8);
  vs.viewer->addLine(box_cloud->at(1),box_cloud->at(5),r,g,b,id9);
  vs.viewer->addLine(box_cloud->at(2),box_cloud->at(6),r,g,b,id10);
  vs.viewer->addLine(box_cloud->at(3),box_cloud->at(7),r,g,b,id11);
}

//draw rect
void draw_rect(PointCloudPtr rect_cloud, Visualizer& vs, float r, float g, float b, const char* id){

  std::stringstream st0;
  std::stringstream st1;
  std::stringstream st2;
  std::stringstream st3;

  st0<<id<<"0";
  st1<<id<<"1";
  st2<<id<<"2";
  st3<<id<<"3";

  std::string id0=st0.str();
  std::string id1=st1.str();
  std::string id2=st2.str();
  std::string id3=st3.str();

  vs.viewer->addLine(rect_cloud->at(0),rect_cloud->at(1),r,g,b,id0);
  vs.viewer->addLine(rect_cloud->at(1),rect_cloud->at(2),r,g,b,id1);
  vs.viewer->addLine(rect_cloud->at(2),rect_cloud->at(3),r,g,b,id2);
  vs.viewer->addLine(rect_cloud->at(3),rect_cloud->at(0),r,g,b,id3);
}
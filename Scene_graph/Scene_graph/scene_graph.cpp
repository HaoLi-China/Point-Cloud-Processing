#include "scene_graph.h"

SceneGraph::SceneGraph(){

}

SceneGraph::~SceneGraph(){

}

void SceneGraph::init(vector<Node>& nodes,vector<Edge>& edges){
  this->nodes=nodes;
  this->edges=edges;
}

void SceneGraph::setNodes(vector<Node>& nodes){
  this->nodes=nodes;
}

void SceneGraph::setEdges(vector<Edge>& edges){
  this->edges=edges;
}

vector<Node>& SceneGraph::getNodes(){
  return this->nodes;
}

vector<Edge>& SceneGraph::getEdges(){
  return this->edges;
}

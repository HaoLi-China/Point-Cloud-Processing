#ifndef SCENE_GRAPH_H
#define SCENE_GRAPH_H

#include <pcl/ModelCoefficients.h>
#include <vector>

using namespace std;

//the struct of GraphNode
typedef struct GraphNode{
    int nodeIndex;
    int nodeType;//0:cylinder 1:sphere 2:rect
    pcl::ModelCoefficients coefficients;
} Node;

//the struct of GraphNode
typedef struct GraphEdge{
    int nodeIndex0;
    int nodeIndex1;
} Edge;

class SceneGraph{
public:
    SceneGraph();
    ~SceneGraph();

public:
    void init(vector<Node>& nodes,vector<Edge>& edges);

    void setNodes(vector<Node>& nodes);
    void setEdges(vector<Edge>& edges);
    vector<Node>& getNodes();
    vector<Edge>& getEdges();

private:
    vector<Node> nodes;
    vector<Edge> edges;
};

#endif // SCENE_GRAPH_H

#include<nokia_libraries/Graph.hpp>

int main(int argc, char **argv)
{
    std::vector<std::string> uavs;
    uavs = {"uav0", "uav1", "uav2", "uav3"};
    std::Graph g(uavs);

    g.addEdge("uav0", "uav1");
    g.addEdge("uav1", "uav2");
    g.addEdge("uav2", "uav3");
    g.addEdge("uav3", "uav0");
    g.printGraph();

    g.clearEdges();
    g.printGraph();
    g.rigidify();
    g.printGraph();
    g.printEdgeList();
    g.printVertexList();
    return 0;
}
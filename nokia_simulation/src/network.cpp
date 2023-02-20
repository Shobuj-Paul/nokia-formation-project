#include<nokia_libraries/Graph.hpp>

int main(int argc, char **argv)
{
    std::vector<std::string> uavs;
    uavs = {"uav0", "uav1", "uav2", "uav3"};
    std::Graph g(uavs);

    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 0);
    g.printGraph();

    g.clearEdges();
    g.printGraph();
    g.rigidify();
    g.printGraph();
    g.printEdgeList();
    g.printVertexList();
    return 0;
}
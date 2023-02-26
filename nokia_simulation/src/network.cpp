#include<nokia_libraries/Graph.hpp>
#include<math.h>

int main(int argc, char **argv)
{
    std::vector<std::string> uavs = {"uav0", "uav1", "uav2", "uav3"};
    std::Graph g(uavs);

    g.AddEdge("uav0", "uav1", 1);
    g.AddEdge("uav1", "uav2", 1);
    g.AddEdge("uav2", "uav3", 1);
    g.AddEdge("uav3", "uav0", 1);
    g.AddEdge("uav2", "uav0", sqrt(2));
    g.PrintEdgeList();
    g.PrintAdjacencyMatrix();

    g.ClearEdges();
    g.RigidPolygon(1);
    g.PrintAdjacencyMatrix();
    g.PrintEdgeList();
    g.PrintVertexList();
    return 0;
}
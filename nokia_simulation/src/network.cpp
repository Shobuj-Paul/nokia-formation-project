#include<nokia_simulation/Graph.hpp>

int main(int argc, char **argv)
{
    std::Graph g(4, false);

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
    return 0;
}
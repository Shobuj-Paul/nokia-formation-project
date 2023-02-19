#include<nokia_simulation/Graph.hpp>

namespace std {

    Graph::Graph(int numVertices, bool isDirected) {
        numEdges = 0;
        adjMatrix = new int*[numVertices];
        for(int i=0; i<numEdges; i++)
            adjMatrix[i] = new int[numVertices];

        for(int i=0; i<numVertices; i++){
            for(int j=0; j<numVertices; j++)
                adjMatrix[i][j] = 0;
        }
    }

    Graph::~Graph(){
        for(int i=0; i<numVertices; i++)
            delete[] adjMatrix[i];
        delete adjMatrix;
    }

    int Graph::getNumVertices() {
        return numVertices;
    }

    int Graph::getNumEdges() {
        return numEdges;
    }

bool Graph::isEdge(int v1, int v2) {
    for(int i=0; i<edgeList.size(); i++) {
        if((edgeList[i].first == v1 && edgeList[i].second == v2) ||
           (edgeList[i].first == v2 && edgeList[i].second == v1))
            return true;
    }
    return false;
}

    void Graph::addEdge(int v1, int v2) {
        numEdges++;
        edgeList.push_back(make_pair(v1, v2));
        adjMatrix[v1][v2] = 1;
        adjMatrix[v2][v1] = 1;
    }

    void Graph::removeEdge(int v1, int v2)
    {
        if(isEdge(v1, v2)) {
            for(int i=0; i<edgeList.size(); i++) {
                if((edgeList[i].first == v1 && edgeList[i].second == v2) ||
                   (edgeList[i].first == v2 && edgeList[i].second == v1)) {
                    edgeList.erase(edgeList.begin() + i);
                    numEdges--;
                    break;
                }
            }
        }
        adjMatrix[v1][v2] = 0;
        adjMatrix[v2][v1] = 0;
    }

    void Graph::printGraph() {
        for(int i=0; i<numVertices; i++) {
            for(int j=0; j<numVertices; j++)
                std::cout << adjMatrix[i][j] << " ";
            std::cout << endl;
        }
    }

}

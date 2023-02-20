#include<nokia_simulation/Graph.hpp>

namespace std {

    Graph::Graph(int numVertices, bool isDirected) : numVertices(numVertices), isDirected(isDirected) {
        numEdges = 0;
        adjMatrix = new int*[numVertices];
        for(int i=0; i<numVertices; i++)
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
        if(isDirected==false){
            for(int i=0; i<edgeList.size(); i++) {
                if((edgeList[i].first == v1 && edgeList[i].second == v2) ||
                   (edgeList[i].first == v2 && edgeList[i].second == v1))
                    return true;
            }
        }
        else if(isDirected==true){
            for(int i=0; i<edgeList.size(); i++) {
                if(edgeList[i].first == v1 && edgeList[i].second == v2)
                    return true;
            }
        }
        return false;
    }

    void Graph::addEdge(int v1, int v2) {
        numEdges++;
        edgeList.push_back(make_pair(v1, v2));
        if(isDirected==false) {
            adjMatrix[v1][v2] = 1;
            adjMatrix[v2][v1] = 1;
        }
        else if(isDirected==true) {
            adjMatrix[v1][v2] = 1;
        }
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
        if (isDirected == false){
            adjMatrix[v1][v2] = 0;
            adjMatrix[v2][v1] = 0;
        }
        else if (isDirected == true){
            adjMatrix[v1][v2] = 0;
        }
    }

    void Graph::clearEdges() {
        edgeList.clear();
        numEdges = 0;
        for(int i=0; i<numVertices; i++){
            for(int j=0; j<numVertices; j++)
                adjMatrix[i][j] = 0;
        }
    }

    bool Graph::isRigid() {
        if(numEdges == 2*numVertices-3)
            return true;
        else
            return false;
    }

    void Graph::rigidify() {
        if(numEdges!=0) {
            std::cout << "Graph is not empty. Clear Edges." << std::endl;
            return;
        }
        else if(isRigid() == true && numEdges == 0) {
            std::cout << "Graph is already rigid." << std::endl;
            return;
        }
        else if(isRigid() == false && numEdges == 0) {
            for(int i=0; i<numVertices-1; i++)
                addEdge(i, i+1);
            addEdge(numVertices-1, 0);
            for(int i=0; i<numVertices-3; i++)
                addEdge(0, i+2);
        }
    }
    
    void Graph::printGraph() {
        std::cout << "Adjacency Matrix:" << std::endl;
        std::cout << "-----------------" << std::endl;
        for(int i=0; i<numVertices; i++) {
            for(int j=0; j<numVertices; j++)
                std::cout << adjMatrix[i][j] << " ";
            std::cout << endl;
        }
        std::cout << "-----------------" << std::endl;
    }
    
    void Graph::printEdgeList() {
        std::cout << "Edge List:" << std::endl;
        std::cout << "---" << std::endl;
        for(int i=0; i<edgeList.size(); i++)
            std::cout << edgeList[i].first << " " << edgeList[i].second << std::endl;
        std::cout << "---" << std::endl;
    }

}

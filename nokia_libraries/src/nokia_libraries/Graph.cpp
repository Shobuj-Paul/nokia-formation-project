#include<nokia_libraries/Graph.hpp>

namespace std {

    Graph::Graph(std::vector<std::string> vertices, bool isDirected)
                 : numVertices(vertices.size()), isDirected(isDirected) {
        numEdges = 0;
        for(int i=0; i<vertices.size(); i++)
            vertexList[vertices[i]] = i;
        
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
        delete[] adjMatrix;
    }

    int Graph::getNumVertices() {
        return numVertices;
    }

    int Graph::getNumEdges() {
        return numEdges;
    }

    bool Graph::isEdge(int vertex_1, int vertex_2) {
        if(isDirected==false){
            for(int i=0; i<edgeList.size(); i++) {
                if(edgeList.find(make_pair(vertex_1, vertex_2)) == edgeList.end() ||
                   edgeList.find(make_pair(vertex_2, vertex_1)) == edgeList.end())
                    return true;
            }
        }
        else if(isDirected==true){
            for(int i=0; i<edgeList.size(); i++) {
                if(edgeList.find(make_pair(vertex_1, vertex_2)) == edgeList.end())
                    return true;
            }
        }
        return false;
    }

    void Graph::addEdge(std::string agent1, std::string agent2, double weight) {
        numEdges++;
        int vertex_1 = vertexList[agent1];
        int vertex_2 = vertexList[agent2];
        edgeList[(make_pair(vertex_1, vertex_2))] = weight;
        if(isDirected==false) {
            adjMatrix[vertex_1][vertex_2] = 1;
            adjMatrix[vertex_2][vertex_1] = 1;
        }
        else if(isDirected==true) {
            adjMatrix[vertex_1][vertex_2] = 1;
        }
    }

    void Graph::addEdge(int vertex_1, int vertex_2, double weight) {
        numEdges++;
        edgeList[make_pair(vertex_1, vertex_2)] = weight;
        if(isDirected==false) {
            adjMatrix[vertex_1][vertex_2] = 1;
            adjMatrix[vertex_2][vertex_1] = 1;
        }
        else if(isDirected==true) {
            adjMatrix[vertex_1][vertex_2] = 1;
        }
    }

    void Graph::removeEdge(int vertex_1, int vertex_2) {
        if(isEdge(vertex_1, vertex_2)) {
            for(int i=0; i<edgeList.size(); i++) {
                if(isEdge(vertex_1, vertex_2)) {
                    edgeList.erase(make_pair(vertex_1, vertex_2));
                    numEdges--;
                    break;
                }
            }
        }
        if (isDirected == false){
            adjMatrix[vertex_1][vertex_2] = 0;
            adjMatrix[vertex_2][vertex_1] = 0;
        }
        else if (isDirected == true){
            adjMatrix[vertex_1][vertex_2] = 0;
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
    
    void Graph::printAdjacencyMatrix() {
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
        std::map<std::pair<int, int>, double>::iterator it;
        for(it=edgeList.begin(); it!=edgeList.end(); it++)
            std::cout << "Edge: (" << it->first.first << ", " << it->first.second << ") Weight: " << it->second << std::endl;
        std::cout << "---" << std::endl;
    }

    void Graph::printVertexList() {
        std::cout << "Vertex List:" << std::endl;
        std::cout << "---------------------" << std::endl;
        std::map<std::string, int>::iterator it;
        for(it=vertexList.begin(); it!=vertexList.end(); it++)
            std::cout << "Vertex: " << it->first << " Index: " << it->second << std::endl;
        std::cout << "---------------------" << std::endl;
    }
}

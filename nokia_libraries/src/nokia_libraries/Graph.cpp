#include<nokia_libraries/Graph.hpp>

namespace std {

    Graph::Graph(std::vector<std::string> vertices, bool isDirected)
                 : numVertices(vertices.size()), isDirected(isDirected) {
        numEdges = 0;
        for(int i=0; i<vertices.size(); i++)
            vertexList[vertices[i]] = i;
        
        adjMatrix = new double*[numVertices];
        for(int i=0; i<numVertices; i++)
            adjMatrix[i] = new double[numVertices];

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

    int Graph::NumVertices() {
        return numVertices;
    }

    int Graph::NumEdges() {
        return numEdges;
    }

    bool Graph::IsEdge(int vertex_1, int vertex_2) {
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

    void Graph::AddEdge(std::string agent1, std::string agent2, double weight) {
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

    void Graph::AddEdge(int vertex_1, int vertex_2, double weight) {
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

    void Graph::RemoveEdge(int vertex_1, int vertex_2) {
        if(IsEdge(vertex_1, vertex_2)) {
            for(int i=0; i<edgeList.size(); i++) {
                if(IsEdge(vertex_1, vertex_2)) {
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

    void Graph::ClearEdges() {
        edgeList.clear();
        numEdges = 0;
        for(int i=0; i<numVertices; i++){
            for(int j=0; j<numVertices; j++)
                adjMatrix[i][j] = 0;
        }
    }

    bool Graph::IsRigid() {
        if(numEdges == 2*numVertices-3)
            return true;
        else
            return false;
    }

    void Graph::MakeRigidPolygon(double sideLength) {
        auto deg2radians = [&](double degrees) -> double {
            return degrees*3.14159265358979323846/180.0;
        };
        auto diagonal = [&](double sideLength, int vertex) -> double {
            return sideLength*sqrt(2.0 - 2.0*cos(deg2radians((1-(vertex+1))*360/numVertices)));
        };

        if(numEdges!=0) {
            std::cout << "Graph is not empty. Clear Edges." << std::endl;
            return;
        }
        else if(IsRigid() == true && numEdges == 0) {
            std::cout << "Graph is already rigid." << std::endl;
            return;
        }
        else if(IsRigid() == false && numEdges == 0) {
            for(int i=0; i<numVertices-1; i++)
                AddEdge(i, i+1, sideLength);
            AddEdge(numVertices-1, 0);
            for(int i=0; i<numVertices-3; i++)
                AddEdge(0, i+2, diagonal(sideLength, i+2));
        }
    }
    
    void Graph::PrintAdjacencyMatrix() {
        std::cout << "Adjacency Matrix:" << std::endl;
        std::cout << "-----------------" << std::endl;
        for(int i=0; i<numVertices; i++) {
            for(int j=0; j<numVertices; j++)
                std::cout << adjMatrix[i][j] << " ";
            std::cout << endl;
        }
        std::cout << "-----------------" << std::endl;
    }
    
    void Graph::PrintEdgeList() {
        std::cout << "Edge List:" << std::endl;
        std::cout << "---" << std::endl;
        std::map<std::pair<int, int>, double>::iterator it;
        for(it=edgeList.begin(); it!=edgeList.end(); it++)
            std::cout << "Edge: (" << it->first.first << ", " << it->first.second << ") Weight: " << it->second << std::endl;
        std::cout << "---" << std::endl;
    }

    void Graph::PrintVertexList() {
        std::cout << "Vertex List:" << std::endl;
        std::cout << "---------------------" << std::endl;
        std::map<std::string, int>::iterator it;
        for(it=vertexList.begin(); it!=vertexList.end(); it++)
            std::cout << "Vertex: " << it->first << " Index: " << it->second << std::endl;
        std::cout << "---------------------" << std::endl;
    }
}

#pragma once

#include <iostream>
#include <vector>
#include <map>

namespace std {

    class Graph {
        private:
            int numVertices; //Number of nodes
            int numEdges; //Number of connections between nodes
            int** adjMatrix; //The adjacency matrix containing the connection information
            bool isDirected; //Whether the graph is directed or not
            std::vector<std::pair<int, int>> edgeList; //The list of edges in pair integer format
            std::map<std::string, int> vertexList; //The list of vertices in string to integer map format
        public:
            Graph(std::vector<std::string> vertices, bool isDirected = false); //Constructor for allocating memory to the adjacency matrix
            ~Graph(); //Destructor for freeing memory from the adjacency matrix
            int getNumVertices(); //Returns the number of nodes in the graph
            int getNumEdges(); //Returns the number of connections in the graph
            bool isEdge(int, int); //Returns whether there is an edge between two nodes given node indices
            bool isRigid(); //Returns whether the graph is stricturally rigid or not
            void addEdge(std::string, std::string); //Adds an edge between two nodes given node names
            void addEdge(int, int); //Adds an edge between two nodes given node indices
            void removeEdge(int, int); //Removes an edge between two nodes given node indices
            void clearEdges(); //Removes all edges from the graph
            void rigidify(); //Generates a rigid graph bu adding minimum number of edges required
            void printAdjacencyMatrix(); //Prints the adjacency matrix
            void printEdgeList(); //Prints the edge list
            void printVertexList(); //Prints the vertex list
    };
}
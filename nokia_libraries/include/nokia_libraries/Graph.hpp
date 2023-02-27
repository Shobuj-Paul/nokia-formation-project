#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <math.h>

namespace std {

    class Graph {
        private:
            int numVertices; //Number of nodes
            int numEdges; //Number of connections between nodes
            double** adjMatrix; //The adjacency matrix containing the connection information
            bool isDirected; //Whether the graph is directed or not
            std::map<std::pair<int, int>, double> edgeList; //Map of edge pairs and their weights
            std::map<std::string, int> vertexList; //The list of vertices in string to integer map format
        public:
            Graph(std::vector<std::string>, bool = false); //Constructor for allocating memory to the adjacency matrix
            ~Graph(); //Destructor for freeing memory from the adjacency matrix

            int NumVertices(); //Returns the number of nodes in the graph
            int NumEdges(); //Returns the number of connections in the graph

            bool IsEdge(int, int); //Returns whether there is an edge between two nodes given node indices
            bool IsRigid(); //Returns whether the graph is stricturally rigid or not
            
            void AddEdge(std::string, std::string, double = 1); //Adds an edge between two nodes given node names
            void AddEdge(int, int, double = 1); //Adds an edge between two nodes given node indices
            void RemoveEdge(int, int); //Removes an edge between two nodes given node indices
            void ClearEdges(); //Removes all edges from the graph
            void MakeRigidPolygon(double); //Generates a rigid graph by adding minimum number of edges required
            void PrintAdjacencyMatrix(); //Prints the adjacency matrix
            void PrintEdgeList(); //Prints the edge list
            void PrintVertexList(); //Prints the vertex list
    };
}
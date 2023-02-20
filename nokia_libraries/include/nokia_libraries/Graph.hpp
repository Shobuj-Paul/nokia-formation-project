#pragma once

#include <iostream>
#include <vector>
#include <map>

namespace std {

    class Graph {
        private:
            int numVertices;
            int numEdges;
            int** adjMatrix;
            bool isDirected;
            std::vector<std::pair<int, int>> edgeList;
            std::map<std::string, int> vertexList;
        public:
            Graph(std::vector<std::string> vertices, bool isDirected = false);
            ~Graph();
            int getNumVertices();
            int getNumEdges();
            bool isEdge(int v1, int v2);
            bool isRigid();
            void addEdge(int v1, int v2);
            void addVertex(std::string vertex);
            void removeEdge(int v1, int v2);
            void clearEdges();
            void rigidify();
            void printGraph();
            void printEdgeList();
            void printVertexList();
    };
}
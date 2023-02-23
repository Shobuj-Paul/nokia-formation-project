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
            bool isEdge(int, int);
            bool isRigid();
            void addEdge(std::string, std::string);
            void addEdge(int, int);
            void removeEdge(int, int);
            void clearEdges();
            void rigidify();
            void printGraph();
            void printEdgeList();
            void printVertexList();
    };
}
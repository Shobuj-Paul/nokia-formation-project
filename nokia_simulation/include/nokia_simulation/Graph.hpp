#pragma once

#include <vector>
#include <iostream>

namespace std {

    class Graph {
        private:
            int numVertices;
            int numEdges;
            int** adjMatrix;
            bool isDirected;
            std::vector<std::pair<int, int>> edgeList;
        public:
            Graph(int numVertices, bool isDirected = false);
            ~Graph();
            int getNumVertices();
            int getNumEdges();
            bool isEdge(int v1, int v2);
            bool isRigid();
            void addEdge(int v1, int v2);
            void removeEdge(int v1, int v2);
            void clearEdges();
            void rigidify();
            void printGraph();
            void printEdgeList();
    };
}
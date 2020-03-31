#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include "structs.h"
#include "tinyxml2.h"
#include "gl_const.h"
#include "lineofsight.h"

namespace SIPP {
    class Map {
    public:
        std::vector<std::vector<std::vector<int>>> Grid;
        unsigned int dimx, dimy, dimz;

    public:
        Map();

        ~Map();

//        bool getMap(const char *FileName);

        bool CellIsTraversable(int i, int j, int k) const;

        bool CellOnGrid(int i, int j, int k) const;

        bool CellIsObstacle(int i, int j, int k) const;

        int getValue(int i, int j, int k) const;

        std::vector<Node> getValidMoves(int i, int j, int k, int m, double size) const;
    };
}

#endif

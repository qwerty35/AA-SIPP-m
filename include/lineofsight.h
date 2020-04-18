/* (c) 2017. Andreychuk A.
 * This class implements line-of-sight function for a variable size of agent.
 * It also has a method for checking cell's traversability.
 * For its work is needed the size of agent and a map container that has 'cellIsObstacle' and 'cellOnGrid' methods.
 * If it is not possible to give the permission to access the grid, the one can use 'getCellsCrossedByLine' method.
 * It doesn't use grid and returns a set of all cells(as pairs of coordinates) that are crossed by an agent moving along a line.
 */

#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "gl_const.h"
#include <array>

#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <algorithm>

namespace SIPP {
    class LineOfSight {
    public:
        LineOfSight(
//                double agentSize = 0.5
                        ) {
//            this->agentSize = agentSize;
//            int add_x, add_y, add_z, num = agentSize + 0.5 - CN_EPSILON;
//            for (int x = -num; x <= +num; x++) {
//                for (int y = -num; y <= +num; y++) {
//                    for (int z = -num; z <= +num; z++) {
//                        add_x = x != 0 ? 1 : 0;
//                        add_y = y != 0 ? 1 : 0;
//                        add_z = z != 0 ? 1 : 0;
//                        if ((pow(2 * abs(x) - add_x, 2) + pow(2 * abs(y) - add_y, 2)) + pow(2 * abs(z) - add_z, 2) <
//                            pow(2 * agentSize, 2)) //TODO: is this right?
//                            cells.push_back({x, y, z});
//                    }
//                }
//            }
//            if (cells.empty())
                cells.push_back({0, 0, 0});
        }

        LineOfSight(CollisionModel collision_model) {
            this->r = collision_model.r;
            this->a = collision_model.a;
            this->b = collision_model.b;

            int x_size = r - CN_EPSILON;
            int y_size = r - CN_EPSILON;
            int zu_size = a - CN_EPSILON;
            int zl_size = b - CN_EPSILON;

            for (int x = -x_size; x <= +x_size; x++) {
                for (int y = -y_size; y <= +y_size; y++) {
                    for (int z = -zl_size; z <= +zu_size; z++) {
                        cells.push_back({x, y, z});
                    }
                }
            }
            if (cells.empty())
                cells.push_back({0, 0, 0});
        }

        void setSize(double agentSize) {
            r = agentSize;
            int add_x, add_y, add_z, num = agentSize + 0.5 - CN_EPSILON;
            cells.clear();
            for (int x = -num; x <= +num; x++) {
                for (int y = -num; y <= +num; y++) {
                    for(int z = -num; z <= +num; z++ ) {
                        add_x = x != 0 ? 1 : 0;
                        add_y = y != 0 ? 1 : 0;
                        add_z = z != 0 ? 1 : 0;
                        if ((pow(2 * abs(x) - add_x, 2) + pow(2 * abs(y) - add_y, 2)) + pow(2 * abs(z) - add_z, 2) <
                            pow(2 * agentSize, 2)) //TODO: is this right?
                            cells.push_back({x, y, z});
                    }
                }
            }
            if (cells.empty())
                cells.push_back({0, 0, 0});
        }

        template<class T>
        std::vector<std::array<int, 3>> getCellsCrossedByLine(int x1, int y1, int z1, int x2, int y2, int z2, const T &map) {
            std::vector<std::array<int, 3>> lineCells(0);
            if (x1 == x2 && y1 == y2 && z1 == z2) {
                for (auto cell:cells)
                    lineCells.push_back({x1 + cell[0], y1 + cell[1], z1 + cell[2]});
                return lineCells;
            }
            int delta_x = std::abs(x1 - x2);
            int delta_y = std::abs(y1 - y2);
            int delta_z = std::abs(z1 - z2);
            if ((delta_x > 0 && x1 > x2) || (delta_y > 0 && y1 > y2) || (delta_z > 0 && z1 > z2)) {
                std::swap(x1, x2);
                std::swap(y1, y2);
                std::swap(z1, z2);
            }
            int step_x = (x1 < x2 ? 1 : -1);
            int step_y = (y1 < y2 ? 1 : -1);
            int step_z = (z1 < z2 ? 1 : -1);
            int x = x1, y = y1, z = z1;
            std::array<int, 3> add;
            std::vector<std::array<int, 3>> circle(0);
            if (delta_x > 0) {
                for(int i = 0; i < cells.size(); i++){
                    if(cells[i][0] == 0){
                        circle.emplace_back(cells[i]);
                    }
                }
                for (x = x1; x != x2 + step_x; x += step_x) {
                    for(int i = 0; i < circle.size(); i++){
                        lineCells.push_back({x, y + circle[i][1], z + circle[i][2]});
                    }
                }
            } else if(delta_y > 0) {
                for(int i = 0; i < cells.size(); i++){
                    if(cells[i][1] == 0){
                        circle.emplace_back(cells[i]);
                    }
                }
                for (y = y1; y != y2 + step_y; y += step_y) {
                    for(int i = 0; i < circle.size(); i++){
                        lineCells.push_back({x + circle[i][0], y, z + circle[i][2]});
                    }
                }
            } else{
                for(int i = 0; i < cells.size(); i++){
                    if(cells[i][2] == 0){
                        circle.emplace_back(cells[i]);
                    }
                }
                for (z = z1; z != z2 + step_z; z += step_z) {
                    for(int i = 0; i < circle.size(); i++){
                        lineCells.push_back({x + circle[i][0], y + circle[i][1], z});
                    }
                }
            }
            for (int k = 0; k < cells.size(); k++) {
                add = {x1 + cells[k][0], y1 + cells[k][1], z1 + cells[k][2]};
                if (std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                    lineCells.push_back(add);
                add = {x2 + cells[k][0], y2 + cells[k][1], z2 + cells[k][2]};
                if (std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                    lineCells.push_back(add);
            }

            for (auto it = lineCells.begin(); it != lineCells.end(); it++)
                if (!map.CellOnGrid(it->at(0), it->at(1), it->at(2))) {
                    lineCells.erase(it);
                    it = lineCells.begin();
                }
            return lineCells;
        }
        //returns all cells that are affected by agent during moving along a line

        template<class T>
        bool checkTraversability(int x, int y, int z, const T &map) {
//            for (int k = 0; k < cells.size(); k++)
//                if (!map.CellOnGrid(x + cells[k][0], y + cells[k][1], z + cells[k][2]) ||
//                    map.CellIsObstacle(x + cells[k][0], y + cells[k][1], z + cells[k][2]))
//                    return false;
//            return true;
            if (!map.CellOnGrid(x, y, z) || map.CellIsObstacle(x, y, z, r))
                return false;
            return true;
        }
        //checks traversability of all cells affected by agent's body

        template<class T>
        bool checkLine(int x1, int y1, int z1, int x2, int y2, int z2, const T &map) {
            //if(!checkTraversability(x1, y1) || !checkTraversability(x2, y2)) //additional check of start and goal traversability,
            //    return false;                                                //it can be removed if they are already checked

            int delta_x = std::abs(x1 - x2);
            int delta_y = std::abs(y1 - y2);
            int delta_z = std::abs(z1 - z2);
            if ((delta_x > 0 && x1 > x2) || (delta_y > 0 && y1 > y2) || (delta_z > 0 && z1 > z2)) {
                std::swap(x1, x2);
                std::swap(y1, y2);
                std::swap(z1, z2);
            }
            int step_x = (x1 < x2 ? 1 : -1);
            int step_y = (y1 < y2 ? 1 : -1);
            int step_z = (z1 < z2 ? 1 : -1);
            int x = x1, y = y1, z = z1;
            std::vector<std::array<int, 3>> circle(0);

            // It assumes that line is all orthogonal.
            if((delta_x > 0 && delta_y > 0) || (delta_x > 0 && delta_z > 0) || (delta_y > 0 && delta_z > 0)){
                std::cerr << "sipp: invalid diagonal move";
            }

            if (delta_x > 0) {
                for (x = x1; x != x2 + step_x; x += step_x) {
                    for(int i = 0; i < cells.size(); i++){
                        if(cells[i][0] == 0){
                            circle.emplace_back(cells[i]);
                        }
                    }
                    for(int i = 0; i < circle.size(); i++) {
                        if (map.CellIsObstacle(x, y + circle[i][1], z + circle[i][2])) {
                            return false;
                        }
                    }
                }
            } else if(delta_y > 0) {
                for(int i = 0; i < cells.size(); i++){
                    if(cells[i][1] == 0){
                        circle.emplace_back(cells[i]);
                    }
                }
                for(int i = 0; i < circle.size(); i++){
                    if (map.CellIsObstacle(x + circle[i][0], y, z + circle[i][2])) {
                        return false;
                    }
                }
            } else{
                for(int i = 0; i < cells.size(); i++){
                    if(cells[i][2] == 0){
                        circle.emplace_back(cells[i]);
                    }
                }
                for(int i = 0; i < circle.size(); i++){
                    if (map.CellIsObstacle(x + circle[i][0], y + circle[i][1], z)) {
                        return false;
                    }
                }
            }
            return true;
        }
        //checks line-of-sight between a line

        std::vector<std::array<int, 3>> getCells(int i, int j, int k) {
            std::vector<std::array<int, 3>> cells;
            for (int iter = 0; iter < this->cells.size(); iter++)
                cells.push_back({i + this->cells[iter][0], j + this->cells[iter][1], k + this->cells[iter][2]});
            return cells;
        }

    private:
        double r, a, b;
        std::vector<std::array<int, 3>> cells; //cells that are affected by agent's body
    };
}
#endif // LINEOFSIGHT_H

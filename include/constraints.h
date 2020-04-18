#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include <lineofsight.h>
#include "map.h"

namespace SIPP {
    class Constraints {
    public:
        Constraints(int dimx, int dimy, int dimz);

        ~Constraints() {}

        void updateConstraints(const Map &map);

        void updateCellSafeIntervals(std::array<int, 3> cell);

        std::vector<std::pair<double, double> >
        getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int dimy, int dimz);

        std::vector<std::pair<double, double> > getSafeIntervals(Node curNode);

        void addConstraints(const std::vector<Node> &sections, int agent_id, double mspeed, const Map &map);

        std::vector<std::pair<double, double> >
        findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close,
                      const Map &map);

        std::pair<double, double> getSafeInterval(int i, int j, int k, int n) { return safe_intervals[i][j][k][n]; }

        void resetConstraints(int dimx, int dimy, int dimz);

        void resetSafeIntervals(int dimx, int dimy, int dimz);

        void addStartConstraint(int i, int j, int k, int size, std::vector<std::array<int, 3>> cells, int agent_id);

        void removeStartConstraint(std::vector<std::array<int, 3>> cells, int start_i, int start_j, int start_k);

//        void setSize(double size) { agentsize = size; }

        void setParams(int agent_id, CollisionModels collision_models, double mspeed, double rspeed, double tweight, double inflateintervals) {
            this->agent_id = agent_id;
            this->collision_models = collision_models;
            this->mspeed = mspeed;
            this->rspeed = rspeed;
            this->tweight = tweight;
            this->inflateintervals = inflateintervals;
        }

        double minDist_LineSegment(Point A, Point C, Point D);

        double minDist_StraightLine(Point A, Point C, Point D);

        double minBoxDist_LineSegment(Vector3D C, Vector3D D);


    private:
        bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
        bool hasBoxPassed(Point A, Point C, Point D, int section_agent_id);
        bool isPointInBox(Point A, Point C, int section_agent_id);
        double boxDistance(Vector3D A);
        bool isMinBoxPointInLineSegment(const Vector3D& C, const Vector3D& D);

        std::vector<section> section_list;
        std::vector<std::vector<std::vector<std::vector<section>>>> constraints;
        std::vector<std::vector<std::vector<std::vector<std::pair<double, double>>>>> safe_intervals;
        double rspeed;
        double mspeed;
        CollisionModels collision_models;
        int agent_id;
        double tweight;
        double inflateintervals;

    };
}

#endif // CONSTRAINTS_H

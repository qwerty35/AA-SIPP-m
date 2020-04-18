#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>

namespace SIPP {
    struct CollisionModel{
        CollisionModel(double _r = -1, double _a = -1, double _b = -1) : r(_r), a(_a), b(_b) {}
        double r;
        double a;
        double b;
    };
    typedef std::vector<CollisionModel> CollisionModels;

    struct conflict {
        int agent1;
        int agent2;
        int sec1;
        int sec2;
        double i;
        double j;
        double k;
        double g;
    };

    struct Agent {
        std::string id;
        int start_i;
        int start_j;
        int start_k;
        double start_heading;
        int goal_i;
        int goal_j;
        int goal_k;
        double goal_heading;
        int agent_id;
        CollisionModels collision_models;
        double size;
        double rspeed;
        double mspeed;

        Agent() {
            start_i = -1;
            start_j = -1;
            start_k = -1;
            goal_i = -1;
            goal_j = -1;
            goal_k = -1;
            agent_id = -1;
            size = CN_DEFAULT_SIZE;
            mspeed = CN_DEFAULT_MSPEED;
            rspeed = CN_DEFAULT_RSPEED;
            start_heading = CN_DEFAULT_SHEADING;
            goal_heading = CN_DEFAULT_GHEADING;
        }
    };

    struct DynamicAgent {
        std::string id;
        std::vector<double> startState;
        std::vector<double> goalState;
        std::vector<double> speed;
        double mspeed;
        double size;

        DynamicAgent() {
            startState.resize(9);
            goalState.resize(9);
            speed.resize(3);
            size = CN_DEFAULT_SIZE;
        }
    };

    struct constraint {
        double i;
        double j;
        double k;
        double g;
        bool goal;
    };

    struct movement {
        double g;
        int p_dir;
        int s_dir;
    };

    struct Node {
        Node(int _i = -1, int _j = -1, int _k = -1, double _g = -1, double _F = -1) : i(_i), j(_j), k(_k), g(_g), F(_F), Parent(nullptr) {}

        ~Node() { Parent = nullptr; }

        int i, j, k;
//        double size;
        double g;
        double F;
        double heading;
        Node *Parent;
        std::pair<double, double> interval;
    };

    struct obstacle {
        std::string id;
        double size;
        double mspeed;
        std::vector<Node> sections;

        obstacle() {
            id = -1;
            size = CN_DEFAULT_SIZE;
            mspeed = CN_DEFAULT_MSPEED;
        }
    };

    struct section {
        section(int _i1 = -1, int _j1 = -1, int _k1 = -1,
                int _i2 = -1, int _j2 = -1, int _k2 = -1,
                double _g1 = -1, double _g2 = -1)
                : i1(_i1), j1(_j1), k1(_k1), i2(_i2), j2(_j2), k2(_k2), g1(_g1), g2(_g2) {}

        section(const Node &a, const Node &b) : i1(a.i), j1(a.j), k1(a.k), i2(b.i), j2(b.j), k2(b.k),
                                                g1(a.g), g2(b.g) {}

        int i1;
        int j1;
        int k1;
        int i2;
        int j2;
        int k2;

        int agent_id;
//        double size;
        double g1;
        double g2;//is needed for goal and wait actions
        double mspeed;

        bool operator==(const section &comp) const { return (i1 == comp.i1 && j1 == comp.j1 && k1 == comp.k1 && g1 == comp.g1); }

    };

    class Vector3D {
    public:
        Vector3D(double _i = 0.0, double _j = 0.0, double _k = 0.0) : i(_i), j(_j), k(_k) {}

        double i, j, k;

        inline Vector3D operator+(const Vector3D &vec) { return Vector3D(i + vec.i, j + vec.j, k + vec.k); }

        inline Vector3D operator-(const Vector3D &vec) { return Vector3D(i - vec.i, j - vec.j, k - vec.k); }

        inline Vector3D operator-() { return Vector3D(-i, -j, -k); }

        inline Vector3D operator/(const double &num) { return Vector3D(i / num, j / num, k / num); }

        inline Vector3D operator*(const double &num) { return Vector3D(i * num, j * num, k * num); }

        inline double operator*(const Vector3D &vec) { return i * vec.i + j * vec.j + k * vec.k; }

        inline void operator+=(const Vector3D &vec) {
            i += vec.i;
            j += vec.j;
            k += vec.k;
        }

        inline void operator-=(const Vector3D &vec) {
            i -= vec.i;
            j -= vec.j;
            k -= vec.k;
        }

        double norm(){
            return sqrt(i * i + j * j + k * k);
        }

        Vector3D normalize(){
            double n = norm();
            return Vector3D(i / n, j / n, k / n);
        }

        Vector3D cross(const Vector3D &vec){
            return Vector3D(j*vec.k-k*vec.j, k*vec.i-i*vec.k, i*vec.j-j*vec.i);
        }
    };

    class Point {
    public:
        double i;
        double j;
        double k;

        Point(double _i = 0.0, double _j = 0.0, double _k = 0.0) : i(_i), j(_j), k(_k) {}

        Point operator-(Point &p) { return Point(i - p.i, j - p.j, k - p.k); }

        int operator==(Point &p) { return (i == p.i) && (j == p.j) && (k == p.k); }

        int classify(Point &pO, Point &p1) {
            Point p2 = *this;
            Vector3D a = Vector3D((p1 - pO).i, (p1 - pO).j, (p1 - pO).k);
            Vector3D b = Vector3D((p2 - pO).i, (p2 - pO).j, (p2 - pO).k);
            Vector3D sa = a.cross(b);
            if (sa.k > 0.0)
                return 1;//LEFT;
            if (sa.k < 0.0)
                return 2;//RIGHT;
            if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0) || (a.k * b.k < 0.0))
                return 3;//BEHIND;
            if (a.norm() < b.norm())
                return 4;//BEYOND;
            if (pO == p2)
                return 5;//ORIGIN;
            if (p1 == p2)
                return 6;//DESTINATION;
            return 7;//BETWEEN;
        }

        int direction(Point &p1){
            Point p0 = *this;
            int delta_x = p1.i - p0.i;
            int delta_y = p1.j - p0.j;
            int delta_z = p1.k - p0.k;

            if((delta_x != 0 && delta_y != 0) || (delta_x != 0 && delta_z != 0) || (delta_y != 0 && delta_z != 0)){
                std::cerr << "sipp: invalid diagonal move";
                return 4;
            }

            if(delta_x > 0)
                return 1;
            if(delta_x < 0)
                return -1;
            if(delta_y > 0)
                return 2;
            if(delta_y < 0)
                return -2;
            if(delta_z > 0)
                return 3;
            if(delta_z < 0)
                return -3;
            return 0;
        }
    };
}
#endif

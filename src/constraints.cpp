#include <structs.h>
#include "constraints.h"
using namespace SIPP;

Constraints::Constraints(int dimx, int dimy, int dimz)
{
    safe_intervals.resize(dimx);
    for(int i = 0; i < dimx; i++)
    {
        safe_intervals[i].resize(dimy);
        for(int j = 0; j < dimy; j++)
        {
            safe_intervals[i][j].resize(dimz);
            for(int k = 0; k < dimz; k++) {
                safe_intervals[i][j][k].resize(0);
                safe_intervals[i][j][k].push_back({0, CN_INFINITY});
            }
        }
    }
    constraints.resize(dimx);
    for(int i = 0; i < dimx; i++)
    {
        constraints[i].resize(dimy);
        for(int j = 0; j < dimy; j++)
        {
            constraints[i][j].resize(dimz);
            for(int k = 0; k < dimz; k++) {
                constraints[i][j][k].resize(0);
            }
        }
    }
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

double Constraints::minDist_LineSegment(Point A, Point C, Point D)
{
    Vector3D a(C.i - A.i, C.j - A.j, C.k - A.k);
    Vector3D b(D.i - A.i, D.j - A.j, D.k - A.k);

    double min_dist = std::min(sqrt(a * a), sqrt(b * b));
    Vector3D n = (b - a).normalize();
    Vector3D c = a - n * (a * n);
    if ((c - a) * (c - b) < 0) {
        min_dist = std::min(min_dist, c.norm());
    }
    return min_dist;
}

double Constraints::minDist_StraightLine(Point A, Point C, Point D)
{
    Vector3D a(C.i - A.i, C.j - A.j, C.k - A.k);
    Vector3D b(D.i - A.i, D.j - A.j, D.k - A.k);
    Vector3D n = (b - a).normalize();
    Vector3D c = a - n * (a * n);
    return c.norm();
}

double Constraints::boxDistance(Vector3D A){
    return std::max({std::abs(A.i), std::abs(A.j), std::abs(A.k)});
}

bool Constraints::isMinBoxPointInLineSegment(const Vector3D& C, const Vector3D& D){
    double boxDist_C = boxDistance(C);
    double boxDist_D = boxDistance(D);
    double minBoxDist;
    if(boxDist_C < boxDist_D) {
        minBoxDist = boxDist_C;
        if (std::abs(C.i) == minBoxDist) {
            return C.i * (D.i - C.i) < 0;
        } else if (std::abs(C.j) == minBoxDist) {
            return C.j * (D.j - C.j) < 0;
        } else {
            return C.k * (D.k - C.k) < 0;
        }
    }
    else{
        minBoxDist = boxDist_D;
        if (std::abs(D.i) == minBoxDist) {
            return D.i * (C.i - D.i) < 0;
        } else if (std::abs(D.j) == minBoxDist) {
            return D.j * (C.j - D.j) < 0;
        } else {
            return D.k * (C.k - D.k) < 0;
        }
    }
}

double Constraints::minBoxDist_LineSegment(Vector3D C, Vector3D D){
    double min_dist;
    double boxDist_C = boxDistance(C);
    double boxDist_D = boxDistance(D);

    if(boxDist_C < boxDist_D) {
        min_dist = boxDist_C;
    }
    else{
        min_dist = boxDist_D;
    }

    if(!isMinBoxPointInLineSegment(C, D)){
        return min_dist;
    }
    double d_xy = 0, d_yz = 0, d_zx = 0;
    if(D.j - C.j != 0 || D.i - C.i != 0){
        d_xy = std::abs(C.j * (D.i - C.i) - C.i * (D.j - C.j)) / (std::abs(D.j - C.j) + std::abs(D.i - C.i));
    }
    if(D.k - C.k != 0 || D.j - C.j != 0){
        d_yz = std::abs(C.k * (D.j - C.j) - C.j * (D.k - C.k)) / (std::abs(D.k - C.k) + std::abs(D.j - C.j));
    }
    if(D.i - C.i != 0 || D.k - C.k != 0){
        d_zx = std::abs(C.i * (D.k - C.k) - C.k * (D.i - C.i)) / (std::abs(D.i - C.i) + std::abs(D.k - C.k));
    }
    min_dist = std::min(min_dist, std::max(d_xy, std::max(d_yz, d_zx)));
    return min_dist;
}

void Constraints::resetConstraints(int dimx, int dimy, int dimz) {
    constraints.resize(dimx);
    for(int i = 0; i < dimx; i++)
    {
        constraints[i].resize(dimy);
        for(int j = 0; j < dimy; j++)
        {
            constraints[i][j].resize(dimz);
            for(int k = 0; k < dimz; k++) {
                constraints[i][j][k].resize(0);
            }
        }
    }
}

void Constraints::resetSafeIntervals(int dimx, int dimy, int dimz)
{
    safe_intervals.resize(dimx);
    for(int i = 0; i < dimx; i++)
    {
        safe_intervals[i].resize(dimy);
        for(int j = 0; j < dimy; j++)
        {
            for(int k = 0; k < dimz; k++) {
                safe_intervals[i][j][k].resize(0);
                safe_intervals[i][j][k].push_back({0, CN_INFINITY});
            }
        }
    }
}

void Constraints::updateConstraints(const Map &map){
    std::vector<std::array<int,3>> cells;
    for(auto sec: section_list) {
        LineOfSight los(collision_models[sec.agent_id]);
        cells = los.getCellsCrossedByLine(sec.i1, sec.j1, sec.k1, sec.i2, sec.j2, sec.k2, map);
        for (auto cell: cells)
            if(map.CellOnGrid(cell[0], cell[1], cell[2]) && !map.CellIsObstacle(cell[0], cell[1], cell[2], 0)) {
                constraints[cell[0]][cell[1]][cell[2]].push_back(sec);
            }
    }
}

void Constraints::updateCellSafeIntervals(std::array<int, 3> cell)
{
    if(safe_intervals[cell[0]][cell[1]][cell[2]].size() > 1)
        return;
    LineOfSight los;
    std::vector<std::array<int,3>> cells = los.getCells(cell[0], cell[1], cell[2]);
    std::vector<section> secs;
    for(int iter = 0; iter < cells.size(); iter++)
        for(int l = 0; l < constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]][l]) == secs.end())
                secs.push_back(constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]][l]);

    std::vector<std::pair<double, double>> intervals;
    int i2(cell[0]), j2(cell[1]), k2(cell[2]);
    Point point(i2, j2, k2);
    for(int iter = 0; iter < secs.size(); iter++) {
        section sec = secs[iter];
        int i0(secs[iter].i1), j0(secs[iter].j1), k0(secs[iter].k1),
            i1(secs[iter].i2), j1(secs[iter].j2), k1(secs[iter].k2);
        Point p0(i0, j0, k0), p1(i1, j1, k1);

        if (!hasBoxPassed(point, p0, p1, sec.agent_id))
            continue;

        std::pair<double, double> interval;
        int dir = p0.direction(p1);
        double r = collision_models[sec.agent_id].r;
        double a = collision_models[sec.agent_id].a;
        double b = collision_models[sec.agent_id].b;

        if (isPointInBox(point, p0, sec.agent_id)) {
            if (isPointInBox(point, p1, sec.agent_id)) {
                interval.first = sec.g1;
                interval.second = sec.g2;
            }

            if (dir == 1) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (point.i - p0.i + r) / sec.mspeed;
            }
            if (dir == -1) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (p0.i - point.i + r) / sec.mspeed;
            }
            if (dir == 2) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (point.j - p0.j + r) / sec.mspeed;
            }
            if (dir == -2) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (p0.j - point.j + r) / sec.mspeed;
            }
            if (dir == 3) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (point.k - p0.k + b) / sec.mspeed;
            }
            if (dir == -3) {
                interval.first = sec.g1;
                interval.second = sec.g1 + (p0.k - point.k + a) / sec.mspeed;
            }
        } else if (isPointInBox(point, p1, sec.agent_id)) {
            if (dir == 1) {
                interval.first = sec.g2 - (p1.i - point.i + r) / sec.mspeed;
                interval.second = sec.g2;
            }
            if (dir == -1) {
                interval.first = sec.g2 - (point.i - p1.i + r) / sec.mspeed;
                interval.second = sec.g2;
            }
            if (dir == 2) {
                interval.first = sec.g2 - (p1.j - point.j + r) / sec.mspeed;
                interval.second = sec.g2;
            }
            if (dir == -2) {
                interval.first = sec.g2 - (point.j - p1.j + r) / sec.mspeed;
                interval.second = sec.g2;
            }
            if (dir == 3) {
                interval.first = sec.g2 - (p1.k - point.k + a) / sec.mspeed;
                interval.second = sec.g2;
            }
            if (dir == -3) {
                interval.first = sec.g2 - (point.k - p1.k + b) / sec.mspeed;
                interval.second = sec.g2;
            }
        } else {
            if (dir == 1) {
                interval.first = sec.g1 + (point.i - p0.i - r) / sec.mspeed;
                interval.second = sec.g1 + (point.i - p0.i + r) / sec.mspeed;
            }
            if (dir == -1) {
                interval.first = sec.g1 + (p1.i - point.i - r) / sec.mspeed;
                interval.second = sec.g1 + (p1.i - point.i + r) / sec.mspeed;
            }
            if (dir == 2) {
                interval.first = sec.g1 + (point.j - p0.j - r) / sec.mspeed;
                interval.second = sec.g1 + (point.j - p0.j + r) / sec.mspeed;
            }
            if (dir == -2) {
                interval.first = sec.g1 + (p1.j - point.j - r) / sec.mspeed;
                interval.second = sec.g1 + (p1.j - point.j + r) / sec.mspeed;
            }
            if (dir == 3) {
                interval.first = sec.g1 + (point.k - p0.k - a) / sec.mspeed;
                interval.second = sec.g1 + (point.k - p0.k + b) / sec.mspeed;
            }
            if (dir == -3) {
                interval.first = sec.g1 + (p0.k - point.k - b) / sec.mspeed;
                interval.second = sec.g1 + (p0.k - point.k + a) / sec.mspeed;
            }
        }

        int j = 0;
        while(j < intervals.size()){
            if(interval.second < intervals[j].first || interval.first > intervals[j].second){
                j++;
            }
            interval.first = std::min(interval.first, intervals[j].first);
            interval.second = std::max(interval.second, intervals[j].second);
            intervals.erase(intervals.begin() + j);
        }
        intervals.emplace_back(interval);
    }

    for(auto interval : intervals)
    {
        for(unsigned int j = 0; j < safe_intervals[i2][j2][k2].size(); j++)
        {
            if(safe_intervals[i2][j2][k2][j].first <= interval.first && safe_intervals[i2][j2][k2][j].second >= interval.first)
            {
                if(safe_intervals[i2][j2][k2][j].first == interval.first)
                {
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, {safe_intervals[i2][j2][k2][j].first,safe_intervals[i2][j2][k2][j].first});
                    j++;
                    if(safe_intervals[i2][j2][k2][j].second < interval.second)
                        safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                    else
                        safe_intervals[i2][j2][k2][j].first = interval.second;
                }
                else if(safe_intervals[i2][j2][k2][j].second < interval.second)
                    safe_intervals[i2][j2][k2][j].second = interval.first;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][k2][j].first;
                    new1.second = interval.first;
                    new2.first = interval.second;
                    new2.second = safe_intervals[i2][j2][k2][j].second;
                    safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, new2);
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, new1);
                }
            }
            else if(safe_intervals[i2][j2][k2][j].first >= interval.first && safe_intervals[i2][j2][k2][j].first < interval.second)
            {
                if(safe_intervals[i2][j2][k2][j].first == interval.first)
                {
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, {safe_intervals[i2][j2][k2][j].first,safe_intervals[i2][j2][k2][j].first});
                    j++;
                }
                if(safe_intervals[i2][j2][k2][j].second < interval.second)
                {
                    safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                }
                else
                {
                    safe_intervals[i2][j2][k2][j].first = interval.second;
                }
            }
        }
    }
}

std::vector<std::pair<double, double> > Constraints::getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int dimy, int dimz)
{
    std::vector<std::pair<double, double> > intervals(0);
    auto range = close.equal_range(curNode.i*dimy*dimz + curNode.j*dimz + curNode.k);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j][curNode.k].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][curNode.k][i].second >= curNode.g
                && safe_intervals[curNode.i][curNode.j][curNode.k][i].first <= (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
        {
            bool has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.first == safe_intervals[curNode.i][curNode.j][curNode.k][i].first)
                if((it->second.g + tweight*fabs(curNode.heading - it->second.heading)/(180*rspeed)) - curNode.g < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    break;
                }
            if(!has)
                intervals.push_back(safe_intervals[curNode.i][curNode.j][curNode.k][i]);
        }
    return intervals;
}

std::vector<std::pair<double, double> > Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j][curNode.k];
}

void Constraints::addStartConstraint(int i, int j, int k, int size, std::vector<std::array<int, 3> > cells, int agent_id)
{
    section sec(i, j, k, i, j, k, 0, size);
    sec.agent_id = agent_id;
    for(auto cell: cells)
        constraints[cell[0]][cell[1]][cell[2]].insert(constraints[cell[0]][cell[1]][cell[2]].begin(), sec);
    return;
}

void Constraints::removeStartConstraint(std::vector<std::array<int, 3> > cells, int start_i, int start_j, int start_k)
{
    for(auto cell: cells)
        for(size_t iter = 0; iter < constraints[cell[0]][cell[1]][cell[2]].size(); iter++)
            if(constraints[cell[0]][cell[1]][cell[2]][iter].i1 == start_i
            && constraints[cell[0]][cell[1]][cell[2]][iter].j1 == start_j
            && constraints[cell[0]][cell[1]][cell[2]][iter].k1 == start_k
            && constraints[cell[0]][cell[1]][cell[2]][iter].g1 < CN_EPSILON)
            {
                constraints[cell[0]][cell[1]][cell[2]].erase(constraints[cell[0]][cell[1]][cell[2]].begin() + iter);
                iter--;
            }
    return;
}

void Constraints::addConstraints(const std::vector<Node> &sections, int agent_id, double mspeed, const Map &map)
{
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.agent_id = agent_id;
    sec.mspeed = mspeed;
    section_list.emplace_back(sec);

    for(unsigned int a = 1; a < sections.size(); a++)
    {
//        cells = los.getCellsCrossedByLine(sections[a-1].i, sections[a-1].j, sections[a-1].k, sections[a].i, sections[a].j, sections[a].k, map);
        sec = section(sections[a-1], sections[a]);
        sec.agent_id = agent_id;
        sec.mspeed = mspeed;
        section_list.emplace_back(sec);
//        for(unsigned int i = 0; i < cells.size(); i++)
//            constraints[cells[i][0]][cells[i][1]][cells[i][2]].push_back(sec);



        /*if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);*/
    }
}

std::vector<std::pair<double,double>> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const Map &map)
{
    std::vector<std::pair<double,double>> curNodeIntervals = getSafeIntervals(curNode, close, map.dimy, map.dimz);
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    EAT.clear();
    LineOfSight los;
    std::vector<std::array<int,3>> cells = los.getCellsCrossedByLine(curNode.i, curNode.j, curNode.k, curNode.Parent->i, curNode.Parent->j, curNode.Parent->k, map);
    std::vector<section> sections(0);
    section sec;
    for(unsigned int i = 0; i < cells.size(); i++)
        for(unsigned int j = 0; j < constraints[cells[i][0]][cells[i][1]][cells[i][2]].size(); j++)
        {
            sec = constraints[cells[i][0]][cells[i][1]][cells[i][2]][j];
            if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.second + curNode.g - curNode.Parent->g))
                continue;
            if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                sections.push_back(sec);
        }
    auto range = close.equal_range(curNode.i*map.dimy*map.dimz + curNode.j*map.dimz + curNode.k);

    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        std::pair<double,double> cur_interval(curNodeIntervals[i]);
        if(cur_interval.first < curNode.g)
            cur_interval.first = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.first > startTimeA + curNode.g - curNode.Parent->g)
            startTimeA = cur_interval.first - curNode.g + curNode.Parent->g;
        unsigned int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;

            if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
            {
                double offset = 1.0;
                startTimeA += offset;
                cur_interval.first += offset;
                j = 0;//start to check all constraints again, because time has changed
                if(goal_collision || cur_interval.first > cur_interval.second || startTimeA > curNode.Parent->interval.second)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    i--;
                    break;
                }
            }
            else
                j++;
        }
        if(j == sections.size())
        {
            bool has = false;
            for(auto rit = range.first; rit != range.second; rit++)
                if(rit->second.interval.first == curNodeIntervals[i].first)
                if((rit->second.g + tweight*fabs(curNode.heading - rit->second.heading)/(180*rspeed) - cur_interval.first) < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    curNodeIntervals.erase(curNodeIntervals.begin()+i);
                    i--;
                    break;
                }
            if(!has)
                EAT.push_back(cur_interval.first);
        }
    }
    return curNodeIntervals;
}

bool Constraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
{
    if(curNode.Parent->i == 9 && curNode.Parent->k == 0 && curNode.i == 10 && curNode.k == 0){
        int debug = 0;
    }


    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    if(startTimeA > endTimeB || startTimeB > endTimeA)
        return false;
    Vector3D A(curNode.Parent->i,curNode.Parent->j, curNode.Parent->k);
    Vector3D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g), (curNode.k - curNode.Parent->k)/(curNode.g - curNode.Parent->g));
    Vector3D B(constraint.i1, constraint.j1, constraint.k1);
    Vector3D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1), (constraint.k2 - constraint.k1)/(constraint.g2 - constraint.g1));
    if(startTimeB > startTimeA)
    {
      // Move A to the same time instant as B
      A += VA*(startTimeB - startTimeA);
      startTimeA = startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }

    double r = collision_models[constraint.agent_id].r;
    double a = collision_models[constraint.agent_id].a;
    double b = collision_models[constraint.agent_id].b;

    Vector3D C = (B - A + Vector3D(0, 0, (a-b)/2));
    C.k = C.k * 2 * r / (a + b);
    Vector3D D = (B - A + (VB - VA) * (std::min(endTimeB, endTimeA) - startTimeA) + Vector3D(0, 0, (a-b)/2));
    D.k = D.k * 2 * r / (a + b);
    double minBoxDist = minBoxDist_LineSegment(C, D);

    if(minBoxDist < r - CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}

bool Constraints::hasBoxPassed(Point A, Point C, Point D, int section_agent_id){
    int direction = C.direction(D);

    double r = collision_models[section_agent_id].r;
    double a = collision_models[section_agent_id].a;
    double b = collision_models[section_agent_id].b;

    std::vector<double> box;
    box.resize(6);
    if(std::abs(direction) == 1){
        if(direction > 0){
            box[0] = C.i - r;
            box[3] = D.i + r;
        }
        else{
            box[0] = D.i - r;
            box[3] = C.i + r;
        }
        box[1] = C.j - r;
        box[2] = C.k - b;
        box[4] = C.j + r;
        box[5] = C.k + a;
    }
    if(std::abs(direction) == 2){
        if(direction > 0){
            box[1] = C.j - r;
            box[4] = D.j + r;
        }
        else{
            box[1] = D.j - r;
            box[4] = C.j + r;
        }
        box[0] = C.i - r;
        box[2] = C.k - b;
        box[3] = C.i + r;
        box[5] = C.k + a;
    }
    if(std::abs(direction) == 3){
        if(direction > 0){
            box[2] = C.i - b;
            box[5] = D.i + a;
        }
        else{
            box[2] = D.i - b;
            box[5] = C.i + a;
        }
        box[0] = C.i - r;
        box[1] = C.j - r;
        box[3] = C.i + r;
        box[4] = C.j + r;
    }

    return A.i > box[0] + CN_EPSILON && A.i < box[3] - CN_EPSILON
           && A.j > box[1] + CN_EPSILON && A.j < box[4] - CN_EPSILON
           && A.j > box[2] + CN_EPSILON && A.k < box[5] - CN_EPSILON;
}

bool Constraints::isPointInBox(Point A, Point C, int section_agent_id){
    double r = collision_models[section_agent_id].r;
    double a = collision_models[section_agent_id].a;
    double b = collision_models[section_agent_id].b;

    std::vector<double> box;
    box.resize(6);
    box[0] = C.i - r;
    box[1] = C.j - r;
    box[2] = C.k - b;
    box[3] = C.i + r;
    box[4] = C.j + r;
    box[5] = C.k + a;

    return A.i > box[0] + CN_EPSILON && A.i < box[3] - CN_EPSILON
           && A.j > box[1] + CN_EPSILON && A.j < box[4] - CN_EPSILON
           && A.j > box[2] + CN_EPSILON && A.k < box[5] - CN_EPSILON;
}


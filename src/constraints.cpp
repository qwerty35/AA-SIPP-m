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

void Constraints::updateCellSafeIntervals(std::array<int, 3> cell)
{
    if(safe_intervals[cell[0]][cell[1]][cell[2]].size() > 1)
        return;
    LineOfSight los(agentsize);
    std::vector<std::array<int, 3>> cells = los.getCells(cell[0], cell[1], cell[2]);
    std::vector<section> secs;
    for(int iter = 0; iter < cells.size(); iter++)
        for(int l = 0; l < constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]][l]) == secs.end())
                secs.push_back(constraints[cells[iter][0]][cells[iter][1]][cells[iter][2]][l]);
    for(int iter = 0; iter < secs.size(); iter++)
    {
        section sec = secs[iter];
        double radius = agentsize + sec.size;
        int i0(secs[iter].i1), j0(secs[iter].j1), k0(secs[iter].k1),
            i1(secs[iter].i2), j1(secs[iter].j2), k1(secs[iter].k2),
            i2(cell[0]), j2(cell[1]), k2(cell[2]);
        std::pair<double,double> interval;
        double dist, mindist;
        if(i0 == i1 && j0 == j1 && i0 == i2 && j0 == j2)
            mindist = 0;
        else
            mindist = minDist_LineSegment(Point(i2,j2,k2), Point(i0,j0,k0), Point(i1,j1,k1));
        if(mindist >= radius)
            continue;
        Point point(i2,j2), p0(i0,j0), p1(i1,j1);
        int cls = point.classify(p0, p1);
        dist = minDist_StraightLine(point, p0, p1);
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        double ha = sqrt(da - dist*dist);
        double size = sqrt(radius*radius - dist*dist);
        if(cls == 3)
        {
            interval.first = sec.g1;
            interval.second = sec.g1 + (sqrt(radius*radius - dist*dist) - ha)/sec.mspeed;
        }
        else if(cls == 4)
        {
            interval.first = sec.g2 - sqrt(radius*radius - dist*dist)/sec.mspeed + sqrt(db - dist*dist)/sec.mspeed;
            interval.second = sec.g2;
        }
        else if(da < radius*radius)
        {
            if(db < radius*radius)
            {
                interval.first = sec.g1;
                interval.second = sec.g2;
            }
            else
            {
                double hb = sqrt(db - dist*dist);
                interval.first = sec.g1;
                interval.second = sec.g2 - hb/sec.mspeed + size/sec.mspeed;
            }
        }
        else
        {
            if(db < radius*radius)
            {
                interval.first = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.second = sec.g2;
            }
            else
            {
                interval.first = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.second = sec.g1 + ha/sec.mspeed + size/sec.mspeed;
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
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

void Constraints::addStartConstraint(int i, int j, int k, int size, std::vector<std::array<int, 3> > cells, double agentsize)
{
    section sec(i, j, k, i, j, k, 0, size);
    sec.size = agentsize;
    for(auto cell: cells)
        constraints[cell[0]][cell[1]][cell[2]].insert(constraints[cell[0]][cell[1]][cell[2]].begin(),sec);
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

void Constraints::addConstraints(const std::vector<Node> &sections, double size, double mspeed, const Map &map)
{
    std::vector<std::array<int,3>> cells;
    LineOfSight los(size);
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.size = size;
    sec.mspeed = mspeed;
    cells = los.getCellsCrossedByLine(sec.i1, sec.j1, sec.k1, sec.i2, sec.j2, sec.k2, map);
    for(auto cell: cells)
        constraints[cell[0]][cell[1]][cell[2]].push_back(sec);
    if(sec.g1 == 0)
        for(auto cell: cells)
            safe_intervals[cell[0]][cell[1]][cell[2]].clear();
    for(unsigned int a = 1; a < sections.size(); a++)
    {
        cells = los.getCellsCrossedByLine(sections[a-1].i, sections[a-1].j, sections[a-1].k, sections[a].i, sections[a].j, sections[a].k, map);
        sec = section(sections[a-1], sections[a]);
        sec.size = size;
        sec.mspeed = mspeed;
        for(unsigned int i = 0; i < cells.size(); i++)
            constraints[cells[i][0]][cells[i][1]][cells[i][2]].push_back(sec);
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
    LineOfSight los(agentsize);
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
      A += VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }
    double r(constraint.size + agentsize + inflateintervals); //combined radius
    Vector3D w(B - A);
    double c(w*w - r*r);
    if(c < 0)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    } // Agents are currently colliding

    // Use the quadratic formula to detect nearest collision (if any)
    Vector3D v(VA - VB);
    double a(v*v);
    double b(w*v);

    double dscr(b*b - a*c);
    if(dscr <= 0)
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}

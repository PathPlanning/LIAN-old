#include "liansearch.h"
#include <cmath>
#include <chrono>
#include <time.h>
#include <list>

#include "gl_const.h"
#include "Bresenham.h"

LianSearch::~LianSearch() {
    delete[] open;
}


LianSearch::LianSearch(float angleLimit, int distance, float weight,
                       unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
                       float decreaseDistanceFactor, int distanceMin,
                       float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius) {
    this->angleLimit = angleLimit;
    this->distance = distance;
    this->weight = weight;
    this->stepLimit = steplimit;
    this->circleRadiusFactor = circleRadiusFactor;
    this->curvatureHeuristicWeight = curvatureHeuristicWeight;
    this->decreaseDistanceFactor = decreaseDistanceFactor;
    this->distanceMin = distanceMin;
    this->linecost = linecost;
    this->lesserCircle = lesserCircle;
    this->numOfParentsToIncreaseRadius = numOfParentsToIncreaseRadius;
    closeSize = 0;
    openSize = 0;
    srand(time(NULL));
}

void LianSearch::calculateCircle(int radius) {
    // радиус - радиус окружности в клетках

    LianSearch::circleNodes.clear();
    LianSearch::circleNodes.resize(listOfDistancesSize);
    for (int k = 0; k < listOfDistancesSize; k++) {
        radius = listOfDistances[k];
        LianSearch::circleNodes[k].clear();
        std::vector<Node> circleNodes;
        int x = 0;
        int y = radius;

        int delta = 2 - 2 * radius;
        int error = 0;
        Node node;
        while (y >= 0) {
            if (x > radius)
                x = radius;
            else if (x < -radius)
                x = -radius;
            if (y > radius)
                y = radius;
            else if (y < -radius)
                y = -radius;
            node.i = x;
            node.j = y;
            circleNodes.push_back(node);
            node.i = x;
            node.j = -y;
            circleNodes.push_back(node);
            node.i = -x;
            node.j = y;
            circleNodes.push_back(node);
            node.i = -x;
            node.j = -y;
            circleNodes.push_back(node);

            error = 2 * (delta + y) - 1;
            if ((delta < 0) && (error <= 0)) {
                delta += 2 * ++x + 1;
                continue;
            }

            error = 2 * (delta - x) - 1;
            if ((delta > 0) && (error > 0)) {
                delta += 1 - 2 * --y;
                continue;
            }
            x++;
            delta += 2 * (x - y);
            y--;
        }

        for (int i = 0; i < circleNodes.size(); i += 4)
            LianSearch::circleNodes[k].push_back(circleNodes[i]);
        for (int i = circleNodes.size() - 7; i > 0; i -= 4)
            LianSearch::circleNodes[k].push_back(circleNodes[i]);
        for (int i = 7; i < circleNodes.size(); i += 4)
            LianSearch::circleNodes[k].push_back(circleNodes[i]);
        for (int i = circleNodes.size() - 6; i > 0; i -= 4)
            LianSearch::circleNodes[k].push_back(circleNodes[i]);
        LianSearch::circleNodes[k].pop_back();
    }
}

bool LianSearch::checkLesserCircle(const cMap &Map, const Node &center, const float radius) {
    int x = center.j;
    int y = center.i;

    int squareRadius = (int) radius;

    for (int i = -1 * squareRadius; i <= squareRadius; i++) {
        for (int j = -1 * squareRadius; j <= squareRadius; j++) {
            if ((x + j < 0) || (x + j >= Map.width)) continue;

            if ((y + i < 0) || (y + i >= Map.height)) continue;

            if (Map.Grid[y + i][x + j] == CN_OBSTL) return false;
        }
    }

    return true;
}

void LianSearch::calculateDistances() {
    int curDistance = distance;
    if (decreaseDistanceFactor > 1)
        while (curDistance >= distanceMin) {
            listOfDistances.push_back(curDistance);
            curDistance = ceil(curDistance / decreaseDistanceFactor);
        }
    else
        listOfDistances.push_back(curDistance);
    listOfDistancesSize = listOfDistances.size();
}

void LianSearch::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal) {
    Liner drawer(&line);
    drawer.append_line(start, goal);
}

bool LianSearch::checkLineSegment(const cMap &map, const Node &start, const Node &goal) {
    LineOfSight checker(map);
    return checker.line_of_sight(start, goal);
}

bool LianSearch::stopCriterion() {
    if (openSize == 0) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }


    if ((closeSize > stepLimit) && (stepLimit > 0)) {
        std::cout << "Algorithm esceeded step limit!" << std::endl;
        return true;
    }


    return false;
}

double LianSearch::calculateDistanceFromCellToCell(const Node &from, const Node &to) const {
    int delta_i, delta_j, delta_z;
    delta_i = abs(to.i - from.i);
    delta_j = abs(to.j - from.j);
    delta_z = abs(to.z - from.z);

    return sqrt(double(delta_i * delta_i) + double(delta_j * delta_j) + double(delta_z * delta_z));
}

void LianSearch::addOpen(Node &newNode) {
    std::list<Node>::iterator iter, pos;

    bool posFound = false;

    pos = open[newNode.i].List.end();

    if (open[newNode.i].List.size() == 0) {
        open[newNode.i].List.push_back(newNode);
        openSize++;
        return;
    }

    for (iter = open[newNode.i].List.begin(); iter != open[newNode.i].List.end(); ++iter) {
        if ((iter->F >= newNode.F) && (!posFound)) {
            pos = iter;
            posFound = true;
        }

        if (iter->i == newNode.i && iter->j == newNode.j && iter->z == newNode.z)
            if ((iter->Parent->i == newNode.Parent->i) && (iter->Parent->j == newNode.Parent->j) &&
                (iter->Parent->z == newNode.Parent->z)) {
                if (newNode.F >= iter->F) {
                    return;
                } else {
                    if (pos == iter) {
                        iter->g = newNode.g;
                        iter->F = newNode.F;
                        iter->c = newNode.c;
                        iter->radius = newNode.radius;
                        return;
                    }
                    open[newNode.i].List.erase(iter);
                    openSize--;
                    break;
                }
            }
    }
    openSize++;
    open[newNode.i].List.insert(pos, newNode);
}

SearchResult LianSearch::startSearch(cLogger *Log, const cMap &Map) {
    auto start_time = std::chrono::high_resolution_clock::now();
    decltype(start_time) finish_time;
    calculateDistances();

    std::cout << "List of distances :";
    for (int i = 0; i < listOfDistancesSize; i++) {
        std::cout << " " << listOfDistances[i];
    }
    std::cout << std::endl;
    open = new cList[Map.height];
    Node curNode(Map.start_i, Map.start_j, Map.start_z, 0.0, 0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * linecost * calculateDistanceFromCellToCell(curNode, Node(Map.goal_i, Map.goal_j, Map.goal_z));
    bool pathFound = false;
    open[curNode.i].List.push_back(curNode);
    openSize++;
    calculateCircle((int) curNode.radius);
    // основной цикл поиска
    while (!stopCriterion()) {
        curNode = findMin(Map.height);
        open[curNode.i].List.pop_front();
        openSize--;
        close.insert({curNode.i * Map.width + curNode.j, curNode});
        closeSize++;
        //Если текущая точка - целевая, цикл поиска завершается
        if (curNode.i == Map.goal_i && curNode.j == Map.goal_j) {
            pathFound = true;
            break;
        }
        if (!expand(curNode, Map) && listOfDistancesSize > 1)
            while (curNode.radius > listOfDistances[listOfDistancesSize - 1])
                if (tryToDecreaseRadius(curNode, Map.width))
                    if (expand(curNode, Map))
                        break;
        if (Log->loglevel >= CN_LOGLVL_LOW)
            Log->writeToLogOpenClose(open, close, Map.height);
    }
    if (Log->loglevel == CN_LOGLVL_MED)
        Log->writeToLogOpenClose(open, close, Map.height);
    if (pathFound) {
        float maxAngle = makeAngles(curNode);
        makePrimaryPath(curNode);

        finish_time = std::chrono::high_resolution_clock::now();

        makeSecondaryPath(curNode);
        sresult.pathfound = true;
        sresult.pathlength = curNode.g;
        sresult.nodescreated = openSize + closeSize;
        sresult.numberofsteps = closeSize;
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.maxAngle = maxAngle;
        sresult.sections = hppath.List.size() - 1;
        return sresult;
    } else {
        sresult.time = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count();
        sresult.time /= 1000000000;
        sresult.pathfound = false;
        sresult.nodescreated = closeSize;
        sresult.numberofsteps = closeSize;
        return sresult;
    }
}

int LianSearch::tryToIncreaseRadius(Node curNode) {
    bool change = false;
    int i, k = 0;
    while (k < numOfParentsToIncreaseRadius) {
        if (curNode.Parent != NULL)
            if (curNode.radius == curNode.Parent->radius) {
                k++;
                curNode = *curNode.Parent;
                continue;
            }
        break;
    }
    if (k == numOfParentsToIncreaseRadius) {
        for (i = listOfDistancesSize - 1; i >= 0; i--)
            if (curNode.radius == listOfDistances[i])
                break;
        if (i > 0)
            change = true;
    }
    if (change)
        return listOfDistances[i - 1];
    else
        return curNode.radius;
}

bool LianSearch::expand(const Node curNode, const cMap &Map) {
    int k;
    for (k = 0; k < listOfDistancesSize; k++)
        if (listOfDistances[k] == curNode.radius)
            break;
    std::vector<Node> circleNodes = LianSearch::circleNodes[k];
    int succi, succj;
    int scalarProduct;
    bool successorsIsFine = false, inclose;
    float cosAngle, angle, curvature;
    int p_i;
    if (curNode.Parent != NULL) {
        std::vector<Node> successors;
        std::vector<int> succs;
        Node succ;
        for (p_i = 0; p_i < circleNodes.size(); p_i++)
            if (curNode.Parent->i == curNode.i + circleNodes[p_i].i &&
                curNode.Parent->j == curNode.j + circleNodes[p_i].j)
                break;
        if (p_i < circleNodes.size() / 2)
            p_i += circleNodes.size() / 2;
        else
            p_i -= circleNodes.size() / 2;
        int k1 = p_i + 1;
        int k2 = p_i - 1;
        succs.push_back(p_i);
        for (int i = 0; i < circleNodes.size(); i++) {
            if (k1 >= circleNodes.size())
                k1 = 0;
            if (k2 < 0)
                k2 = circleNodes.size() - 1;
            succs.push_back(k1);
            succs.push_back(k2);
            k1++;
            k2--;
            if (succs.size() >= circleNodes.size() / 2)
                break;
        }
        for (int i = 0; i < circleNodes.size() / 2; i++) {
            succi = curNode.i + circleNodes[succs[i]].i;
            succj = curNode.j + circleNodes[succs[i]].j;

            if ((succi < 0) || (succi >= Map.height))
                continue;
            if ((succj < 0) || (succj >= Map.width))
                continue;
            if (Map.Grid[succi][succj] == CN_OBSTL)
                continue;
            scalarProduct = (curNode.j - curNode.Parent->j) * (succj - curNode.j) +
                            (curNode.Parent->i - curNode.i) * (curNode.i - succi);
            cosAngle = (float) scalarProduct / calculateDistanceFromCellToCell(curNode.i, curNode.j, succi, succj);
            cosAngle = (float) cosAngle / (curNode.g - curNode.Parent->g);
            if (cosAngle > 1)
                cosAngle = 1;
            if (cosAngle < -1)
                cosAngle = -1;

            angle = acos(cosAngle);
            curvature = fabs(angle);
            angle = angle * 180 / CN_PI_CONSTANT;

            if (fabs(angle) > angleLimit)
                break;
            succ.i = circleNodes[succs[i]].i;
            succ.j = circleNodes[succs[i]].j;
            successors.push_back(succ);
        }
        circleNodes = successors;

    }
    for (unsigned int i = 0; i <= circleNodes.size(); ++i) {
        Node newNode;
        // на итерации цикла, когда i == circleNodes.size(),проверяем целевую точку
        if (i == circleNodes.size()) {
            if (calculateDistanceFromCellToCell(curNode, Node(Map.goal_i, Map.goal_j, Map.goal_z)) <= curNode.radius) {
                succi = Map.goal_i;
                succj = Map.goal_j;
                if (curNode.Parent != NULL) {
                    scalarProduct = (curNode.j - curNode.Parent->j) * (succj - curNode.j) +
                                    (curNode.Parent->i - curNode.i) * (curNode.i - succi);
                    cosAngle =
                            (float) scalarProduct / calculateDistanceFromCellToCell(curNode.i, curNode.j, succi, succj);
                    cosAngle = (float) cosAngle / calculateDistanceFromCellToCell(*(curNode.Parent), curNode);
                    angle = acos(cosAngle);
                    curvature = fabs(angle);
                    angle = angle * 180 / CN_PI_CONSTANT;
                    if (fabs(angle) > angleLimit)
                        continue;
                }
            } else
                continue;
        } else {
            succi = curNode.i + circleNodes[i].i;
            succj = curNode.j + circleNodes[i].j;
        }
        if ((succi < 0) || (succi >= Map.height))
            continue;
        if ((succj < 0) || (succj >= Map.width))
            continue;
        if (Map.Grid[succi][succj] == CN_OBSTL)
            continue;

        if (circleRadiusFactor > 0) {
            float sin_gamma = (float) (succi - curNode.i) /
                              calculateDistanceFromCellToCell(curNode.i, curNode.j, succi, succj);

            float cos_gamma = (float) (succj - curNode.j) /
                              calculateDistanceFromCellToCell(curNode.i, curNode.j, succi, succj);

            float circleRadius = (float) distance / (2 * sin(CN_PI_CONSTANT * angleLimit / 360));

            float circleCenter_x = succj + circleRadius * sin_gamma;
            float circleCenter_y = succi - circleRadius * cos_gamma;

            float incircleCircumcircleFactor = cos(CN_PI_CONSTANT * angleLimit / 360);

            if (pow(circleCenter_x - Map.goal_j, 2) + pow(circleCenter_y - Map.goal_i, 2) <
                circleRadiusFactor * pow(incircleCircumcircleFactor * circleRadius, 2))
                continue;

            circleCenter_x = succj - circleRadius * sin_gamma;
            circleCenter_y = succi + circleRadius * cos_gamma;

            if (pow(circleCenter_x - Map.goal_j, 2) + pow(circleCenter_y - Map.goal_i, 2) <
                circleRadiusFactor * pow(incircleCircumcircleFactor * circleRadius, 2))
                continue;
        }

        newNode.i = succi;
        newNode.j = succj;
        newNode.Parent = &(close.find(curNode.i * Map.width + curNode.j)->second);
        newNode.radius = newNode.Parent->radius;
        newNode.pathToParent = false;
        newNode.g = newNode.Parent->g + linecost * calculateDistanceFromCellToCell(curNode, newNode);
        newNode.c = newNode.Parent->c + curvature;
        newNode.F =
                newNode.g + weight * linecost * calculateDistanceFromCellToCell(succi, succj, Map.goal_i, Map.goal_j)
                + curvatureHeuristicWeight * distance * newNode.c;
        newNode.pathToParent = checkLineSegment(Map, *newNode.Parent, newNode);


        if (lesserCircle)
            if (newNode.pathToParent && ((curNode.i != Map.goal_i) || (curNode.j != Map.goal_j)))
                newNode.pathToParent = checkLesserCircle(Map, curNode, CN_PTD_LR);
        if (newNode.pathToParent) {
            std::unordered_multimap<int, Node>::const_iterator it = close.find(newNode.i * Map.width + newNode.j);
            if (it != close.end()) {
                inclose = false;
                auto range = close.equal_range(it->first);
                for (auto it = range.first; it != range.second; it++) {
                    if (it->second.Parent)
                        if (it->second.Parent->i == curNode.i && it->second.Parent->j == curNode.j)
                            inclose = true;
                }
                if (!inclose) {
                    if (listOfDistancesSize > 1)
                        newNode.radius = tryToIncreaseRadius(newNode);
                    addOpen(newNode);
                    successorsIsFine = true;
                }
            } else {
                if (listOfDistancesSize > 1)
                    newNode.radius = tryToIncreaseRadius(newNode);
                addOpen(newNode);
                successorsIsFine = true;
            }
        }
    }
    return successorsIsFine;
}

bool LianSearch::tryToDecreaseRadius(Node &curNode, int width) {
    int i;
    for (i = listOfDistancesSize - 1; i >= 0; i--)
        if (curNode.radius == listOfDistances[i])
            break;
    if (i < listOfDistancesSize - 1) {
        curNode.radius = listOfDistances[i + 1];
        auto it = close.find(curNode.i * width + curNode.j);
        auto range = close.equal_range(it->first);
        for (auto it = range.first; it != range.second; it++) {
            if (it->second.Parent)
                if (it->second.Parent->i == curNode.Parent->i && it->second.Parent->j == curNode.Parent->j) {
                    it->second.radius = listOfDistances[i + 1];
                    break;
                }
        }
        return true;
    }
    return false;
}

void LianSearch::makePrimaryPath(Node curNode) {
    int k = 0;
    hppath.List.push_front(curNode);
    curNode = *curNode.Parent;
    do {
        //if(angles[k]!=0)
        hppath.List.push_front(curNode);
        k++;
        curNode = *curNode.Parent;

    } while (curNode.Parent != NULL);
    hppath.List.push_front(curNode);
}

void LianSearch::makeSecondaryPath(Node curNode) {
    std::vector<Node> lineSegment;
    do {
        calculateLineSegment(lineSegment, *curNode.Parent, curNode);
        lppath.List.insert(lppath.List.begin(), ++lineSegment.begin(), lineSegment.end());
        curNode = *curNode.Parent;
    } while (curNode.Parent != NULL);
    lppath.List.push_front(*lineSegment.begin());
}

double LianSearch::makeAngles(Node curNode) {
    angles.clear();
    double angle = 0;
    double dis1 = 0;
    double dis2 = 0;
    double scalarProduct = 0;
    double cosAngle = 0;
    double maxAngle = 0;
    do {
        if ((curNode.Parent != NULL) && (curNode.Parent->Parent != NULL)) {
            dis1 = calculateDistanceFromCellToCell(curNode, *(curNode.Parent));
            dis2 = calculateDistanceFromCellToCell(*(curNode.Parent), *(curNode.Parent->Parent));

            scalarProduct = (curNode.j - curNode.Parent->j) * (curNode.Parent->j - curNode.Parent->Parent->j) +
                            (curNode.Parent->i - curNode.i) * (curNode.Parent->Parent->i - curNode.Parent->i);

            if ((dis1 != 0) && (dis2 != 0)) {
                cosAngle = (double) scalarProduct /
                           calculateDistanceFromCellToCell(curNode, *(curNode.Parent));
                cosAngle = (double) cosAngle /
                        calculateDistanceFromCellToCell(*(curNode.Parent), *(curNode.Parent->Parent));
                angle = acos(cosAngle);
                angle = angle * 180 / CN_PI_CONSTANT;
                if (angle > maxAngle)
                    maxAngle = angle;
                angles.push_back(angle);
            }
        }
        curNode = *curNode.Parent;
    } while (curNode.Parent != NULL);
    return maxAngle;
}

Node LianSearch::findMin(int size) {
    Node min;
    min.F = -1;
    for (int i = 0; i < size; i++) {
        if (open[i].List.size() != 0)
            if (open[i].List.begin()->F <= min.F || min.F == -1) {
                if (open[i].List.begin()->F == min.F) {
                    if (open[i].List.begin()->g >= min.g)
                        min = *open[i].List.begin();
                } else
                    min = *open[i].List.begin();
            }
    }
    return min;
}

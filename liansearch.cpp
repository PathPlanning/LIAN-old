#include "liansearch.h"
#include <cmath>
#include <chrono>
#include <functional>
#include <time.h>
#include <list>
#include <unordered_set>

#include "gl_const.h"
#include "Bresenham.h"

LianSearch::~LianSearch() {
    delete[] open;
}


LianSearch::LianSearch(double angleLimitDegree, int distance, float weight,
                       unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
                       float decreaseDistanceFactor, int distanceMin,
                       float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius) {
    //this->angleLimit = (angleLimitDegree / 180) * M_PI;
    this->angleLimit = angleLimitDegree;
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
/*
void LianSearch::calculateCircles() {
    circleNodes.resize(listOfDistances.size());
    int greatest = listOfDistances[0];
    int left, right, mid;
    long long rad, left_border_rad, right_border_rad;
    for (int x = 0; x <= greatest; ++x) {
        for (int y = 0; y <= greatest; ++y) {
            for (int z = 0; z <= greatest; ++z) {
                rad = x * x + y * y + z * z;
                rad *= 4;

                // Finding the radius which fits the point
                left = 0;
                right = listOfDistances.size();
                while (right - left > 1){
                    mid = (right + left) >> 1;
                    right_border_rad = 2 * listOfDistances[mid] + 1;
                    right_border_rad *= right_border_rad;
                    if (rad > right_border_rad) {
                        right = mid;
                    } else {
                        left = mid;
                    }
                }
                    // Checking if point really corresponds to radius with found number
                    right_border_rad = 2 * listOfDistances[left] + 1;
                    right_border_rad *= right_border_rad;
                    left_border_rad = 2 * listOfDistances[left] - 1;
                    left_border_rad *= left_border_rad;
                    if (left_border_rad <= rad && rad <= right_border_rad) {
                        circleNodes[left].push_back(Node(x, y, z));
                    }
            }
        }
    }
}*/

void LianSearch::calculateCircles() {
    // радиус - радиус окружности в клетках
    int radius;
    LianSearch::circleNodes.clear();
    LianSearch::circleNodes.resize(listOfDistancesSize);
    for (int k = 0; k < listOfDistancesSize; k++) {
        radius = listOfDistances[k];
        // LianSearch::circleNodes[k].clear();
        std::unordered_set<Node, std::hash<Node>, NodeCoordEqual> circle;
        int x = 0;
        int y = radius;

        // Processing quarter of circle
        int delta = 2 - 2 * radius;
        int error = 0;
        int z;
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
            for (int i = x; i != 0; --i) {
                z = static_cast<int>(round(sqrt(radius * radius - y * y - i * i)));
                for (int i_factor : {-1, 1}) {
                    for (int j_factor : {-1, 1}) {
                        for (int z_factor : {-1, 1}) {
                            node.i = i_factor * i;
                            node.j = j_factor * y;
                            node.z = z_factor * z;
                            circle.insert(node);
                        }
                    }
                }
            }

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

        circleNodes[k] = std::vector<Node>(circle.begin(), circle.end());
    }
}

// TODO rewrite checking lesser circle
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

    if (open[newNode.i].List.empty()) {
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

    calculateCircles();

    open = new cList[Map.height];
    Node curNode(Map.start_i, Map.start_j, Map.start_z, 0.0, 0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * linecost * calculateDistanceFromCellToCell(curNode, Node(Map.goal_i, Map.goal_j, Map.goal_z));
    open[curNode.i].List.push_back(curNode);
    openSize++;

    const Node *node_ptr;
    bool pathFound = false;
    // основной цикл поиска
    while (!stopCriterion()) {
        curNode = findMin(Map.height);
        open[curNode.i].List.pop_front();
        openSize--;
        node_ptr = &(*(close.insert(curNode)));
        closeSize++;
        //Если текущая точка - целевая, цикл поиска завершается
        if (curNode.i == Map.goal_i && curNode.j == Map.goal_j && curNode.z == Map.goal_z) {
            pathFound = true;
            break;
        }
        if (!expand(node_ptr, Map) && listOfDistancesSize > 1)
            while (curNode.radius > listOfDistances[listOfDistancesSize - 1])
                if (tryToDecreaseRadius(curNode, Map.width))
                    if (expand(node_ptr, Map))
                        break;
        // TODO correct openclose logging
        /*if (Log->loglevel >= CN_LOGLVL_LOW)
            Log->writeToLogOpenClose(open, close, Map.height);*/
    }
    // TODO correct openclose logging
    /*if (Log->loglevel >= CN_LOGLVL_LOW)
        Log->writeToLogOpenClose(open, close, Map.height);*/
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
    int k = 0;
    while (k < numOfParentsToIncreaseRadius) {
        if (curNode.Parent != nullptr)
            if (curNode.radius == curNode.Parent->radius) {
                k++;
                curNode = *curNode.Parent;
                continue;
            }
        break;
    }
    if (k == numOfParentsToIncreaseRadius) {
        int i = findRadiusInDistances(curNode.radius);
        if (i > 0) {
            return listOfDistances[i - 1];
        }
    }
    return curNode.radius;
}

bool LianSearch::expand(const Node *Node_ptr, const cMap &Map) {
    Node curNode = *Node_ptr;
    const Node goal = Node(Map.goal_i, Map.goal_j, Map.goal_z);

    int k;
    for (k = 0; k < listOfDistancesSize; k++)
        if (listOfDistances[k] == curNode.radius)
            break;
    const std::vector<Node> &curCircleNodes = LianSearch::circleNodes[k];
    Node successor_node;
    bool successorsIsFine = false, inclose, pathToParent;
    float cosAngle, angle, curvature;

    for (size_t pos = 0; pos <= curCircleNodes.size(); ++pos) {
        // On last iteration we check if goal point inside the circle and process it if possible
        if (pos == curCircleNodes.size()) {
            if (calculateDistanceFromCellToCell(curNode, goal) <= curNode.radius) {
                successor_node = Node(Map.goal_i, Map.goal_j, Map.goal_z);
            } else {
                continue;
            }
        } else {
            successor_node.i = curNode.i + curCircleNodes[pos].i;
            successor_node.j = curNode.j + curCircleNodes[pos].j;
            successor_node.z = curNode.z + curCircleNodes[pos].z;
        }

        if (!Map.NodeOnGrid(successor_node) || Map.NodeIsObstacle(successor_node)) {
            continue;
        }

        if (curNode.Parent != nullptr && !checkTurnAngle(*(curNode.Parent), curNode, successor_node)) {
            continue;
        }

        // I don't know what is it. Some extra criterion.
        /*
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
        }*/

        successor_node.Parent = Node_ptr;
        successor_node.radius = curNode.radius;
        successor_node.g = curNode.g + linecost * calculateDistanceFromCellToCell(curNode, successor_node);
        successor_node.c = curNode.c + curvature;
        successor_node.F =
                successor_node.g + weight * linecost * calculateDistanceFromCellToCell(successor_node, goal) +
                curvatureHeuristicWeight * distance * successor_node.c;


        pathToParent = checkLineSegment(Map, curNode, successor_node);

        if (lesserCircle)
            if (pathToParent && ((curNode.i != Map.goal_i) || (curNode.j != Map.goal_j) || (curNode.z != Map.goal_z)))
                pathToParent = checkLesserCircle(Map, curNode, CN_PTD_LR);
        if (pathToParent) {
            auto range = close.equal_range(successor_node);
            inclose = false;
            for (auto it = range.first; it != range.second; it++) {
                if (it->Parent != nullptr && it->Parent->i == curNode.i && it->Parent->j == curNode.j &&
                    it->Parent->z == curNode.z) {
                    inclose = true;
                    break;
                }
            }
            if (!inclose) {
                if (listOfDistancesSize > 1)
                    successor_node.radius = tryToIncreaseRadius(successor_node);
                addOpen(successor_node);
                successorsIsFine = true;
            }
        }
    }
    return successorsIsFine;
}

bool LianSearch::checkTurnAngle(const Node &start, const Node &middle, const Node &finish) const {
    double scalar_product =
            (middle.i - start.i) * (finish.i - middle.i) + (middle.j - start.j) * (finish.j - middle.j) +
            (middle.z - start.z) * (finish.z - middle.z);
    double cosAngle = scalar_product / calculateDistanceFromCellToCell(start, middle);
    cosAngle /= calculateDistanceFromCellToCell(middle, finish);

    if (cosAngle > 1)
        cosAngle = 1;
    if (cosAngle < -1)
        cosAngle = -1;

    double angle = fabs(acos(cosAngle)) * M_1_PI * 180 ;
    return (angle <= angleLimit);
}

int LianSearch::findRadiusInDistances(int radius) const {
    for (int i = 0; i < listOfDistances.size(); ++i) {
        if (listOfDistances[i] == radius) {
            return i;
        }
    }


    int left = 0;
    int right = listOfDistances.size();
    int mid;
    while (right - left > 1) {
        mid = (left + right) >> 1;
        std::cout << left << ' ' << right << ' ' << mid << '\n';
        if (listOfDistances[mid] > radius) {
            right = mid;
        } else {
            left = mid;
        }
    }
    std::cout << '\n';
    return left;
}

bool LianSearch::tryToDecreaseRadius(Node &curNode, int width) {
    int i = findRadiusInDistances(curNode.radius);
    if (i < listOfDistancesSize - 1) {
        curNode.radius = listOfDistances[i + 1];
        auto range = close.equal_range(curNode);
        for (auto it = range.first; it != range.second; it++) {
            if (it->Parent)
                if (it->Parent->i == curNode.Parent->i && it->Parent->j == curNode.Parent->j && it->Parent->z == curNode.Parent->z) {
                    // TODO write updating radius in close list
                    //it->radius = listOfDistances[i + 1];
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

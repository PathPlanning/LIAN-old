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
    delete open;
}


LianSearch::LianSearch(double angleLimitDegree, int distance, float weight,
                       unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
                       float decreaseDistanceFactor, int distanceMin,
                       float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius) {
    this->angleLimit = (angleLimitDegree / 180) * M_PI;
    //this->angleLimit = angleLimitDegree;
    this->cosLimit = cos(angleLimit);
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
    srand(time(NULL));
}


// Bit smarter brute force
void LianSearch::calculateCircles() {
    circleNodes.resize(listOfDistances.size());
    int greatest = listOfDistances[0];
    size_t left, right, mid;
    long long rad, left_border_rad, right_border_rad;
    Node node;
    for (int x = 0; x <= greatest; ++x) {
        for (int y = 0; y <= greatest; ++y) {
            for (int z = 0; z <= greatest; ++z) {
                rad = x * x + y * y + z * z;
                rad *= 4;

                // Searching the radius which fits the point
                for (left = 0; left < listOfDistances.size(); ++left) {
                    left_border_rad = (listOfDistances[left] << 1) - 1;
                    left_border_rad *= left_border_rad;
                    if (left_border_rad <= rad) {
                        break;
                    }
                }
                if (left < listOfDistances.size()) {
                    // Checking if point really corresponds to the found radius
                    right_border_rad = (listOfDistances[left] << 1) + 1;
                    right_border_rad *= right_border_rad;
                    if (rad <= right_border_rad) {
                        for (int x_factor : {-1, 1}) {
                            for (int j_factor : {-1, 1}) {
                                for (int z_factor : {-1, 1}) {
                                    node.i = x_factor * x;
                                    node.j = j_factor * y;
                                    node.z = z_factor * z;
                                    circleNodes[left].push_back(node);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


/* Brute force
void LianSearch::calculateCircles() {
    circleNodes.resize(listOfDistances.size());
    Node node;
    int radius;
    double cand_radius;
    for (size_t i = 0; i < listOfDistances.size(); ++i) {
        radius = listOfDistances[i];
        for (int x = 0; x <= radius; ++x) {
            for (int y = 0; y <= radius; ++y) {
                for (int z = 0; z <= radius; ++z) {
                    cand_radius = sqrt(x * x + y * y + z * z);
                    if (fabs(cand_radius - radius) > 0.5) {
                        continue;
                    }

                    for (int x_factor : {-1, 1}) {
                        for (int j_factor : {-1, 1}) {
                            for (int z_factor : {-1, 1}) {
                                node.i = x_factor * x;
                                node.j = j_factor * y;
                                node.z = z_factor * z;
                                circleNodes[i].push_back(node);
                            }
                        }
                    }
                }
            }
        }
    }
}*/

/* Partial integer calculations
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
*/

bool LianSearch::checkLesserCircle(const cMap &Map, const Node &center, const float radius) {
    int squareRadius = (int) radius;

    for (int i = -1 * squareRadius; i <= squareRadius; i++) {
        for (int j = -1 * squareRadius; j <= squareRadius; j++) {
            for (int z = -1 * squareRadius; j <= squareRadius; j++) {
                if (Map.CellOnGrid(center.i + i, center.j + j, center.z + z) &&
                    Map.CellIsObstacle(center.i + i, center.j + j, center.z + z)) {
                    return false;
                }
            }
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

bool LianSearch::checkLineSegment(const cMap &map, const Node &start, const Node &goal) {
    LineOfSight checker(map);
    return checker.line_of_sight(start, goal);
}

bool LianSearch::stopCriterion() {
    if (open->empty()) {
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

    return sqrt(delta_i * delta_i + delta_j * delta_j + delta_z * delta_z);
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

    open = new SortedList(Map.height);
    //open = new ClusteredHeap(Map.height);
    //open = new MinHeap(CN_SP_BT_GMAX, false);
    Node curNode(Map.start_i, Map.start_j, Map.start_z, 0.0, 0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * linecost * calculateDistanceFromCellToCell(curNode, Node(Map.goal_i, Map.goal_j, Map.goal_z));
    open->Insert(curNode);

    const Node *node_ptr;
    bool pathFound = false;
    // основной цикл поиска
    while (!stopCriterion()) {
        curNode = open->FindMin();
        open->DeleteMin();

        node_ptr = &(*(close.insert(curNode)));
        closeSize++;
        //Если текущая точка - целевая, цикл поиска завершается
        if (curNode.i == Map.goal_i && curNode.j == Map.goal_j && curNode.z == Map.goal_z) {
            pathFound = true;
            break;
        }
        if (!expand(node_ptr, Map) && listOfDistancesSize > 1) {
            while (node_ptr->radius > listOfDistances[listOfDistancesSize - 1]) {
                node_ptr = tryToDecreaseRadius(node_ptr, Map.width);
                if (expand(node_ptr, Map)) {
                    break;
                }
            }
        }
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
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.maxAngle = maxAngle;
        sresult.sections = hppath.List.size() - 1;
    } else {
        sresult.pathfound = false;
        sresult.pathlength = 0;
    }
    sresult.time = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count();
    sresult.time /= 1000000000;
    sresult.nodescreated = open->size() + closeSize;
    sresult.numberofsteps = closeSize;
    return sresult;
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
    const std::vector<Node> &curCircleNodes = circleNodes[k];
    Node successor_node;
    bool successorsIsFine = false, inclose, pathToParent;
    float curvature = 0;

    for (size_t pos = 0; pos <= curCircleNodes.size(); ++pos) {
        // On last iteration we check if goal point inside the circle and process it if possible
        if (pos == curCircleNodes.size()) {
            if (calculateDistanceFromCellToCell(curNode, goal) <= curNode.radius) {
                successor_node = goal;
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
                successor_node.g + weight * linecost * calculateDistanceFromCellToCell(successor_node, goal);
        /*successor_node.F =
                successor_node.g + weight * linecost * calculateDistanceFromCellToCell(successor_node, goal) +
                curvatureHeuristicWeight * distance * successor_node.c;*/


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
                open->Insert(successor_node);
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

    //cosAngle = std::min(cosAngle, 1.0);
    //cosAngle = std::max(-1.0, cosAngle);

    //double angle = fabs(acos(cosAngle));
    //return (angle <= angleLimit);
    return cosAngle >= cosLimit;
}

int LianSearch::findRadiusInDistances(int radius) const {
    for (int i = 0; i < listOfDistances.size(); ++i) {
        if (listOfDistances[i] == radius) {
            return i;
        }
    }
    /*
    int left = 0;
    int right = listOfDistances.size();
    int mid;
    while (right - left > 1) {
        mid = (left + right) >> 1;
        if (listOfDistances[mid] > radius) {
            right = mid;
        } else {
            left = mid;
        }
    }
    return left;
     */
}

const Node *LianSearch::tryToDecreaseRadius(const Node *node_ptr, int width) {
    int i = findRadiusInDistances(node_ptr->radius);
    if (i < listOfDistancesSize - 1) {
        auto range = close.equal_range(*node_ptr);

        for (auto it = range.first; it != range.second; it++) {
            if (it->Parent == nullptr ||
                (it->Parent->i == node_ptr->Parent->i && it->Parent->j == node_ptr->Parent->j &&
                 it->Parent->z == node_ptr->Parent->z)) {
                Node newNode = *node_ptr;
                newNode.radius = listOfDistances[i + 1];
                close.erase(it);
                return &(*(close.insert(newNode)));
            }
        }
    }
    return node_ptr;
}

void LianSearch::makePrimaryPath(Node curNode) {
    std::vector<float> compressed_angles;
    hppath.List.push_front(curNode);
    curNode = *curNode.Parent;
    for (size_t k = 0; curNode.Parent != nullptr; ++k) {
        if (angles[k] != 0.0) {
            hppath.List.push_front(curNode);
            compressed_angles.push_back(angles[k]);
        }
        curNode = *curNode.Parent;
    }
    hppath.List.push_front(curNode);
    angles = std::move(compressed_angles);
}

void LianSearch::makeSecondaryPath(Node curNode) {
    auto cur = hppath.List.begin();
    auto prev = cur++;
    Liner drawer(&lppath.List);

    for (; cur != hppath.List.end(); ++cur, ++prev) {
        if (prev != hppath.List.begin()) {
            lppath.List.pop_back();
        }
        drawer.append_line(*prev, *cur);
    }
}

double LianSearch::makeAngles(Node curNode) {
    angles.resize(0);
    double angle;
    double dis1;
    double dis2;
    double scalarProduct;
    double cosAngle;
    double maxAngle = 0;
    while ((curNode.Parent != nullptr) && (curNode.Parent->Parent != nullptr)) {
        dis1 = calculateDistanceFromCellToCell(curNode, *(curNode.Parent));
        dis2 = calculateDistanceFromCellToCell(*(curNode.Parent), *(curNode.Parent->Parent));
        scalarProduct = (curNode.j - curNode.Parent->j) * (curNode.Parent->j - curNode.Parent->Parent->j) +
                        (curNode.Parent->i - curNode.i) * (curNode.Parent->Parent->i - curNode.Parent->i) +
                        (curNode.z - curNode.Parent->z) * (curNode.Parent->z - curNode.Parent->Parent->z);

        if (dis1 != 0 && dis2 != 0) {
            cosAngle = (scalarProduct / dis1) / dis2;
            cosAngle = std::min(cosAngle, 1.0);
            cosAngle = std::max(-1.0, cosAngle);
            angle = acos(cosAngle) * M_1_PI * 180;
            maxAngle = std::max(maxAngle, fabs(angle));
            angles.push_back(angle);
        }
        curNode = *curNode.Parent;
    }

    angles.shrink_to_fit();
    return maxAngle;
}

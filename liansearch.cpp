#include "liansearch.h"
#include <cmath>
#include <chrono>
#include <functional>
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
    angleLimit = angleLimitDegree * (M_PI / 180);
    //this->angleLimit = angleLimitDegree;
    cosLimit = cos(angleLimit);
    sinLimit = sin(angleLimit);
    shift_radius = std::max(sinLimit, (double) 1 - cosLimit);
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
                                    circleNodes[left].insert(node);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

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
}

bool LianSearch::checkLineSegment(const cMap &map, const Node &start, const Node &goal) const {
    LineOfSight checker(map);
    return checker.line_of_sight(start, goal);
}

bool LianSearch::stopCriterion() {
    if (open->empty()) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }


    if (stepLimit > 0 && close.size() > stepLimit) {
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
    for (int i = 0; i < listOfDistances.size(); i++) {
        std::cout << " " << listOfDistances[i];
    }
    std::cout << std::endl;

    calculateCircles();

    open = new ClusteredSets(Map.height);
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
        //Если текущая точка - целевая, цикл поиска завершается
        if (curNode.i == Map.goal_i && curNode.j == Map.goal_j && curNode.z == Map.goal_z) {
            pathFound = true;
            break;
        }
        if (!expand(node_ptr, Map) && listOfDistances.size() > 1) {
            while (node_ptr->radius > listOfDistances[listOfDistances.size() - 1]) {
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
        sresult.maxAngle = makeAngles(curNode);
        makePrimaryPath(curNode, Map);
        PostSmooth(Map);
        sresult.maxAngle = makeAngles(*hppath.rbegin());

        finish_time = std::chrono::high_resolution_clock::now();

        makeSecondaryPath(curNode);
        sresult.pathfound = true;
        sresult.pathlength = curNode.g;
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.sections = hppath.size() - 1;
    } else {
        sresult.pathfound = false;
        sresult.pathlength = 0;
    }
    sresult.time = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count();
    sresult.time /= 1000000000;
    sresult.nodescreated = open->size() + close.size();
    sresult.numberofsteps = close.size();
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
    int k;
    for (k = 0; k < listOfDistances.size() && listOfDistances[k] != Node_ptr->radius; k++) {}

    const std::unordered_set<Node, std::hash<Node>, NodeCoordEqual> &curCircleNodes = circleNodes[k];

    // Looking for successor in forward direction
    bool found_direct_succ = false;
    Node direct_succ;
    int direct_shift_limit = 0;
    if (Node_ptr->Parent != nullptr) {
        double vect_i = (double) (Node_ptr->i - Node_ptr->Parent->i) * Node_ptr->radius / Node_ptr->Parent->radius;
        double vect_j = (double) (Node_ptr->j - Node_ptr->Parent->j) * Node_ptr->radius / Node_ptr->Parent->radius;
        double vect_z = (double) (Node_ptr->z - Node_ptr->Parent->z) * Node_ptr->radius / Node_ptr->Parent->radius;
        for (auto add_i : {0, 1}) {
            for (auto add_j : {0, 1}) {
                for (auto add_z : {0, 1}) {
                    if (!found_direct_succ) {
                        direct_succ.i = (int) floor(vect_i) + add_i;
                        direct_succ.j = (int) floor(vect_j) + add_j;
                        direct_succ.z = (int) floor(vect_z) + add_z;
                        if (curCircleNodes.find(direct_succ) != curCircleNodes.end()) {
                            found_direct_succ = true;
                        }
                    }
                }
            }
        }

        if (!found_direct_succ) {   // Fake successor in forward direction with less radius
            direct_succ.i = (int) floor(vect_i);
            direct_succ.j = (int) floor(vect_j);
            direct_succ.z = (int) floor(vect_z);
            direct_shift_limit += 2;
        }

        // Calculating borders for searching successors
        direct_shift_limit += ceil(shift_radius * Node_ptr->radius);

        direct_succ.i += Node_ptr->i;
        direct_succ.j += Node_ptr->j;
        direct_succ.z += Node_ptr->z;
    }


    bool successorsIsFine = false;
    Node successor_node;
    if (!found_direct_succ) {
        successorsIsFine |= ProcessSuccessor(Node_ptr, direct_succ, Map);
    }
    for (auto pos = curCircleNodes.begin(); pos != curCircleNodes.end(); ++pos) {
        successor_node.i = Node_ptr->i + pos->i;
        successor_node.j = Node_ptr->j + pos->j;
        successor_node.z = Node_ptr->z + pos->z;

        if (Node_ptr->Parent != nullptr) {
            successorsIsFine |= ProcessSuccessor(Node_ptr, successor_node, Map, direct_succ, direct_shift_limit);
        } else {
            successorsIsFine |= ProcessSuccessor(Node_ptr, successor_node, Map);
        }
    }

    if (calculateDistanceFromCellToCell(*Node_ptr, Node(Map.goal_i, Map.goal_j, Map.goal_z)) <= Node_ptr->radius) {
        successorsIsFine |= ProcessSuccessor(Node_ptr, Node(Map.goal_i, Map.goal_j, Map.goal_z), Map);
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
    return cosAngle > cosLimit + COMPUTATION_EPS;
}

int LianSearch::findRadiusInDistances(int radius) const {
    for (int i = 0; i < listOfDistances.size(); ++i) {
        if (listOfDistances[i] == radius) {
            return i;
        }
    }
    return listOfDistances.size() - 1;
}

const Node *LianSearch::tryToDecreaseRadius(const Node *node_ptr, int width) {
    int i = findRadiusInDistances(node_ptr->radius);
    if (i < listOfDistances.size() - 1) {
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

void LianSearch::makePrimaryPath(Node curNode, const cMap &map) {
    std::vector<float> compressed_angles;
    hppath.push_front(curNode);
    curNode = *curNode.Parent;
    for (size_t k = 0; curNode.Parent != nullptr; ++k) {
        if (angles[k] != 0.0) {
            hppath.push_front(curNode);
            compressed_angles.push_back(angles[k]);
        }
        curNode = *curNode.Parent;
    }
    hppath.push_front(curNode);
    angles = std::move(compressed_angles);
}

void LianSearch::makeSecondaryPath(Node) {
    auto cur = hppath.begin();
    auto prev = cur++;
    Liner drawer(&lppath);

    for (; cur != hppath.end(); ++cur, ++prev) {
        if (prev != hppath.begin()) {
            lppath.pop_back();
        }
        drawer.append_line(*prev, *cur);
    }
}

double LianSearch::makeAngles(Node curNode) {
    angles.resize(0);
    double angle, cosAngle, maxAngle = 0, dis1, dis2, scalarProduct;
    while ((curNode.Parent != nullptr) && (curNode.Parent->Parent != nullptr)) {
        // Checking if there is a turn. Used integral to prevent errors close to zero
        if ((curNode.i - curNode.Parent->i) * (curNode.Parent->j - curNode.Parent->Parent->j) ==
            (curNode.j - curNode.Parent->j) * (curNode.Parent->i - curNode.Parent->Parent->i) &&
            (curNode.j - curNode.Parent->j) * (curNode.Parent->z - curNode.Parent->Parent->z) ==
            (curNode.Parent->j - curNode.Parent->Parent->j) * (curNode.z - curNode.Parent->z)) {

            angles.push_back(0.0);
        } else {
            dis1 = calculateDistanceFromCellToCell(curNode, *(curNode.Parent));
            dis2 = calculateDistanceFromCellToCell(*(curNode.Parent), *(curNode.Parent->Parent));
            scalarProduct = (curNode.j - curNode.Parent->j) * (curNode.Parent->j - curNode.Parent->Parent->j) +
                            (curNode.Parent->i - curNode.i) * (curNode.Parent->Parent->i - curNode.Parent->i) +
                            (curNode.z - curNode.Parent->z) * (curNode.Parent->z - curNode.Parent->Parent->z);

            if (dis1 != 0 && dis2 != 0) {
                cosAngle = scalarProduct / (dis1 * dis2);
                cosAngle = std::min(cosAngle, 1.0);
                cosAngle = std::max(-1.0, cosAngle);
                angle = acos(cosAngle) * (M_1_PI * 180);
                maxAngle = std::max(maxAngle, fabs(angle));
                angles.push_back(angle);
            }
        }
        curNode = *curNode.Parent;
    }

    return maxAngle;
}

bool LianSearch::ProcessSuccessor(const Node *Node_ptr, Node successor, const cMap &Map) {
    if (!Map.NodeOnGrid(successor) || Map.NodeIsObstacle(successor)) {
        return false;
    }

    if (Node_ptr->Parent != nullptr && !checkTurnAngle(*(Node_ptr->Parent), *Node_ptr, successor)) {
        return false;
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

    float curvature = 0;
    successor.Parent = Node_ptr;
    successor.radius = Node_ptr->radius;
    successor.g = Node_ptr->g + linecost * calculateDistanceFromCellToCell(*Node_ptr, successor);
    successor.c = Node_ptr->c + curvature;
    successor.F = successor.g + weight * linecost *
                                calculateDistanceFromCellToCell(successor, Node(Map.goal_i, Map.goal_j, Map.goal_z)) +
                  curvatureHeuristicWeight * distance * successor.c;


    bool pathToParent = checkLineSegment(Map, *Node_ptr, successor);

    if (lesserCircle)
        if (pathToParent && ((Node_ptr->i != Map.goal_i) || (Node_ptr->j != Map.goal_j) || (Node_ptr->z != Map.goal_z)))
            pathToParent = checkLesserCircle(Map, *Node_ptr, CN_PTD_LR);
    bool inclose;
    if (pathToParent) {
        auto range = close.equal_range(successor);
        inclose = false;
        for (auto it = range.first; it != range.second; it++) {
            if (it->Parent != nullptr && it->Parent->i == Node_ptr->i && it->Parent->j == Node_ptr->j &&
                it->Parent->z == Node_ptr->z) {
                inclose = true;
                break;
            }
        }
        if (!inclose) {
            successor.radius = tryToIncreaseRadius(successor);
            open->Insert(successor);
            return true;
        }
    }
    return false;
}

bool LianSearch::ProcessSuccessor(const Node *Node_ptr, const Node &successor, const cMap &Map, const Node &direct_succ,
                                  int max_shift) {
    if (std::max(std::max(abs(successor.i - direct_succ.i), abs(successor.j - direct_succ.j)),
                 abs(successor.z - direct_succ.z)) >
        max_shift) {
        return false;
    }
    return ProcessSuccessor(Node_ptr, successor, Map);
}

void LianSearch::PostSmooth(const cMap &map) {
    std::list<Node> smoothed_path;
    {
        auto prev = hppath.rbegin();
        auto candidate = prev++ ++ ++;
        auto current_parent = candidate++ ++;
        Node current = *current_parent++;
        Node next = current;
        for (; candidate != hppath.rend(); ++candidate, ++prev) {
            if (candidate->g + linecost * calculateDistanceFromCellToCell(*candidate, current) + COMPUTATION_EPS < current.g &&
                (prev == hppath.rend() || checkTurnAngle(*prev, *candidate, current)) &&
                checkTurnAngle(*candidate, current, next) && checkLineSegment(map, *candidate, current)) {

                current.g = candidate->g + linecost * calculateDistanceFromCellToCell(*candidate, current);
                current_parent = candidate;
            } else {
                next = current;
                smoothed_path.push_front(current);
                current = *current_parent++;
            }
        }
        smoothed_path.push_front(current);
        if (current_parent != hppath.rend()) smoothed_path.push_front(*current_parent);
    }

    auto it = smoothed_path.begin();
    it->Parent = nullptr;
    for (auto prev = it++; it != smoothed_path.end(); ++it, ++prev) {
        it->g = prev->g + linecost * calculateDistanceFromCellToCell(*prev, *it);
        it->Parent = &(*prev);
    }

    hppath = std::move(smoothed_path);
}
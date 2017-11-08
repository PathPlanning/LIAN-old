#include "liansearch.h"
#include <math.h>
#include <chrono>
#include "gl_const.h"
#include <list>

#ifdef __linux__

#include <sys/time.h>

#else
#include <windows.h>
#endif


LianSearch::LianSearch(float angleLimit, int distance, float weight,
                       unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
                       float decreaseDistanceFactor, int distanceMin,
                       float linecost, float pivotRadius, int numOfParentsToIncreaseRadius, int breakingties) {
    this->angleLimit = angleLimit;
    this->distance = distance;
    this->weight = weight;
    this->stepLimit = steplimit;
    this->circleRadiusFactor = circleRadiusFactor;
    this->curvatureHeuristicWeight = curvatureHeuristicWeight;
    this->decreaseDistanceFactor = decreaseDistanceFactor;
    this->distanceMin = distanceMin;
    this->linecost = linecost;
    this->pivotRadius = pivotRadius;
    this->numOfParentsToIncreaseRadius = numOfParentsToIncreaseRadius;
    this->breakingties = breakingties;
    closeSize = 0;
    openSize = 0;
}

void LianSearch::calculateCircle(int radius) {
    // ������ - ������ ���������� � �������

    circleNodes.clear();
    circleNodes.resize(listOfDistancesSize);
    for (int k = 0; k < listOfDistancesSize; k++) {
        radius = listOfDistances[k];
        circleNodes[k].clear();
        std::vector<Node> circle_nodes;
        int x = 0;
        int y = radius;

        int delta = 2 - 2 * radius;
        int error = 0;

        while (y >= 0) {
            if (x > radius)
                x = radius;
            else if (x < -radius)
                x = -radius;
            if (y > radius)
                y = radius;
            else if (y < -radius)
                y = -radius;

            circle_nodes.push_back(Node(Cell(x, y)));
            circle_nodes.push_back(Node(Cell(x, -y)));
            circle_nodes.push_back(Node(Cell(-x, y)));
            circle_nodes.push_back(Node(Cell(-x, -y)));

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

        for (int i = 0; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = circle_nodes.size() - 7; i > 0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = 7; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = circle_nodes.size() - 6; i > 0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        circleNodes[k].pop_back();
    }
}

void LianSearch::calculatePivotCircle() {
    int add_x, add_y, num = pivotRadius + 0.5;
    for (int x = -num; x <= +num; x++)
        for (int y = -num; y <= +num; y++) {
            add_x = x != 0 ? 1 : 0;
            add_y = y != 0 ? 1 : 0;
            if ((pow(2 * abs(x) - add_x, 2) + pow(2 * abs(y) - add_y, 2)) < pow(2 * pivotRadius, 2)) {
                pivotCircle.push_back(Cell(x, y));
            }
        }
    if (pivotCircle.empty()) {
        pivotCircle.push_back(Cell(0, 0));
    }
}

bool LianSearch::checkPivotCircle(const Map &map, const Node &center) {
    Cell new_cell;
    for (auto cell : pivotCircle) {
        new_cell = center.cell + cell;
        if (new_cell.i > 0 && new_cell.j > 0 && new_cell.i < map.height && new_cell.j < map.width && map.CellIsObstacle(new_cell)) {
            return false;
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


void LianSearch::calculateLineSegment(std::list<Node> &line, const Node &start, const Node &goal) {
    Cell c1 = start.cell;
    Cell c2 = goal.cell;

    int i_, j_;
    Cell delta;
    int step = 0;
    int rotate = 0;

    line.clear();

    if (c1.i > c2.i && c1.j > c2.j) {
        std::swap(c1, c2);
        delta = c2 - c1;
    } else {
        delta = c2 - c1;
        if (delta.i >= 0 && delta.j >= 0) {
            rotate = 2;
        } else if (delta.i < 0) {
            delta.i = -delta.i;
            std::swap(c1.i, c2.i);
            rotate = 1;
        } else if (delta.j < 0) {
            delta.j = -delta.j;
            std::swap(c1.j, c2.j);
            rotate = 3;
        }
    }
    switch (rotate)
    {
        case 1:
        {
            if (delta.i >= delta.j) {
                j_ = c2.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    line.push_back(Node(Cell(i_, j_)));
                    step += delta.j;
                    if (step >= delta.i) {
                        --j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c2.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    line.push_front(Node(Cell(i_, j_)));
                    step += delta.i;
                    if (step >= delta.j) {
                        --i_;
                        step -= delta.j;
                    }
                }
            }
            return;
        }
        case 2:
        {
            if (delta.i >= delta.j) {
                j_ = c1.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    line.push_back(Node(Cell(i_, j_)));
                    step += delta.j;
                    if (step >= delta.i) {
                        ++j_;
                        step -= delta.i;
                    }
                }
                return;
            } else {
                i_ = c1.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    line.push_back(Node(Cell(i_, j_)));
                    step += delta.i;
                    if (step >= delta.j) {
                        ++i_;
                        step -= delta.j;
                    }
                }
                return;
            }
        }
        case 3:
        {
            if (delta.i >= delta.j) {
                j_ = c2.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    line.push_front(Node(Cell(i_, j_)));
                    step += delta.j;
                    if (step >= delta.i) {
                        --j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c2.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    line.push_back(Node(Cell(i_, j_)));
                    step += delta.i;
                    if (step >= delta.j) {
                        --i_;
                        step -= delta.j;
                    }
                }
            }
            return;
        }
        default:
        {
            if (delta.i >= delta.j) {
                j_ = c1.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    line.push_front(Node(Cell(i_, j_)));
                    step += delta.j;
                    if (step >= delta.i) {
                        ++j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c1.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    line.push_front(Node(Cell(i_, j_)));
                    step += delta.i;
                    if (step >= delta.j) {
                        ++i_;
                        step -= delta.j;
                    }
                }
            }
        }
    }
}

bool LianSearch::checkLineSegment(const Map &map, const Node &start, const Node &goal) {
    Cell c1 = start.cell;
    Cell c2 = goal.cell;

    int i_, j_;
    Cell delta;
    int step = 0;
    int rotate = 0;

    if (c1.i > c2.i && c1.j > c2.j) {
        std::swap(c1, c2);
        delta = c2 - c1;
    } else {
        delta = c2 - c1;
        if (delta.i >= 0 && delta.j >= 0) {
            rotate = 2;
        } else if (delta.j < 0) {
            delta.j = -delta.j;
            std::swap(c1.j, c2.j);
            rotate = 1;
        } else if (delta.i < 0) {
            delta.i = -delta.i;
            std::swap(c1.i, c2.i);
            rotate = 3;
        }
    }
    switch (rotate)
    {
        case 1:
        {
            if (delta.i >= delta.j) {
                j_ = c2.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;

                    step += delta.j;
                    if (step >= delta.i) {
                        --j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c2.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;

                    step += delta.i;
                    if (step >= delta.j) {
                        --i_;
                        step -= delta.j;
                    }
                }
            }
            return true;
        }
        case 2:
        {
            if (delta.i >= delta.j) {
                j_ = c1.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.j;
                    if (step >= delta.i) {
                        ++j_;
                        step -= delta.i;
                    }
                }
                return true;
            } else {
                i_ = c1.i;
                for (j_ = c1.j; j_ <= c2.j; j_++) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.i;
                    if (step >= delta.j) {
                        ++i_;
                        step -= delta.j;
                    }
                }
                return true;
            }
        }
        case 3:
        {
            if (delta.i >= delta.j) {
                j_ = c2.j;
                for (i_ = c1.i; i_ <= c2.i; ++i_) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.j;
                    if (step >= delta.i) {
                        --j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c2.i;
                for (j_ = c1.j; j_ <= c2.j; ++j_) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.i;
                    if (step >= delta.j) {
                        --i_;
                        step -= delta.j;
                    }
                }
            }
            return true;
        }

        default:
        {
            if (delta.i >= delta.j) {
                j_ = c1.j;
                for (i_ = c1.i; i_ <= c2.i; i_++) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.j;
                    if (step >= delta.i) {
                        ++j_;
                        step -= delta.i;
                    }
                }
            } else {
                i_ = c1.i;
                for (j_ = c1.j; j_ <= c2.j; j_++) {
                    if (map.CellIsObstacle(Cell(i_,j_))) return false;
                    step += delta.i;
                    if (step >= delta.j) {
                        ++i_;
                        step -= delta.j;
                    }
                }
            }
            return true;
        }
    }
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

double LianSearch::calculateDistanceFromCellToCell(Cell a, Cell b) {
    return sqrt(abs(a.i - b.i) * abs(a.i - b.i) + abs(a.j - b.j) * abs(a.j - b.j));
}

SearchResult LianSearch::startSearch(Logger *Log, const Map &map) {
    auto start_time = std::chrono::high_resolution_clock::now();
    calculateDistances();

    std::cout << "List of distances :";
    for (int i = 0; i < listOfDistancesSize; i++) {
        std::cout << " " << listOfDistances[i];
    }
    std::cout << std::endl;

    open.resize(map.height);
    cluster_minimums.resize(map.height);

    Node curNode(map.start, breakingties);
    curNode.g = 0;
    curNode.c = 0;
    curNode.radius = distance;
    curNode.F = weight * linecost * calculateDistanceFromCellToCell(curNode.cell, map.goal);

    bool pathFound = false;
    addOpen(curNode);
    calculateCircle((int) curNode.radius);
    calculatePivotCircle();
    // �������� ���� ������
    while (!stopCriterion()) {
        curNode = findMin();
        //deleteMin(curNode);
        close.insert({curNode.cell.i * map.width + curNode.cell.j, curNode});
        closeSize++;
        //std::cout << curNode.cell.j << "," << curNode.cell.i << std::endl;
        //std::cout << "open size: " << openSize << " close size: " << closeSize << std::endl;

        //���� ������� ����� - �������, ���� ������ �����������
        if (curNode.cell == map.goal) {
            pathFound = true;
            break;
        }
        if (!expand(curNode, map) && listOfDistancesSize > 1)
            while (curNode.radius > listOfDistances[listOfDistancesSize - 1])
                if (tryToDecreaseRadius(curNode, map.width))
                    if (expand(curNode, map))
                        break;
        if (Log->loglevel >= CN_LOGLVL_LOW) {
            Log->writeToLogOpenClose(open, close);
        }
    }
    if (Log->loglevel == CN_LOGLVL_MED) {
        Log->writeToLogOpenClose(open, close);
    }
    if (pathFound) {
        float maxAngle = makeAngles(curNode);
        makePrimaryPath(curNode);
        auto finish_time = std::chrono::high_resolution_clock::now();
        sresult.time = std::chrono::duration_cast<std::chrono::microseconds>(finish_time - start_time).count();
        sresult.time /= 1000000;
        makeSecondaryPath(curNode);
        sresult.pathfound = true;
        sresult.pathlength = curNode.g;
        sresult.nodescreated = openSize + closeSize;
        sresult.numberofsteps = closeSize;
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.maxAngle = maxAngle;
        sresult.sections = hppath.size() - 1;
        return sresult;
    } else {
        auto finish_time = std::chrono::high_resolution_clock::now();
        sresult.time = std::chrono::duration_cast<std::chrono::microseconds>(finish_time - start_time).count();
        sresult.time /= 1000000;
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
        if (curNode.parent)
            if (curNode.radius == curNode.parent->radius) {
                ++k;
                curNode = *curNode.parent;
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

bool LianSearch::expand(const Node curNode, const Map &map) {
    int k;
    for(k = 0; k < listOfDistancesSize; k++)
        if(listOfDistances[k] == curNode.radius)
            break;
    std::vector<Node> circle_nodes = circleNodes[k];
    Cell successor;
    bool successor_is_fine = false, in_close;
    float cos_angle,angle,curvature;
    int pos;
    if (curNode.parent) {
        std::vector<Node> successors;
        std::vector<int> succs_index;
        for (pos = 0; pos < circle_nodes.size(); ++pos)
            if (curNode.parent->cell == curNode.cell + circle_nodes[pos].cell)
                break;
        if (pos < circle_nodes.size() / 2) pos += circle_nodes.size() / 2;
        else pos -= circle_nodes.size() / 2;
        int k1 = pos + 1;
        int k2 = pos - 1;
        succs_index.push_back(pos);
        for (int i = 0; i < circle_nodes.size(); ++i) {
            if(k1 >= circle_nodes.size()) k1 = 0;
            if(k2 < 0) k2 = circle_nodes.size() - 1;
            succs_index.push_back(k1++);
            succs_index.push_back(k2--);
            if (succs_index.size() >= circle_nodes.size() / 2)
                break;
        }
        for (int i = 0; i < circle_nodes.size() / 2; ++i) {
            successor = curNode.cell + circle_nodes[succs_index[i]].cell;
            if (!map.CellOnGrid(successor)) continue;
            if (map.CellIsObstacle(successor)) continue;

            Cell vec1 = curNode.cell - curNode.parent->cell;
            Cell vec2 = successor - curNode.cell;
            double dist1 = calculateDistanceFromCellToCell(curNode.cell, curNode.parent->cell);
            double dist2 = calculateDistanceFromCellToCell(successor, curNode.cell);
            cos_angle = ( vec1.i * vec2.i + vec1.j * vec2.j) / (dist1 * dist2);

            if (cos_angle > 1)
                cos_angle = 1;
            if (cos_angle < -1)
                cos_angle = -1;

            angle = acos(cos_angle);
            curvature = fabs( angle );
            angle = angle * 180 / CN_PI_CONSTANT;

            if (fabs(angle) > angleLimit)
                break;
            successors.push_back(circle_nodes[succs_index[i]]);
        }
        circle_nodes=successors;

    }
    for (unsigned int i = 0; i <= circle_nodes.size(); ++i) {
        Node newNode;
        // ьЯ УЫт­ЯШУУ ШУЖвЯ, ЖЬсСЯ i == circle_nodes.size(),№­ЬРт­ тВ ШтвтРз■ ЫЬэЖз
        if (i == circle_nodes.size()) {
            if (calculateDistanceFromCellToCell(curNode.cell,map.goal) <= curNode.radius) {
                successor = map.goal;
                if (curNode.parent) {
                    Cell vec1 = curNode.cell - curNode.parent->cell;
                    Cell vec2 = successor - curNode.cell;
                    double dist1 = calculateDistanceFromCellToCell(curNode.cell, curNode.parent->cell);
                    double dist2 = calculateDistanceFromCellToCell(successor, curNode.cell);
                    cos_angle = ( vec1.i * vec2.i + vec1.j * vec2.j) / (dist1 * dist2);

                    angle = acos(cos_angle);
                    curvature = fabs( angle );
                    angle = angle * 180 / CN_PI_CONSTANT;
                    if (fabs(angle) > angleLimit) continue;
                }
            }
            else continue;
        } else successor = curNode.cell + circle_nodes[i].cell;

        if (!map.CellOnGrid(successor)) continue;
        if (map.CellIsObstacle(successor)) continue;


        newNode.cell = successor;
        newNode.parent = &(close.find(curNode.convolution(map.width))->second);
        newNode.radius = newNode.parent->radius;
        newNode.path_to_parent = false;
        newNode.g = newNode.parent->g + linecost * calculateDistanceFromCellToCell(curNode.cell,newNode.cell);
        newNode.c = newNode.parent->c + curvature;
        newNode.F = newNode.g + weight * linecost * calculateDistanceFromCellToCell(successor, map.goal)
                    + curvatureHeuristicWeight * distance * newNode.c;
        newNode.path_to_parent = checkLineSegment(map,*newNode.parent,newNode);

        if (pivotRadius > 0) {
            if (newNode.cell != map.goal) {
                newNode.path_to_parent &= checkPivotCircle(map, newNode);
            }
        }

        if(newNode.path_to_parent) {
            std::unordered_multimap<int,Node>::const_iterator it = close.find(newNode.convolution(map.width));
            if (it != close.end()) {
                in_close = false;
                auto range = close.equal_range(it->first);
                for (auto it = range.first; it != range.second; ++it)
                    if (it->second.parent && it->second.parent->cell == curNode.cell) in_close=true;
                if (!in_close) {
                    if (listOfDistancesSize > 1) newNode.radius = tryToIncreaseRadius(newNode);
                    addOpen(newNode);
                    successor_is_fine = true;
                }
            } else {
                if (listOfDistancesSize > 1) newNode.radius=tryToIncreaseRadius(newNode);
                addOpen(newNode);
                successor_is_fine = true;
            }
        }
    }
    return successor_is_fine;
}

bool LianSearch::tryToDecreaseRadius(Node &curNode, int width) {
    int i;
    for (i = listOfDistancesSize - 1; i >= 0; --i)
        if (curNode.radius == listOfDistances[i])
            break;
    if (i < listOfDistancesSize - 1) {
        curNode.radius = listOfDistances[i + 1];
        auto it = close.find(curNode.convolution(width));
        auto range = close.equal_range(it->first);
        for (auto it = range.first; it != range.second; it++) {
            if (it->second.parent && it->second.parent->cell == curNode.cell) {
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
    hppath.push_front(curNode);
    curNode = *curNode.parent;
    do {

        hppath.push_front(curNode);
        k++;
        curNode = *curNode.parent;

    } while (curNode.parent);
    hppath.push_front(curNode);
}

void LianSearch::makeSecondaryPath(Node curNode) {
    std::list<Node> lineSegment;
    do {
        calculateLineSegment(lineSegment, *curNode.parent, curNode);
        lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
        curNode = *curNode.parent;
    } while (curNode.parent);
    lppath.push_front(*lineSegment.begin());
}

double LianSearch::makeAngles(Node curNode) {
    angles.clear();
    double max_angle = 0;
    while (curNode.parent && curNode.parent->parent) {
        double angle = 0;
        double dis1 = calculateDistanceFromCellToCell(curNode.cell, curNode.parent->cell);
        double dis2 = calculateDistanceFromCellToCell(curNode.parent->cell, curNode.parent->parent->cell);

        Cell vec1 = curNode.cell - curNode.parent->cell;
        Cell vec2 = curNode.parent->cell - curNode.parent->parent->cell;
        double cos_angle = static_cast<double>(vec1.i * vec2.i + vec1.j * vec2.j) / (dis1 * dis2);
        if (cos_angle > 1) cos_angle = 1;
        if (cos_angle < -1) cos_angle = -1;

        angle = acos(cos_angle);
        angle = angle * 180 / CN_PI_CONSTANT;

        if (angle > max_angle)
            max_angle = angle;
        angles.push_back(angle);
        curNode = *curNode.parent;
    }
    return max_angle;
}

Node LianSearch::findMin() {
    Node min;
    min.F = std::numeric_limits<double>::infinity();
    min.g = std::numeric_limits<double>::infinity();
    for (auto elem : open) {
        if (!elem.empty()) {
            Node node = elem.front();
            if (node < min) min = node;
        }
    }
    open[min.cell.i].pop_front();
    --openSize;
    return min;
}

void LianSearch::addOpen(Node &newNode) {
    if (open[newNode.cell.i].empty()) {
        open[newNode.cell.i].push_front(newNode);
        ++openSize;
        return;
    }
    std::list<Node>::iterator position = open[newNode.cell.i].end();
    bool pos_found = false;
    for (auto it = open[newNode.cell.i].begin(); it != open[newNode.cell.i].end(); ++it) {
        if (!(*it < newNode) && !pos_found) {
            position = it;
            pos_found = true;
        }
        if (it->cell == newNode.cell && it->parent->cell == newNode.parent->cell) {
            if (*it < newNode) return;
            else if (position == it) {
                it->F = newNode.F;
                it->g = newNode.g;
                it->c = newNode.c;
                it->radius = newNode.radius;
                return;
            } else {
                open[newNode.cell.i].erase(it);
                --openSize;
                break;
            }
        }
    }
    ++openSize;
    open[newNode.cell.i].insert(position,newNode);
}

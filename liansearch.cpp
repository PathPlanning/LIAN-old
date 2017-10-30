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

void LianSearch::calculatePivotCircle() {
    int add_x, add_y, num = pivotRadius + 0.5;
    Node node;
    for (int x = -num; x <= +num; x++)
        for (int y = -num; y <= +num; y++) {
            add_x = x != 0 ? 1 : 0;
            add_y = y != 0 ? 1 : 0;
            if ((pow(2 * abs(x) - add_x, 2) + pow(2 * abs(y) - add_y, 2)) < pow(2 * pivotRadius, 2)) {
                node.i = x;
                node.j = y;
                pivotCircle.push_back(node);
            }
        }
    if (pivotCircle.empty()) {
        node.i = node.j = 0;
        pivotCircle.push_back(node);
    }
}

bool LianSearch::checkPivotCircle(const cMap &Map, const Node &center) {
    int i, j;
    for (Node node : pivotCircle) {
        i = center.i + node.i;
        j = center.j + node.j;
        if (i > 0 && j > 0 && i < Map.height && j < Map.width && Map.Grid[i][j] != 0) {
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


void LianSearch::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal) {
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x, y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;
    int Tmp;
    Node newNode;

    line.clear();

    if (x1 > x2 && y1 > y2) {
        Tmp = x2;
        x2 = x1;
        x1 = Tmp;

        Tmp = y2;
        y2 = y1;
        y1 = Tmp;

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) {
            Rotate = 2;
        } else if (dy < 0) {
            dy = -dy;

            Tmp = y2;
            y2 = y1;
            y1 = Tmp;

            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;

            Tmp = x2;
            x2 = x1;
            x1 = Tmp;

            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if (dx >= dy) {
            y = y2;
            for (x = x1; x <= x2; x++) {
                newNode.i = x;
                newNode.j = y;
                line.push_back(newNode);

                StepVal += dy;
                if (StepVal >= dx) {
                    y--;
                    StepVal -= dx;
                }
            }
        } else {
            x = x2;
            for (y = y1; y <= y2; y++) {
                newNode.i = x;
                newNode.j = y;
                line.insert(line.begin(), newNode);

                StepVal += dx;
                if (StepVal >= dy) {
                    x--;
                    StepVal -= dy;
                }
            }
        }

        return;
    } else if (Rotate == 2) {
        if (dx >= dy) {
            y = y1;
            for (x = x1; x <= x2; x++) {
                newNode.i = x;
                newNode.j = y;
                line.push_back(newNode);

                StepVal += dy;
                if (StepVal >= dx) {
                    y++;
                    StepVal -= dx;
                }
            }

            return;
        } else {
            x = x1;
            for (y = y1; y <= y2; y++) {
                newNode.i = x;
                newNode.j = y;
                line.push_back(newNode);

                StepVal += dx;
                if (StepVal >= dy) {
                    x++;
                    StepVal -= dy;
                }
            }

            return;
        }
    } else if (Rotate == 3) {
        if (dx >= dy) {
            y = y2;
            for (x = x1; x <= x2; x++) {
                newNode.i = x;
                newNode.j = y;
                line.insert(line.begin(), newNode);

                StepVal += dy;
                if (StepVal >= dx) {
                    y--;
                    StepVal -= dx;
                }
            }
        } else {
            x = x2;
            for (y = y1; y <= y2; y++) {
                newNode.i = x;
                newNode.j = y;
                line.push_back(newNode);

                StepVal += dx;
                if (StepVal >= dy) {
                    x--;
                    StepVal -= dy;
                }
            }
        }
        return;
    }

    if (dx >= dy) {
        y = y1;
        for (x = x1; x <= x2; x++) {
            newNode.i = x;
            newNode.j = y;
            line.insert(line.begin(), newNode);

            StepVal += dy;

            if (StepVal >= dx) {
                y++;
                StepVal -= dx;
            }
        }
    } else {
        x = x1;
        for (y = y1; y <= y2; y++) {
            newNode.i = x;
            newNode.j = y;
            line.insert(line.begin(), newNode);

            StepVal += dx;
            if (StepVal >= dy) {
                x++;
                StepVal -= dy;
            }
        }
    }
}

bool LianSearch::checkLineSegment(const cMap &Map, const Node &start, const Node &goal) {
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x, y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;
    int Tmp;

    if (x1 > x2 && y1 > y2) {
        Tmp = x2;
        x2 = x1;
        x1 = Tmp;

        Tmp = y2;
        y2 = y1;
        y1 = Tmp;

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) {
            Rotate = 2;
        } else if (dy < 0) {
            dy = -dy;

            Tmp = y2;
            y2 = y1;
            y1 = Tmp;

            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;

            Tmp = x2;
            x2 = x1;
            x1 = Tmp;

            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if (dx >= dy) {
            y = y2;
            for (x = x1; x <= x2; x++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dy;
                if (StepVal >= dx) {
                    y--;
                    StepVal -= dx;
                }
            }
        } else {
            x = x2;
            for (y = y1; y <= y2; y++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dx;
                if (StepVal >= dy) {
                    x--;
                    StepVal -= dy;
                }
            }
        }

        return true;
    } else if (Rotate == 2) {
        if (dx >= dy) {
            y = y1;
            for (x = x1; x <= x2; x++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dy;
                if (StepVal >= dx) {
                    y++;
                    StepVal -= dx;
                }
            }

            return true;
        } else {
            x = x1;
            for (y = y1; y <= y2; y++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dx;
                if (StepVal >= dy) {
                    x++;
                    StepVal -= dy;
                }
            }

            return true;
        }
    } else if (Rotate == 3) {
        if (dx >= dy) {
            y = y2;
            for (x = x1; x <= x2; x++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dy;
                if (StepVal >= dx) {
                    y--;
                    StepVal -= dx;
                }
            }
        } else {
            x = x2;
            for (y = y1; y <= y2; y++) {
                if (Map.Grid[x][y] == CN_OBSTL) return false;

                StepVal += dx;
                if (StepVal >= dy) {
                    x--;
                    StepVal -= dy;
                }
            }
        }
        return true;
    }

    if (dx >= dy) {
        y = y1;
        for (x = x1; x <= x2; x++) {
            if (Map.Grid[x][y] == CN_OBSTL) return false;

            StepVal += dy;

            if (StepVal >= dx) {
                y++;
                StepVal -= dx;
            }
        }
    } else {
        x = x1;
        for (y = y1; y <= y2; y++) {
            if (Map.Grid[x][y] == CN_OBSTL) return false;

            StepVal += dx;
            if (StepVal >= dy) {
                x++;
                StepVal -= dy;
            }
        }
    }

    return true;
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

double LianSearch::calculateDistanceFromCellToCell(int start_i, int start_j, int fin_i, int fin_j) {
    int delta_i, delta_j;
    delta_i = abs(start_i - fin_i);
    delta_j = abs(start_j - fin_j);

    return sqrt(double(delta_i * delta_i) + double(delta_j * delta_j));
}

SearchResult LianSearch::startSearch(cLogger *Log, const cMap &Map) {
    auto start_time = std::chrono::high_resolution_clock::now();
    calculateDistances();

    std::cout << "List of distances :";
    for (int i = 0; i < listOfDistancesSize; i++) {
        std::cout << " " << listOfDistances[i];
    }
    std::cout << std::endl;
    open.resize(Map.height);
    cluster_minimums.resize(Map.height);
    Node curNode(Map.start_i, Map.start_j, 0.0, 0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * linecost * calculateDistanceFromCellToCell(curNode.i, curNode.j, Map.goal_i, Map.goal_j);
    bool pathFound = false;
    addOpen(curNode, (unsigned) curNode.i * Map.width + (unsigned) curNode.j);
    calculateCircle((int) curNode.radius);
    calculatePivotCircle();
    // �������� ���� ������
    while (!stopCriterion()) {
        curNode = findMin();
        deleteMin(curNode);
        close.insert({curNode.i * Map.width + curNode.j, curNode});
        closeSize++;
        //���� ������� ����� - �������, ���� ������ �����������
        if (curNode.i == Map.goal_i && curNode.j == Map.goal_j) {
            pathFound = true;
            break;
        }
        if (!expand(curNode, Map) && listOfDistancesSize > 1)
            while (curNode.radius > listOfDistances[listOfDistancesSize - 1])
                if (tryToDecreaseRadius(curNode, Map.width))
                    if (expand(curNode, Map))
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
        sresult.sections = hppath.List.size() - 1;
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
        // �� �������� �����, ����� i == circleNodes.size(),��������� ������� �����
        if (i == circleNodes.size()) {
            if (calculateDistanceFromCellToCell(curNode.i, curNode.j, Map.goal_i, Map.goal_j) <= curNode.radius) {
                succi = Map.goal_i;
                succj = Map.goal_j;
                if (curNode.Parent != NULL) {
                    scalarProduct = (curNode.j - curNode.Parent->j) * (succj - curNode.j) +
                                    (curNode.Parent->i - curNode.i) * (curNode.i - succi);
                    cosAngle =
                            (float) scalarProduct / calculateDistanceFromCellToCell(curNode.i, curNode.j, succi, succj);
                    cosAngle = (float) cosAngle /
                               calculateDistanceFromCellToCell(curNode.Parent->i, curNode.Parent->j, curNode.i,
                                                               curNode.j);
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

        // Deprecated. Check if goal can be reached from current node, if turning with maximal angle.
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
        }
        */
        newNode.i = succi;
        newNode.j = succj;
        newNode.Parent = &(close.find(curNode.i * Map.width + curNode.j)->second);
        newNode.radius = newNode.Parent->radius;
        newNode.pathToParent = false;
        newNode.g = newNode.Parent->g +
                    linecost * calculateDistanceFromCellToCell(curNode.i, curNode.j, newNode.i, newNode.j);
        newNode.c = newNode.Parent->c + curvature;
        newNode.F =
                newNode.g + weight * linecost * calculateDistanceFromCellToCell(succi, succj, Map.goal_i, Map.goal_j)
                + curvatureHeuristicWeight * distance * newNode.c;
        newNode.pathToParent = checkLineSegment(Map, *newNode.Parent, newNode);


        if (pivotRadius > 0) {
            if (newNode.i != Map.goal_i || newNode.j != Map.goal_j) {
                newNode.pathToParent &= checkPivotCircle(Map, newNode);
            }
        }
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
                    addOpen(newNode, (unsigned) newNode.i * Map.width + (unsigned) newNode.j);
                    successorsIsFine = true;
                }
            } else {
                if (listOfDistancesSize > 1)
                    newNode.radius = tryToIncreaseRadius(newNode);
                addOpen(newNode, (unsigned) newNode.i * Map.width + (unsigned) newNode.j);
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
            angle = 0;
            dis1 = calculateDistanceFromCellToCell(curNode.i, curNode.j, curNode.Parent->i, curNode.Parent->j);
            dis2 = calculateDistanceFromCellToCell(curNode.Parent->i, curNode.Parent->j,
                                                   curNode.Parent->Parent->i, curNode.Parent->Parent->j);

            scalarProduct = (curNode.j - curNode.Parent->j) * (curNode.Parent->j - curNode.Parent->Parent->j) +
                            (curNode.Parent->i - curNode.i) * (curNode.Parent->Parent->i - curNode.Parent->i);

            if ((dis1 != 0) && (dis2 != 0)) {
                cosAngle = (double) scalarProduct /
                           calculateDistanceFromCellToCell(curNode.Parent->i, curNode.Parent->j, curNode.i, curNode.j);
                cosAngle = (double) cosAngle /
                           calculateDistanceFromCellToCell(curNode.Parent->Parent->i, curNode.Parent->Parent->j,
                                                           curNode.Parent->i, curNode.Parent->j);
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

Node LianSearch::findMin() const {
    Node min;
    min.F = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < open.size(); ++i) {
        if (!open.empty()) {
            auto min_range = open[i].equal_range(cluster_minimums[i]);
            for (auto it = min_range.first; it != min_range.second; ++it) {
                if (it->second.F < min.F || (it->second.F == min.F &&
                                             (breakingties == CN_BT_GMAX && it->second.g > min.g ||
                                              breakingties == CN_BT_GMIN && it->second.g < min.g))) {
                    min = it->second;
                }
            }
        }
    }

    return min;
}

void LianSearch::deleteMin(const Node &min) {
    --openSize;
    auto deleting_range = open[min.i].equal_range(cluster_minimums[min.i]);
    for (auto it = deleting_range.first; it != deleting_range.second; ++it) {
        if (it->second.g == min.g && it->second.Parent == min.Parent) {
            open[min.i].erase(it);
            break;
        }
    }
    if (!open[min.i].empty()) {
        Node new_min;
        new_min.F = std::numeric_limits<float>::infinity();
        for (auto it = open[min.i].begin(); it != open[min.i].end(); ++it) {
            if (it->second.F < new_min.F || (it->second.F == new_min.F &&
                                             (breakingties == CN_BT_GMAX && it->second.g > new_min.g ||
                                              breakingties == CN_BT_GMIN && it->second.g < new_min.g))) {
                new_min = it->second;
                cluster_minimums[min.i] = it->first;
            }
        }
    }
}

void LianSearch::addOpen(Node &newNode, unsigned key) {
    auto range = open[newNode.i].equal_range(key);
    bool found = false;
    bool must_be_replaced = false;
    for (auto it = range.first; it != range.second; ++it) {
        if (it->second.Parent == newNode.Parent) {
            found = true;
            if (newNode.g < it->second.g) {
                must_be_replaced = true;
                open[newNode.i].erase(it);
                break;
            }
        }
    }
    if (!found) {
        ++openSize;
    }
    if (!found || must_be_replaced) {
        open[newNode.i].insert({key, newNode});
        if (open[newNode.i].size() == 1) {
            cluster_minimums[newNode.i] = key;
        } else {
            Node min;
            min.F = std::numeric_limits<float>::infinity();
            auto min_range = open[newNode.i].equal_range(cluster_minimums[newNode.i]);
            for (auto it = min_range.first; it != min_range.second; ++it) {
                if (it->second.F < min.F || (it->second.F == min.F &&
                                             (breakingties == CN_BT_GMAX && it->second.g > min.g ||
                                              breakingties == CN_BT_GMIN && it->second.g < min.g))) {
                    min = it->second;
                }
            }
            if (newNode.F < min.F || (newNode.F == min.F && (breakingties == CN_BT_GMAX && newNode.g > min.g ||
                                                             breakingties == CN_BT_GMIN && newNode.g < min.g))) {
                cluster_minimums[newNode.i] = key;
            }
        }
    }
}

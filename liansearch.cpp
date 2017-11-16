#include "liansearch.h"

LianSearch::~LianSearch() {}


LianSearch::LianSearch(float angleLimit, int distance, float weight, int breakingties,
                       unsigned int steplimit, float curvatureHeuristicWeight,
                       float decreaseDistanceFactor, int distanceMin,
                       double PivotRadius, int numOfParentsToIncreaseRadius) {
    this->angleLimit = angleLimit;
    this->distance = distance;
    this->weight = weight;
    this->BT = breakingties;
    this->stepLimit = steplimit;
    this->curvatureHeuristicWeight = curvatureHeuristicWeight;
    this->decreaseDistanceFactor = decreaseDistanceFactor;
    this->distanceMin = distanceMin;
    this->pivotRadius = PivotRadius;
    this->numOfParentsToIncreaseRadius = numOfParentsToIncreaseRadius;
    closeSize = 0;
}

void LianSearch::calculateCircle(int radius) { //here radius - radius of the circle in cells
    circleNodes.clear();
    circleNodes.resize(listOfDistancesSize);
    for(int k = 0; k < listOfDistancesSize; ++k) {
        radius = listOfDistances[k];
        circleNodes[k].clear();
        std::vector<Node> circle_nodes;
        int x = 0;
        int y = radius;
        int delta = 2 - 2 * radius;
        int error = 0;
        while (y >= 0) {
            if(x > radius) x = radius;
            else if(x < -radius) x = -radius;
            if(y > radius) y = radius;
            else if(y < -radius) y = -radius;

            circle_nodes.push_back(Node(x, y));
            circle_nodes.push_back(Node(x, -y));
            circle_nodes.push_back(Node(-x, y));
            circle_nodes.push_back(Node(-x, -y));

            error = 2 * (delta + y) - 1;
            if (delta < 0 && error <= 0) {
                delta += 2 * ++x + 1;
                continue;
            }

            error = 2 * (delta - x) - 1;
            if (delta > 0 && error > 0) {
                delta += 1 - 2 * --y;
                continue;
            }
            delta += 2 * (++x - y--);
        }

        for(int i = 0; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for(int i = circle_nodes.size() - 7; i > 0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for(int i = 7; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for(int i = circle_nodes.size() - 6; i>0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        circleNodes[k].pop_back();
    }
}

void LianSearch::calculatePivotCircle() {
    int add_x, add_y, num = pivotRadius + 0.5;
    Node node;
    for (int x = -num; x <= +num; ++x) {
        for (int y = -num; y <= +num; ++y) {
            add_x = x != 0 ? 1 : 0;
            add_y = y != 0 ? 1 : 0;
            if ((pow(2 * abs(x) - add_x, 2) + pow(2 * abs(y) - add_y, 2)) < pow(2 * pivotRadius, 2)) {
                pivotCircle.push_back(Node(x, y));
            }
        }
    }
    if (pivotCircle.empty()) {
        node.i = node.j = 0;
        pivotCircle.push_back(node);
    }
}

bool LianSearch::checkPivotCircle(const Map &map, const Node &center) {
    int i, j;
    for (Node node : pivotCircle) {
        i = center.i + node.i;
        j = center.j + node.j;
        if (map.CellOnGrid(i, j) && map.CellIsObstacle(i,j)) return false;
    }
    return true;
}


void LianSearch::calculateDistances() {
    int curDistance = distance;
    if(decreaseDistanceFactor > 1) {
        while(curDistance >= distanceMin) {
            listOfDistances.push_back(curDistance);
            curDistance = ceil(curDistance / decreaseDistanceFactor);
        }
    } else {
        listOfDistances.push_back(curDistance);
    }
    listOfDistancesSize = listOfDistances.size();
}

void LianSearch::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal) {
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x,y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;

    line.clear();

    if (x1 > x2 && y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) Rotate = 2;
        else if (dy < 0) {
            dy = -dy;
            std::swap(y1, y2);

            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;
            std::swap(x1, x2);

            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.push_back(Node(x, y2));
                StepVal += dy;
                if(StepVal >= dx) {
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                line.insert(line.begin(),Node(x2, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return;
    } else if(Rotate == 2) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.push_back(Node(x, y1));
                StepVal += dy;
                if(StepVal >= dx) {
                    ++y1;
                    StepVal -= dx;
                }
            }
            return;
        } else {
            for(y = y1; y <= y2; ++y) {
                line.push_back(Node(x1, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    ++x1;
                    StepVal -= dy;
                }
            }
            return;
        }
    } else if (Rotate == 3) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.insert(line.begin(),Node(x, y2));
                StepVal += dy;
                if(StepVal >= dx){
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                line.push_back(Node(x2, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return;
    }

    if(dx >= dy) {
        for(x = x1; x <= x2; ++x) {
            line.insert(line.begin(),Node(x, y1));
            StepVal += dy;
            if(StepVal >= dx){
                ++y1;
                StepVal -= dx;
            }
        }
    } else {
        for(y = y1; y <= y2; ++y) {
            line.insert(line.begin(),Node(x1, y));
            StepVal += dx;
            if(StepVal >= dy) {
                ++x1;
                StepVal -= dy;
            }
        }
    }
}

bool LianSearch::checkLineSegment(const Map &map, const Node &start, const Node &goal)
{
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x,y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;

    if (x1 > x2 && y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) Rotate = 2;
        else if (dy < 0) {
            dy = -dy;
            std::swap(y1, y2);
            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;
            std::swap(x1, x2);
            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if (dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y2)) return false;
                StepVal += dy;
                if (StepVal >= dx){
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for (y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x2, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return true;
    } else if(Rotate == 2) {
        if (dx >= dy) {
            y = y1;
            for (x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y1)) return false;
                StepVal += dy;
                if (StepVal >= dx) {
                    ++y1;
                    StepVal -= dx;
                }
            }
            return true;
        } else {
            for (y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x1, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    ++x1;
                    StepVal -= dy;
                }
            }
            return true;
        }
    } else if (Rotate == 3) {
        if (dx >= dy) {
            for (x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y2)) return false;
                StepVal += dy;
                if (StepVal >= dx) {
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x2, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return true;
    }

    if(dx >= dy) {
        for(x = x1; x <= x2; ++x) {
            if (map.CellIsObstacle(x, y1)) return false;
            StepVal += dy;
            if(StepVal >= dx){
                ++y1;
                StepVal -= dx;
            }
        }
    } else {
        for(y = y1; y <= y2; ++y) {
            if (map.CellIsObstacle(x1, y)) return false;
            StepVal += dx;
            if (StepVal >= dy) {
                ++x1;
                StepVal -= dy;
            }
        }
    }
    return true;
}

bool LianSearch::stopCriterion()
{
    if(open.is_empty()) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }

    if (closeSize > stepLimit && stepLimit > 0) {
        std::cout << "Algorithm esceeded step limit!" << std::endl;
        return true;
    }
    return false;
}

double LianSearch::getCost(int a_i, int a_j, int b_i, int b_j) {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}

SearchResult LianSearch::startSearch(Logger *Log, const Map &map) {
    std::chrono::time_point<std::chrono::system_clock> time_start, time_end;
    time_start = std::chrono::system_clock::now();

    calculateDistances();

    std::cout << "List of distances :";
    for (auto dist : listOfDistances) {
        std::cout << " " << dist;
    }
    std::cout << std::endl;

    open.resize(map.getHeight(), BT);
    Node curNode(map.start_i,map.start_j, 0.0, 0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * getCost(curNode.i,curNode.j, map.goal_i,map.goal_j);
    bool pathFound = false;
    open.add(curNode);
    calculateCircle((int) curNode.radius);
    calculatePivotCircle();

    while(!stopCriterion()) { // main cycle of the search
        curNode = open.getMin();
        open.pop(curNode);
        close.insert({curNode.convolution(map.getWidth()),curNode});
        ++closeSize;

        if (curNode.i == map.goal_i && curNode.j == map.goal_j) { // if current point is goal point - end of the cycle
            pathFound = true;
            break;
        }

        if(!expand(curNode, map) && listOfDistancesSize>1)
            while(curNode.radius>listOfDistances[listOfDistancesSize-1])
                if(tryToDecreaseRadius(curNode,map.getWidth()))
                    if(expand(curNode, map))
                        break;
        if(Log->loglevel >= CN_LOGLVL_LOW) Log->writeToLogOpenClose(open, close, map.getHeight());
    }
    if(Log->loglevel==CN_LOGLVL_MED) Log->writeToLogOpenClose(open, close, map.getHeight());

    time_end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count()) / 1000000000;
    sresult.nodescreated = open.get_size() + closeSize;
    sresult.numberofsteps = closeSize;
    if (pathFound) {
        float max_angle = makeAngles(curNode);
        makePrimaryPath(curNode);
        //hppath = smoothPath(hppath, map);
        makeSecondaryPath(curNode);
        sresult.pathfound = true;
        sresult.pathlength = curNode.g;
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.max_angle = max_angle;
        sresult.sections = hppath.size()-1;
        return sresult;
    } else {
        sresult.pathfound = false;
        return sresult;
    }
}

int LianSearch::tryToIncreaseRadius(Node curNode) {
    bool change = false;
    int i, k = 0;
    while (k < numOfParentsToIncreaseRadius) {
        if (curNode.parent != NULL) {
            if (curNode.radius == curNode.parent->radius) {
                ++k;
                curNode = *curNode.parent;
                continue;
            }
        }
        break;
    }
    if (k == numOfParentsToIncreaseRadius) {
        for (i = listOfDistancesSize-1; i >= 0; --i)
            if (curNode.radius == listOfDistances[i]) break;
        if(i > 0) change=true;
    }
    if(change) return listOfDistances[i-1];
    else return curNode.radius;
}

bool LianSearch::expand(const Node curNode, const Map &map) {
    int k;
    for(k = 0; k < listOfDistancesSize; ++k)
        if(listOfDistances[k] == curNode.radius) break;

    std::vector<Node> circle_nodes = circleNodes[k];
    int succi, succj;
    bool successors_are_fine = false, in_close;
    float cos_angle, angle, curvature;
    int position;

    if (curNode.parent != NULL) {
        std::vector<Node> successors;
        std::vector<int> succs;
        for (position = 0; position < circle_nodes.size(); ++position)
            if (curNode.parent->i == curNode.i + circle_nodes[position].i && curNode.parent->j == curNode.j + circle_nodes[position].j) break;

        if (position < circle_nodes.size() / 2) position += circle_nodes.size() / 2;
        else position -= circle_nodes.size() / 2;
        int k1 = position + 1;
        int k2 = position - 1;
        succs.push_back(position);
        for (int i = 0; i < circle_nodes.size(); ++i) {
            if (k1 >= circle_nodes.size()) k1 = 0;
            if (k2 < 0) k2 = circle_nodes.size() - 1;
            succs.push_back(k1++);
            succs.push_back(k2--);
            if (succs.size()>=circle_nodes.size()/2)
                break;
        }
        for (int i=0; i<circle_nodes.size()/2; i++) {
            succi = curNode.i + circle_nodes[succs[i]].i;
            succj = curNode.j + circle_nodes[succs[i]].j;

            if (!map.CellOnGrid(succi, succj)) continue;
            if (map.CellIsObstacle(succi, succj)) continue;

            int scalar_product = (curNode.j - curNode.parent->j) * (succj - curNode.j) + (curNode.parent->i - curNode.i) * (curNode.i - succi);
            cos_angle = (float) scalar_product / getCost(curNode.i, curNode.j, succi, succj);
            cos_angle = (float) cos_angle / (curNode.g - curNode.parent->g);

            if (cos_angle > 1) cos_angle = 1;
            if (cos_angle < -1) cos_angle = -1;

            angle = acos(cos_angle);
            curvature = fabs(angle);
            angle = angle * 180 / CN_PI_CONSTANT;

            if (fabs(angle) > angleLimit) break;
            successors.push_back(Node(circle_nodes[succs[i]].i, circle_nodes[succs[i]].j));
        }
        circle_nodes=successors;

    }
    for (unsigned int i = 0; i <= circle_nodes.size(); ++i) {
        Node newNode;
        if (i == circle_nodes.size()) { // for cycle iterations, where i equals circle_nodes size, check path to the goal cell
            if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode.radius) {
                succi = map.goal_i;
                succj = map.goal_j;
                if(curNode.parent != NULL) {
                    int scalar_product = (curNode.j - curNode.parent->j) * (succj - curNode.j) + (curNode.parent->i - curNode.i) * (curNode.i - succi);
                    cos_angle = (float) scalar_product / getCost(curNode.i, curNode.j, succi, succj);
                    cos_angle = (float) cos_angle / getCost(curNode.parent->i, curNode.parent->j, curNode.i, curNode.j);
                    angle = acos(cos_angle);
                    curvature = fabs(angle);
                    angle = angle * 180 / CN_PI_CONSTANT;
                    if (fabs(angle) > angleLimit) continue;
                }
            } else continue;
        } else {
            succi = curNode.i + circle_nodes[i].i;
            succj = curNode.j + circle_nodes[i].j;
        }
        if (!map.CellOnGrid(succi, succj)) continue;
        if (map.CellIsObstacle(succi, succj)) continue;

        newNode.i = succi;
        newNode.j = succj;
        newNode.parent = &(close.find(curNode.convolution(map.getWidth()))->second);
        newNode.radius = newNode.parent->radius;
        newNode.pathToParent = false;
        newNode.g = newNode.parent->g + getCost(curNode.i, curNode.j, newNode.i, newNode.j);
        newNode.c = newNode.parent->c + curvature;
        newNode.F = newNode.g + weight * getCost(succi, succj, map.goal_i, map.goal_j)
                              + curvatureHeuristicWeight * distance * newNode.c;
        newNode.pathToParent = checkLineSegment(map, *newNode.parent, newNode);


        if (pivotRadius > 0) {
            if (newNode.i != map.goal_i || newNode.j != map.goal_j) {
                newNode.pathToParent &= checkPivotCircle(map, newNode);
            }
        }

        if(newNode.pathToParent) {
            std::unordered_multimap<int, Node>::const_iterator it = close.find(newNode.convolution(map.getWidth()));
            if(it != close.end()) {
                in_close=false;
                auto range = close.equal_range(it->first);
                for(auto it = range.first; it != range.second; ++it)
                    if(it->second.parent && it->second.parent->i == curNode.i && it->second.parent->j == curNode.j)
                        in_close=true;

                if(!in_close) {
                    if(listOfDistancesSize > 1) newNode.radius = tryToIncreaseRadius(newNode);
                    open.add(newNode);
                    successors_are_fine = true;
                }
            } else {
                if(listOfDistancesSize > 1) newNode.radius=tryToIncreaseRadius(newNode);
                open.add(newNode);
                successors_are_fine = true;
            }
        }
    }
    return successors_are_fine;
}


bool LianSearch::tryToDecreaseRadius(Node& curNode, int width) {
    int i;
    for(i = listOfDistancesSize - 1; i >= 0; --i)
        if (curNode.radius == listOfDistances[i]) break;
    if (i < listOfDistancesSize - 1) {
        curNode.radius = listOfDistances[i + 1];
        auto it = close.find(curNode.convolution(width));
        auto range = close.equal_range(it->first);
        for(auto it = range.first; it != range.second; ++it) {
            if(it->second.parent && it->second.parent->i == curNode.parent->i && it->second.parent->j == curNode.parent->j) {
                it->second.radius = listOfDistances[i + 1];
                break;
            }
        }
        return true;
    }
    return false;
}

void LianSearch::makePrimaryPath(Node curNode) {
    hppath.push_front(curNode);
    curNode = *curNode.parent;
    do {
        hppath.push_front(curNode);
        curNode = *curNode.parent;

    } while (curNode.parent != nullptr);
    hppath.push_front(curNode);
}


bool LianSearch::checkAngle(Node dad, Node node, Node son) {
    double cos_angle = (node.j - dad.j) * (son.j - node.j) +
                       (node.i - dad.i) * (son.i - node.i);
    cos_angle /= getCost(son.i, son.j, node.i, node.j);
    cos_angle /= getCost(node.i, node.j, dad.i, dad.j);
    if (acos(cos_angle) <= angleLimit) {
        return true;
    }
    return false;
}

std::list<Node> LianSearch::smoothPath(const std::list<Node>& path, const Map& map) {
    std::list<Node> new_path;
    auto it = path.begin();
    auto curr_it = path.begin();
    Node start_section = *it;
    Node end_section = *it;
    bool first = true;
    Node previous;
    ++it;
    while (end_section != path.back()) {
        for (it; it != path.end(); ++it) {
            Node next = *(++it);
            --it;
            if (!first && !checkAngle(previous, start_section, *it)) continue;
            if (checkAngle(start_section, *it, next) && checkLineSegment(map, start_section, *it)) {
                end_section = *it;
                curr_it = it;
            }
        }
        new_path.push_back(start_section);
        previous = start_section;
        first = false;
        start_section = end_section;
        it = ++curr_it;
    }
    new_path.push_back(end_section);
    return new_path;
}

void LianSearch::makeSecondaryPath(Node curNode) {
    std::vector<Node> lineSegment;
    auto it = hppath.begin();
    Node parent = *it++;
    while (it != hppath.end()) {
        calculateLineSegment(lineSegment, parent, *it);
        lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
        parent = *it++;
    }
    lppath.push_front(*hppath.begin());
}

double LianSearch::makeAngles(Node curNode) {
    angles.clear();
    double cos_angle = 0;
    double max_angle = 0;
    do {
        if (curNode.parent != nullptr && curNode.parent->parent != nullptr) {
            double angle = 0;
            double dis1 = getCost(curNode.i, curNode.j, curNode.parent->i, curNode.parent->j);
            double dis2 = getCost(curNode.parent->i, curNode.parent->j, curNode.parent->parent->i, curNode.parent->parent->j);

            double scalarProduct = (curNode.j - curNode.parent->j) * (curNode.parent->j - curNode.parent->parent->j) +
                                   (curNode.parent->i - curNode.i) * (curNode.parent->parent->i - curNode.parent->i);

            if (dis1 != 0 && dis2 != 0) {
                cos_angle = (double) scalarProduct / getCost(curNode.parent->i, curNode.parent->j, curNode.i, curNode.j);
                cos_angle = (double) cos_angle / getCost(curNode.parent->parent->i, curNode.parent->parent->j, curNode.parent->i, curNode.parent->j);
                angle = acos(cos_angle);
                angle = angle * 180 / CN_PI_CONSTANT;
                if (angle > max_angle) max_angle = angle;
                angles.push_back(angle);
            }
        }
        curNode = *curNode.parent;
    } while(curNode.parent != nullptr);
    return max_angle;
}

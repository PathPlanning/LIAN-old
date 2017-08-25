#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "sNode.h"
#include "cMap.h"
#include "cSearch.h"
#include "Queues.h"

#include <vector>
#include <list>
#include <unordered_set>

class LianSearch : public cSearch {

public:

    // Конструктор с параметрами:
    LianSearch(double angleLimitDegree, int distance, float weight,
               unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
               float decreaseDistanceFactor, int distanceMin,
               float linecost, float pivotCircleRadius, int numOfParentsToIncreaseRadius, int breakingties);

    ~LianSearch();

    // Собственно алгоритм поиска
    SearchResult startSearch(cLogger *Log, const cMap &Map);

private:

    // Maximum allowed angle of turn. Stored in radians
    double angleLimit;

    double cosLimit;
    double sinLimit;
    // The upper estimate of distance in which the successors with allowed turn angle might be founded
    // The center of the sphere is in "no turn" successor
    double shift_radius;

    int distance; // Initial section length

    int numOfParentsToIncreaseRadius;

    //array of radius value. The greatest radius at the beginning
    std::vector<int> listOfDistances;

    // Heuristic weight coefficient
    float weight;

    // Другой эвристический коэффициент
    // Если проверяется местонахождение целевой точки относительно окружности
    // минимального радиуса, квадрат радиуса домножается на этот коэффициент
    float circleRadiusFactor;

    // Еще один эвристический коэффициент
    // Если используется эвристика, характеризующая отклонение траектории от прямой
    // на каждом шаге, то вычисляемая величина умножается на этот коэффициент
    float curvatureHeuristicWeight;

    float linecost; // "стоимость" движения по вертикали или горизонтали

    float pivotCircleRadius; // проверять ближайшие вершины на проходимость

    std::vector<Node> pivotCircleShifts; // Shifts for checking pivot circle

    unsigned int stepLimit; // Maximal number of steps made by algorithm

    float decreaseDistanceFactor;

    int distanceMin; // Minimal section length

    int breakingties;

    // Precomputed spheres for each radius with centers in (0, 0, 0)
    std::vector<std::unordered_set<Node, std::hash<Node>, NodeCoordEqual>> circleNodes;

    std::vector<float> angles;

    iOpen *open;

    // Resulting paths
    std::list<Node> hppath, lppath;

    std::unordered_multiset<Node, std::hash<Node>, NodeCoordEqual> close;

    // метод, вычисляющий окружность и записывающий
    // координаты узлов в список circleNodes (центр в точке [0, 0, 0] )
    // радиус - радиус окружности в клетках
    void calculateCircles();

    // Calclulating shifts for pivot cirle check
    void calculatePivotCircleShifts();

    // метод вычисляет предпочтительный радиус исходя из парамтеров карты
    int calculatePreferableRadius(const cMap &Map);

    void calculateDistances();

    // метод строит отрезок с помощью алгоритма Брезенхема
    // и проверяет его на наличие препятствий
    bool checkLineSegment(const cMap &Map, const Node &start, const Node &goal) const;

    // метод, вычисляющий "малую" окружность по брезенхему, для проверки свободного
    // пространства в опорных точках
    bool checkPivotCircle(const cMap &Map, const Node &center);

    // Checks is angle of turn in a path is correct
    bool checkTurnAngle(const Node &start, const Node &middlie, const Node &finish) const;

    int findRadiusInDistances(int radius) const;

    double calculateDistanceFromCellToCell(const Node &from, const Node &to) const;

    // критерий остановки. Возвращает истину, если цикл поиска
    // следует прекратить. Входящее значение - текущий номер шага алгоритма
    bool stopCriterion();

    int tryToIncreaseRadius(Node curNode);

    // Returns the pointer to the new node
    const Node *tryToDecreaseRadius(const Node *node_ptr, int width);

    void findSuccessors(const Node curNode, std::vector<Node> &successors, const cMap &Map);

    bool expand(const Node *Node_ptr, const cMap &Map);

    void makePrimaryPath(Node curNode, const cMap &map);

    void makeSecondaryPath(Node curNode);

    double makeAngles(Node curNode);

    // Checks successor and put it in open if necessary
    bool ProcessSuccessor(const Node *Node_ptr, Node successor, const cMap &Map);

    bool ProcessSuccessor(const Node *Node_ptr, const Node &successor, const cMap &Map, const Node &direct_succ,
                          int max_shift);

    void PostSmooth(const cMap &map);
};

#endif // LIANSEARCH_H

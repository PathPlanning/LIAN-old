#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "sNode.h"
#include "cList.h"
#include "cMap.h"
#include "cSearch.h"
#include "Queues.h"

#include <vector>
#include <unordered_set>

class LianSearch : public cSearch {

public:

    // Конструктор с параметрами:
    LianSearch(double angleLimitDegree, int distance, float weight,
               unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
               float decreaseDistanceFactor, int distanceMin,
               float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius);

    ~LianSearch();

    // Собственно алгоритм поиска
    SearchResult startSearch(cLogger *Log, const cMap &Map);

private:

    // Maximum allowed angle of turn. Stored in radians
    double angleLimit;

    double cosLimit;

    // Минимальная дистанция шага
    int distance;

    int numOfParentsToIncreaseRadius;

    //array of radius value. The greatest radius at the beginning
    std::vector<int> listOfDistances;
    int listOfDistancesSize;

    // Вес эвристики
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

    bool lesserCircle; // проверять ближайшие вершины на проходимость

    // максимальное число шагов цикла поиска
    unsigned int stepLimit;

    // число вершин в списках open и close
    unsigned int closeSize;

    // во сколько раз можно уменьшать изначальную длину шага
    float decreaseDistanceFactor;
    int distanceMin;

    // Виртуальные узлы, составляющие окружность
    std::vector<std::vector<Node>> circleNodes;

    std::vector<float> angles;

    iOpen *open;

    // Итоговый путь
    cList hppath, lppath;

    // список Close
    std::unordered_multiset<Node, std::hash<Node>, NodeCoordEqual> close;

    // метод, вычисляющий окружность по Брезенхему и записывающий
    // координаты узлов в список circleNodes (центр в точке [0, 0, 0] )
    // радиус - радиус окружности в клетках
    void calculateCircles();

    // метод вычисляет предпочтительный радиус исходя из парамтеров карты
    int calculatePreferableRadius(const cMap &Map);

    void calculateDistances();

    // метод строит отрезок с помощью алгоритма Брезенхема
    // и проверяет его на наличие препятствий
    bool checkLineSegment(const cMap &Map, const Node &start, const Node &goal);

    // метод, вычисляющий "малую" окружность по брезенхему, для проверки свободного
    // пространства в опорных точках
    bool checkLesserCircle(const cMap &Map, const Node &center, const float radius);

    // Checks is angle of turn in a path is correct
    bool checkTurnAngle(const Node &start, const Node &middlie, const Node &finish) const;

    int findRadiusInDistances(int radius) const;

    double calculateDistanceFromCellToCell(const Node &from, const Node &to) const;

    // критерий остановки. Возвращает истину, если цикл поиска
    // следует прекратить. Входящее значение - текущий номер шага алгоритма
    bool stopCriterion();

    int tryToIncreaseRadius(Node curNode);

    // Returns the pointer to the new node
    const Node* tryToDecreaseRadius(const Node *node_ptr, int width);

    void findSuccessors(const Node curNode, std::vector<Node> &successors, const cMap &Map);

    bool expand(const Node *Node_ptr, const cMap &Map);

    void makePrimaryPath(Node curNode);

    void makeSecondaryPath(Node curNode);

    double makeAngles(Node curNode);
};

#endif // LIANSEARCH_H
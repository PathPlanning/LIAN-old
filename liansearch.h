#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "node.h"
#include "map.h"
#include "search.h"
#include <vector>
#include <unordered_map>

class LianSearch : public Search
{

public:

    // ����������� � �����������:
    LianSearch(float angleLimit, int distance, float weight,
               unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
               float decreaseDistanceFactor, int distanceMin,
               float linecost, float pivotRadius, int numOfParentsToIncreaseRadius, int breakingties);

    // ���������� �������� ������
    SearchResult startSearch(Logger *Log, const Map &Map);

private:

    typedef std::list<Node> open_cluster_t;
    // ������������ ���� ����������
    float angleLimit;

    // ����������� ��������� ����
    int distance;

    int numOfParentsToIncreaseRadius;

    std::vector<int> listOfDistances;
    int listOfDistancesSize;

    // Heuristic weight
    float weight;

    int breakingties;

    // ������ ������������� �����������
    // ���� ����������� ��������������� ������� ����� ������������ ����������
    // ������������ �������, ������� ������� ����������� �� ���� �����������
    float circleRadiusFactor;

    // ��� ���� ������������� �����������
    // ���� ������������ ���������, ��������������� ���������� ���������� �� ������
    // �� ������ ����, �� ����������� �������� ���������� �� ���� �����������
    float curvatureHeuristicWeight;

    float linecost; // cost of straight move between two neighbour (be edge) cells

    float pivotRadius; // Radius of safety circle around every turn point.

    // Limit of steps made by the algorithm
    unsigned int stepLimit;

    // Sizes of open and close sets
    unsigned int closeSize, openSize;

    // the factor, on which DLIAN tries to decrease path section length
    float decreaseDistanceFactor;
    int distanceMin;

    // precomputed shifts of circles for every distance
    std::vector< std::vector<Node> > circleNodes;

    // Vector of nodes (shifts) for pivot security check
    std::vector<Cell> pivotCircle;

    std::vector<float> angles;

    // ����� Open + �������� ����
    std::list<Node> hppath, lppath;

    std::vector<open_cluster_t> open;

    std::vector<unsigned> cluster_minimums;
    size_t global_minimum;

    std::unordered_multimap<int, Node> close;

    void addOpen(Node &newNode);

    // Precompute shifts for every distance circle
    void calculateCircle(int radius);

    void calculatePivotCircle();

    int calculatePreferableRadius(const Map &map);

    void calculateDistances();

    Node findMin();
    void deleteMin(const Node &min);

    // Draw discrete line between two nodes
    void calculateLineSegment(std::list<Node> &line, const Node &start, const Node &goal);

    // Check if there are no obstacles on line between two nodes
    bool checkLineSegment(const Map &map, const Node &start, const Node &goal);

    // check that there are no obstacle in a safety radius from a turn point
    bool checkPivotCircle(const Map &map, const Node &center);

    double calculateDistanceFromCellToCell(Cell a, Cell b);

    bool stopCriterion();

    int tryToIncreaseRadius(Node curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void findSuccessors(const Node curNode,std::vector<Node> &successors, const Map &map);
    bool expand(const Node curNode, const Map &map);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    double makeAngles(Node curNode);
};

#endif // LIANSEARCH_H

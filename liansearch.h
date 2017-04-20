#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "sNode.h"
#include "cList.h"
#include "cMap.h"
#include "cSearch.h"
#include <vector>
#include <unordered_map>
class LianSearch : public cSearch
{

public:

    // ����������� � �����������:
    LianSearch(float angleLimit, int distance, float weight,
               unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
               float decreaseDistanceFactor, int distanceMin,
               float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius);

    ~LianSearch();

    // ���������� �������� ������
    SearchResult startSearch(cLogger *Log, const cMap &Map);

private:

    // ������������ ���� ����������
    float angleLimit;

    // ����������� ��������� ����
    int distance;

    int numOfParentsToIncreaseRadius;

    std::vector<int> listOfDistances;
    int listOfDistancesSize;

    // ��� ���������
    float weight;

    // ������ ������������� �����������
    // ���� ����������� ��������������� ������� ����� ������������ ����������
    // ������������ �������, ������� ������� ����������� �� ���� �����������
    float circleRadiusFactor;

    // ��� ���� ������������� �����������
    // ���� ������������ ���������, ��������������� ���������� ���������� �� ������
    // �� ������ ����, �� ����������� �������� ���������� �� ���� �����������
    float curvatureHeuristicWeight;

    float linecost; // "���������" �������� �� ��������� ��� �����������

    bool lesserCircle; // ��������� ��������� ������� �� ������������

    // ������������ ����� ����� ����� ������
    unsigned int stepLimit;

    // ����� ������ � ������� open � close
    unsigned int closeSize, openSize;

    // �� ������� ��� ����� ��������� ����������� ����� ����
    float decreaseDistanceFactor;
    int distanceMin;

    // ����������� ����, ������������ ����������
    std::vector< std::vector<Node> > circleNodes;

    std::vector<float> angles;

    // ����� Open + �������� ����
    cList *open, hppath, lppath;

    // ������ Close
    std::unordered_multimap<int, Node> close;

    void addOpen(Node &newNode);

    // �����, ����������� ���������� �� ���������� � ������������
    // ���������� ����� � ������ circleNodes (����� � ����� [0, 0] )
    // ������ - ������ ���������� � �������
    void calculateCircle(int radius);

    // ����� ��������� ���������������� ������ ������ �� ���������� �����
    int calculatePreferableRadius(const cMap &Map);

    void calculateDistances();

    Node findMin(int size);

    // ����� ������ ������� � ������� ��������� ����������
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);

    // ����� ������ ������� � ������� ��������� ����������
    // � ��������� ��� �� ������� �����������
    bool checkLineSegment(const cMap &Map, const Node &start, const Node &goal);

    // �����, ����������� "�����" ���������� �� ����������, ��� �������� ����������
    // ������������ � ������� ������
    bool checkLesserCircle(const cMap &Map, const Node &center, const float radius);

    double calculateDistanceFromCellToCell(int start_i, int start_j, int fin_i, int fin_j);

    // �������� ���������. ���������� ������, ���� ���� ������
    // ������� ����������. �������� �������� - ������� ����� ���� ���������
    bool stopCriterion();

    int tryToIncreaseRadius(Node curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void findSuccessors(const Node curNode,std::vector<Node> &successors, const cMap &Map);
    bool expand(const Node curNode, const cMap &Map);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    double makeAngles(Node curNode);
};

#endif // LIANSEARCH_H

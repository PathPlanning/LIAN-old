#ifndef LIANSEARCH_QUEUES_H
#define LIANSEARCH_QUEUES_H

#include <list>
#include <vector>

#include "sNode.h"

class iOpen {
protected:
    virtual bool less(const Node &x, const Node &y) const;
public:
    iOpen() {};

    virtual bool Insert(const Node &NewNode) = 0;

    virtual Node FindMin() const = 0;

    virtual void DeleteMin() = 0;

    virtual size_t size() const = 0;

    virtual bool empty() const = 0;
};

class SortedList : public iOpen {
private:
    std::vector<std::list<Node>> data;
    size_t size_;

    mutable size_t min_pos;

public:
    SortedList() = default;

    SortedList(size_t size);

    virtual bool Insert(const Node &NewNode);

    virtual Node FindMin() const;

    virtual void DeleteMin();

    virtual size_t size() const;

    virtual bool empty() const;
};

class MinHeap : public iOpen {
private:
    std::vector<Node> _vector;
    bool unique_;
    int breaking_tie;

    virtual bool less(const Node &x, const Node &y) const;

    void BubbleDown(size_t index);

    void BubbleUp(size_t index);

    void Heapify();

public:
    MinHeap();

    MinHeap(int breakingtie, bool unique_nodes);

    template<typename T>
    MinHeap(T begin, T end, int breakingtie, bool unique_nodes);

    virtual bool Insert(const Node &newValue);

    virtual Node FindMin() const;

    virtual void DeleteMin();

    virtual size_t size() const;

    virtual bool empty() const;

    //MinHeap& operator=(const MinHeap& other) = default;

    template<typename T>
    void ReplaceData(T begin, T end);
};

class ClusteredHeap : public iOpen {
private:
    std::vector<MinHeap> data;

    size_t size_;

    mutable size_t min_pos;

public:
    ClusteredHeap() = default;

    ClusteredHeap(size_t size);

    virtual bool Insert(const Node &NewNode);

    virtual Node FindMin() const;

    virtual void DeleteMin();

    virtual size_t size() const;

    virtual bool empty() const;

};

#endif //LIANSEARCH_QUEUES_H

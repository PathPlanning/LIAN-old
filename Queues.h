#ifndef LIANSEARCH_QUEUES_H
#define LIANSEARCH_QUEUES_H

#include <list>
#include <vector>
#include <unordered_set>

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

class ClusteredSets : public iOpen {
private:
    std::vector<Node> loc_mins;
    std::vector<std::unordered_multiset<Node, std::hash<Node>, NodeCoordEqual>> data;
    size_t size_;
    mutable size_t min_pos;

public:
    ClusteredSets() = default;

    ClusteredSets(size_t size);

    virtual bool Insert(const Node &NewNode);

    virtual Node FindMin() const;

    virtual void DeleteMin();

    virtual size_t size() const;

    virtual bool empty() const;
};

#endif //LIANSEARCH_QUEUES_H

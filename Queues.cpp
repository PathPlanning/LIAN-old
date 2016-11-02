#include "Queues.h"
#include "sNode.h"
#include "gl_const.h"

bool iOpen::less(const Node &x, const Node &y) const {
    if (x.F == y.F) {
        return x.g > y.g;
    }
    return x.F < y.F;
}

SortedList::SortedList(size_t size) : data(size), size_(0), min_pos(size) {}

size_t SortedList::size() const {
    return size_;
}

bool SortedList::empty() const {
    return (size_ == 0);
}

bool SortedList::Insert(const Node &newNode) {
    std::list<Node>::iterator iter, pos;

    bool posFound = false;

    pos = data[newNode.i].end();

    if (data[newNode.i].empty()) {
        data[newNode.i].push_back(newNode);
        ++size_;
        return true;
    }

    for (iter = data[newNode.i].begin(); iter != data[newNode.i].end(); ++iter) {
        if (!posFound && !less(*iter, newNode)) {
            pos = iter;
            posFound = true;
        }

        if (iter->j == newNode.j && iter->z == newNode.z && (iter->Parent->i == newNode.Parent->i) &&
            (iter->Parent->j == newNode.Parent->j) &&
            (iter->Parent->z == newNode.Parent->z)) {
            if (newNode.F >= iter->F) {
                return false;
            } else {
                if (pos == iter) {
                    iter->g = newNode.g;
                    iter->F = newNode.F;
                    iter->c = newNode.c;
                    iter->radius = newNode.radius;
                    return true;
                }
                data[newNode.i].erase(iter);
                --size_;
                break;
            }
        }
    }
    ++size_;
    data[newNode.i].insert(pos, newNode);
    return true;
}

Node SortedList::FindMin() const {
    Node min;
    min.F = -1;
    for (size_t i = 0; i < data.size(); i++) {
        if (!data[i].empty())
            if (min.F == -1 || less(*data[i].begin(), min)) {
                min = *data[i].begin();
                min_pos = i;
            }
    }
    return min;
}

void SortedList::DeleteMin() {
    if (min_pos >= data.size()) {
        FindMin();
    }
    data[min_pos].pop_front();
    --size_;
    min_pos = data.size();
}


MinHeap::MinHeap() {}

MinHeap::MinHeap(int breakingtie, bool unique_nodes) : _vector(), unique_(unique_nodes), breaking_tie(breakingtie) {}

template<typename T>
MinHeap::MinHeap(T begin, T end, int breakingtie, bool unique_nodes) : _vector(begin, end), unique_(unique_nodes),
                                                                       breaking_tie(breakingtie) {
    Heapify();
}

bool MinHeap::less(const Node &x, const Node &y) const {
    if (x.F < y.F) {
        return true;
    } else if (x.F == y.F) {
        switch (breaking_tie) {
            case CN_SP_BT_GMIN:
                return x.g < y.g;
            default:
            case CN_SP_BT_GMAX:
                return x.g > y.g;
        }
    }
    return false;
}

void MinHeap::Heapify() {
    for (long long i = _vector.size() - 1; i >= 0; --i) {
        BubbleDown(i);
    }
}

void MinHeap::BubbleDown(size_t index) {
    size_t length = _vector.size();
    size_t leftChildIndex = (index << 1) + 1;
    size_t rightChildIndex = leftChildIndex + 1;

    if (leftChildIndex >= length)
        return; //index is a leaf

    size_t minIndex = index;

    if (less(_vector[leftChildIndex], _vector[index])) {
        minIndex = leftChildIndex;
    }

    if ((rightChildIndex < length) && less(_vector[rightChildIndex], _vector[minIndex])) {
        minIndex = rightChildIndex;
    }

    if (minIndex != index) {
        //need to swap
        Node temp = _vector[index];
        _vector[index] = _vector[minIndex];
        _vector[minIndex] = temp;
        BubbleDown(minIndex);
    }
}

void MinHeap::BubbleUp(size_t index) {
    if (index == 0)
        return;

    size_t parentIndex = (index - 1) / 2;

    if (less(_vector[index], _vector[parentIndex])) {
        Node temp = _vector[parentIndex];
        _vector[parentIndex] = _vector[index];
        _vector[index] = temp;
        BubbleUp(parentIndex);
    }
}

bool MinHeap::Insert(const Node &newValue) {
    size_t found = _vector.size();
    if (unique_) {
        for (size_t i = 0; i != _vector.size(); ++i) {
            if (_vector[i].i == newValue.i && _vector[i].j == newValue.j && _vector[i].z == newValue.z) {
                if (newValue.g < _vector[i].g) {
                    found = i;
                }
                break;
            }
        }
    }
    bool inserted = false;
    if (found == _vector.size()) {
        _vector.push_back(newValue);
        inserted = true;
    }

    BubbleUp(found);
    return inserted;
}

Node MinHeap::FindMin() const {
    return _vector[0];
}

void MinHeap::DeleteMin() {
    size_t length = _vector.size();

    if (length == 0) {
        return;
    }

    _vector[0] = _vector[length - 1];
    _vector.pop_back();

    BubbleDown(0);
}

size_t MinHeap::size() const {
    return _vector.size();
}

bool MinHeap::empty() const {
    return _vector.empty();
}

template<typename T>
void MinHeap::ReplaceData(T begin, T end) {
    _vector = std::vector<Node>(begin, end);
    Heapify();
}


ClusteredHeap::ClusteredHeap(size_t size) : data(size, MinHeap(CN_SP_BT_GMAX, false)), size_(0) {}

size_t ClusteredHeap::size() const {
    return size_;
}

bool ClusteredHeap::empty() const {
    return (size_ == 0);
}

bool ClusteredHeap::Insert(const Node &newNode) {
    data[newNode.i].Insert(newNode);
    ++size_;
    return true;
}

Node ClusteredHeap::FindMin() const {
    Node min;
    min.F = -1;
    for (size_t i = 0; i < data.size(); i++) {
        if (!data[i].empty())
            if (min.F == -1 || less(data[i].FindMin(), min)) {
                min = data[i].FindMin();
                min_pos = i;
            }
    }
    return min;
}

void ClusteredHeap::DeleteMin() {
    if (min_pos >= data.size()) {
        FindMin();
    }
    data[min_pos].DeleteMin();
    --size_;
    min_pos = data.size();
}

ClusteredSets::ClusteredSets(size_t size) : loc_mins(size), data(size), size_(0), min_pos(size) {}

size_t ClusteredSets::size() const {
    return size_;
}

bool ClusteredSets::empty() const {
    return (size_ == 0);
}

bool ClusteredSets::Insert(const Node &NewNode) {
    auto range = data[NewNode.i].equal_range(NewNode);
    bool node_found = false;
    bool updated = false;

    NodeCoordEqual equal;
    for (auto it = range.first; it != range.second; ++it) {
        if (equal(NewNode, *it)) {
            if ((NewNode.Parent == nullptr && it->Parent == nullptr ) || equal(*NewNode.Parent, *it->Parent)) {
                node_found = true;
                if (NewNode.F < it->F) {
                    updated = true;
                    data[NewNode.i].erase(it);
                    data[NewNode.i].insert(NewNode);
                }
                break;
            }
        }
    }
    if (!node_found) {
        updated = true;
        data[NewNode.i].insert(NewNode);
        ++size_;
    }
    if (data[NewNode.i].size() == 1 || less(NewNode, loc_mins[NewNode.i])) {
        loc_mins[NewNode.i] = NewNode;
    }
    return updated;
}

Node ClusteredSets::FindMin() const {
    for (min_pos = 0; data[min_pos].empty(); ++min_pos) {}
    for (size_t i = min_pos + 1; i < loc_mins.size(); ++i) {
        if (!data[i].empty() && less(loc_mins[i], loc_mins[min_pos])) {
            min_pos = i;
        }
    }
    return loc_mins[min_pos];
}

void ClusteredSets::DeleteMin() {
    if (min_pos == loc_mins.size()) {
        FindMin();
    }

    NodeCoordEqual equal;
    Node min = loc_mins[min_pos];
    auto range = data[min_pos].equal_range(min);
    for (auto it = range.first; it != range.second; ++it) {
        if (equal(min, *it)) {
            if ((min.Parent == nullptr && it->Parent == nullptr ) || equal(*min.Parent, *it->Parent)) {
                data[min_pos].erase(it);
                --size_;
                break;
            }
        }
    }

    if (!data[min_pos].empty()) {
        auto it = data[min_pos].begin();
        min = *(it++);
        for (; it != data[min_pos].end(); ++it) {
            if (less(*it, min)) {
                min = *it;
            }
        }

        loc_mins[min_pos] = min;
    }
    min_pos = loc_mins.size();
}
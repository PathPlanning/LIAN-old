#include "Queues.h"
#include "sNode.h"
#include "gl_const.h"

bool iOpen::less(const Node &x, const Node &y) const {
    if (x.F == y.F) {
        switch (breakingties) {
            default:
            case CN_SP_BT_GMAX:
                return x.g > y.g;
            case CN_SP_BT_GMIN:
                return x.g < y.g;
        }
    }
    return x.F < y.F;
}

SortedList::SortedList(size_t size, int breakingties) : data(size), size_(0), min_pos(size),
                                                                        iOpen(breakingties) {}

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

std::vector<Node> SortedList::dump() const {
    std::vector<Node> res;
    for (size_t i = 0; i < data.size(); ++i) {
        res.insert(res.end(), data[i].begin(), data[i].end());
    }
    return res;
}

ClusteredSets::ClusteredSets(size_t size, int breakingties) : loc_mins(size), data(size), size_(0),
                                                                              min_pos(size),
                                                                              iOpen(breakingties) {}

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
        if ((NewNode.Parent == nullptr && it->Parent == nullptr) || equal(*NewNode.Parent, *it->Parent)) {
            node_found = true;
            if (NewNode.F < it->F) {
                updated = true;
                data[NewNode.i].erase(it);
                data[NewNode.i].insert(NewNode);
            }
            break;
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
            if ((min.Parent == nullptr && it->Parent == nullptr) || equal(*min.Parent, *it->Parent)) {
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

std::vector<Node> ClusteredSets::dump() const {
    std::vector<Node> res;
    for (size_t i = 0; i < data.size(); ++i) {
        res.insert(res.end(), data[i].begin(), data[i].end());
    }
    return res;
}
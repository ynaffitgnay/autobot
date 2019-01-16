#ifndef DSL_P_QUEUE_H
#define DSL_P_QUEUE_H

#include <queue>
#include <vector>
#include <planning/structures/PNCmp.h>

// A derived priority queue to allow for removing a node from the top and updating
class DSLPQueue : public std::priority_queue<PathNode*, std::vector<PathNode*>, PNCmp> {
  public:
    bool remove(PathNode* value) {
      auto it = std::find(this->c.begin(), this->c.end(), value);
      if (it != this->c.end()) {
        this->c.erase(it);
        std::make_heap(this->c.begin(), this->c.end(), this->comp);
        return true;
      } else {
        return false;
      }
    }

    bool contains(PathNode* value) {
      auto it = std::find(this->c.begin(), this->c.end(), value);
      if (it != this->c.end()) {
        return true;
      } else {
        return false;
      }
    }
};
    

#endif

#ifndef DSL_P_QUEUE_H
#define DSL_P_QUEUE_H

#include <queue>
#include <vector>
#include <planning/structures/PNCmp.h>

//template<typename T,
//         typename Cont,
//         typename Cmp>
class DSLPQueue : public std::priority_queue<PathNode*, std::vector<PathNode*>, PNCmp> {
  public:
  //DSLPQueue() {priority_queue<PathNode*, vector<PathNode*>, PNCmp>()};
    bool remove(const PathNode*& value) {
      auto it = std::find(this->c.begin(), this->c.end(), value);
      if (it != this->c.end()) {
        this->c.erase(it);
        std::make_heap(this->c.begin(), this->c.end(), this->comp);
        return true;
      } else {
        return false;
      }
    }
};
    

#endif

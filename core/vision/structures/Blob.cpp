#include <vision/structures/Blob.h>

bool sortBlobAreaPredicate(Blob& left, Blob& right) {
  return left.total > right.total;
}



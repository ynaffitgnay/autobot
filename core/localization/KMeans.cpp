#include <localization/KMeans.h>

KMeans::KMeans(MemoryCache& cache, TextLogger*& tlogger, int k)
  : cache_(cache), tlogger_(tlogger), k_(k) {
}

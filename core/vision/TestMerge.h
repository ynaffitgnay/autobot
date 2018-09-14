#ifndef TEST_MERGE_H
#define TEST_MERGE_H


//#include "VisionPointAlt.h"
#include <vision/structures/VisionPointAlt.h>
#include <vision/structures/Blob.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


class TestMerge {
  /*
  * Merging struct for runs in Union-Find
  */
  struct MergeNode {
    VisionPointAlt data;  // The run data
    MergeNode *parent;    // Pointer to the parent
    int rank;       // Rank for comparing sets to merge
  };
  

  private:

    /*
    * Just a quick fake data setup
    */
    void createFakeVPAData(std::vector<VisionPointAlt>& vpav);

    /*
    * Reads in the fake data
    */
    void getTestFile(std::vector<unsigned char>& test_data);

    /*
    * Main method for merging
    */
    void mergeRuns(std::vector<Blob>& blobs);

    /*
    * Checking adjacency
    */
    void checkAdj(MergeNode node, std::vector<MergeNode> checkAdj);

    /*
    * find method for Union-Find
    */
    MergeNode * findParent(MergeNode node);

    /*
    * Union by rank for Union-Find
    */
    void unionByRank(MergeNode& a, MergeNode& b);

    
    std::vector<VisionPointAlt> vpav_;  // The runs from RLE
    std::vector<Blob> blobs_;     // The hopefully filled in blobs from this whole mess
  public:
    TestMerge();  // Default constructor
      ~TestMerge(); // Default destructor
    

};
#endif

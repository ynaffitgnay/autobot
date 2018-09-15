#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <math/Point.h>
#include <list>
#include <vector>

#include <memory/TextLogger.h>
//#include <vision/ImageConstants.h>
#include <vision/VisionConstants.h>
//#include <vision/structures/VisionPoint.h>
#include <vision/structures/VisionPointAlt.h>
#include <vision/structures/Blob.h>
#include <vision/structures/VisionParams.h>
#include <vision/structures/HorizonLine.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/VisionBlocks.h>
#include <vision/structures/FocusArea.h>
#include <vision/Macros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <common/Profiling.h>

/// @ingroup vision
class Classifier {
  // struct MergeNode {
  //   VisionPointAlt data;  // The run data
  //   MergeNode* parent;    // Pointer to the parent
  //   int rank;        // Rank for comparing sets to merge

  //   MergeNode() : data(VisionPointAlt(0,0,0)), parent(this), rank(-1) {}
  // };
  
 public:
  Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera);
  ~Classifier();
  void init(TextLogger* tl){textlogger = tl;};

  bool classifyImage(unsigned char*);
  inline Color xy2color(int x, int y) {
    return (Color)segImg_[y * iparams_.width + x];
  }
  void getStepSize(int&,int&) const;
  void makeBlobs(std::vector<Blob>& blobs);

 private:
  void classifyImage(const FocusArea& area, unsigned char*);
  bool setImagePointers();
  void constructRuns(std::vector<VisionPointAlt>& runs);

  /*
  * Main method for merging
  */
  void mergeRuns(std::vector<VisionPointAlt>& runs);

  /*
  * Checking adjacency
  */
  void checkAdj(VisionPointAlt& node, std::vector<VisionPointAlt>::iterator row_begin, std::vector<VisionPointAlt>::iterator row_end);

  /*
  * find method for Union-Find
  */
  VisionPointAlt * findParent(VisionPointAlt& node);

  /*
  * Make lists of parents
  */
  void makeParentLists(std::vector<VisionPointAlt>& runs, std::vector<std::vector<VisionPointAlt*>>& parents);



  /*
  * Union by rank for Union-Find
  */
  void unionByRank(VisionPointAlt& a, VisionPointAlt& b);

  
  const VisionBlocks& vblocks_;
  const VisionParams& vparams_;
  const ImageParams& iparams_;
  const Camera::Type& camera_;
  bool initialized_;
  TextLogger* textlogger;

  unsigned char* img_;
  unsigned char* segImg_, *segImgLocal_;
  HorizonLine horizon_;
  unsigned char* colorTable_;
};
#endif

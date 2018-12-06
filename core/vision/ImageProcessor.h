#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <vector>
#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <common/RobotCalibration.h>
#include <math/Pose3D.h>
#include <vision/structures/VisionParams.h>
#include <vision/structures/Blob.h>

class Classifier;
class BeaconDetector;
class ObstacleDetector;

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    ~ImageProcessor();
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    std::unique_ptr<Classifier> color_segmenter_;
    std::unique_ptr<BeaconDetector> beacon_detector_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    std::vector<Blob> blobs_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(const RobotCalibration& calibration);
    void enableCalibration(bool value);
    void updateTransform();
    bool isImageLoaded();
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;
    
    std::unique_ptr<RobotCalibration> calibration_;
    bool enableCalibration_;

    //void saveImg(std::string filepath);
    int topFrameCounter_ = 0;
    int bottomFrameCounter_ = 0;
};

#endif

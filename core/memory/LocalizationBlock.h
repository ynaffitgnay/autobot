#ifndef LOCALIZATION_BLOCK_H
#define LOCALIZATION_BLOCK_H
#pragma once

#include <Eigen/Core>
#include <memory/MemoryBlock.h>
#include <math/Geometry.h>
#include <common/Particle.h>
#include <schema/gen/LocalizationBlock_generated.h>
#define STATE_SIZE 2
#define COV_SIZE (STATE_SIZE * STATE_SIZE)
#define MAX_MODELS_IN_MEM 1
#define MODEL_STATE_SIZE (MAX_MODELS_IN_MEM * STATE_SIZE)
#define MODEL_COV_SIZE (MAX_MODELS_IN_MEM * STATE_SIZE * STATE_SIZE)
#define PART_SIZE (200)

class Particle;

DECLARE_INTERNAL_SCHEMA(struct LocalizationBlock : public MemoryBlock {
  public:
    SCHEMA_METHODS(LocalizationBlock);
    LocalizationBlock();
    SCHEMA_FIELD(Point2D player_);
    //SCHEMA_FIELD(Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> state);
    //SCHEMA_FIELD(Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> covariance);
    mutable SCHEMA_FIELD(std::array<float, STATE_SIZE> state_data);
    Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign> state;

    mutable SCHEMA_FIELD(std::array<float, COV_SIZE> covariance_data);
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign> covariance;

    mutable SCHEMA_FIELD(std::array<Particle, PART_SIZE> particles_data);
    std::vector<Particle> particles;

  SCHEMA_PRE_SERIALIZATION({
      std::copy(
        __source_object__.state.data(), 
        __source_object__.state.data() + __source_object__.state.size(), 
        __source_object__.state_data.data()
      );
      std::copy(
        __source_object__.covariance.data(), 
        __source_object__.covariance.data() + __source_object__.covariance.size(), 
        __source_object__.covariance_data.data()
      );
      std::copy(
        __source_object__.particles.data(), 
        __source_object__.particles.data() + __source_object__.particles.size(), 
        __source_object__.particles_data.data()
      );
  });
  SCHEMA_POST_DESERIALIZATION({
      std::copy(
        __target_object__.state_data.data(), 
        __target_object__.state_data.data() + __target_object__.state.size(),
        __target_object__.state.data()
      );
      std::copy(
        __target_object__.covariance_data.data(), 
        __target_object__.covariance_data.data() + __target_object__.covariance.size(), 
        __target_object__.covariance.data()
      );
      std::copy(
        __target_object__.particles_data.data(), 
        __target_object__.particles_data.data() + __target_object__.particles.size(),
        __target_object__.particles.data()
      );
  });

    Point2D getBallPosition();
    Point2D getBallVel();
    Eigen::Matrix2f getBallCov();
    //SCHEMA_FIELD(std::vector<Particle> particles);
    //void serialize(StreamBuffer& buffer, std::string);
    //bool deserialize(const StreamBuffer& buffer, std::string);
});

#endif

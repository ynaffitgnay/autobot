#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#pragma once

// TODO: fix this definition
struct Obstacle {
  Pose2D center;
  float width;
  float height;

  Obstacle(Pose2D center, int height, int width) : center(center), width(width), height(height) { };
};

#endif // OBSTACLE_H_

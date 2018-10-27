#pragma once
#ifndef PARTICLE_
#define PARTICLE_

#include <common/Serialization.h>
#include <schema/gen/Particle_generated.h>

DECLARE_INTERNAL_SCHEMA(struct Particle {
  SCHEMA_METHODS(Particle);
  SCHEMA_FIELD(float x); // X coordinate
  SCHEMA_FIELD(float y); // Y coordinate
  SCHEMA_FIELD(float t); // Theta (Orientation)
  SCHEMA_FIELD(float w); // Weight
});
#endif

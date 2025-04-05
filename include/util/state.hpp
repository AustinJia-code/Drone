/**
 * @file state.hpp
 * @brief Various states and enums
 */

#pragma once

enum class FlightMode : unsigned char 
{
  MANUAL          = 0,
  POSITION_HOLD   = 1,
  TRACK           = 2
};

struct Vec3D
{
  float x;
  float y;
  float z;
};

struct Pose3D
{
  Vec3D position;
  Vec3D heading;
};
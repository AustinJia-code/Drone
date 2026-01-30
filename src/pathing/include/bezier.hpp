/**
 * @file bezier.hpp
 * @brief Low-level cubic Bezier math primitives
 */

#pragma once

#include <cmath>
#include <array>

/**
 * Simple 3D vector with arithmetic operators
 */
struct Vec3
{
  float x = 0, y = 0, z = 0;

  Vec3 operator+ (const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator- (const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator* (float s)       const { return {x * s, y * s, z * s}; }

  float length () const { return std::sqrt (x * x + y * y + z * z); }
};

inline Vec3 operator* (float s, const Vec3& v) { return v * s; }

/**
 * Cubic Bezier curve defined by 4 control points
 *
 *   B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
 */
class CubicBezier
{
public:
  std::array<Vec3, 4> P;

  CubicBezier (Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
    : P {p0, p1, p2, p3} {}

  /**
   * Evaluate position at parameter t in [0,1]
   */
  Vec3 evaluate (float t) const
  {
    float u  = 1.0f - t;
    float u2 = u * u;
    float t2 = t * t;

    return u2 * u * P[0]
         + 3.0f * u2 * t * P[1]
         + 3.0f * u * t2 * P[2]
         + t2 * t * P[3];
  }

  /**
   * First derivative B'(t) — tangent direction
   */
  Vec3 tangent (float t) const
  {
    float u = 1.0f - t;

    return 3.0f * u * u * (P[1] - P[0])
         + 6.0f * u * t * (P[2] - P[1])
         + 3.0f * t * t * (P[3] - P[2]);
  }

  /**
   * Approximate arc length via chord summation
   */
  float arc_length (int steps = 64) const
  {
    float len = 0.0f;
    Vec3 prev = P[0];

    for (int i = 1; i <= steps; ++i)
    {
      float t = static_cast<float> (i) / static_cast<float> (steps);
      Vec3 cur = evaluate (t);
      len += (cur - prev).length ();
      prev = cur;
    }

    return len;
  }
};

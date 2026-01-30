/**
 * @file bezier_path.hpp
 * @brief Piecewise cubic Bezier spline with arc-length parameterization
 */

#pragma once

#include <vector>
#include <algorithm>
#include "bezier.hpp"
#include "pose.hpp"

/**
 * Smooth path through a sequence of waypoints using Catmull-Rom to Bezier.
 * Provides arc-length parameterized sampling for uniform-speed traversal.
 */
class BezierPath
{
public:
  /**
   * Build a piecewise cubic Bezier spline from ordered waypoints.
   * Uses Catmull-Rom tangent estimation for C1 continuity.
   */
  explicit BezierPath (const std::vector<Pose>& waypoints)
  {
    std::size_t n = waypoints.size ();
    if (n < 2)
      return;

    // Convert waypoints to Vec3
    std::vector<Vec3> W (n);
    for (std::size_t i = 0; i < n; ++i)
      W[i] = {waypoints[i].x, waypoints[i].y, waypoints[i].z};

    // Compute tangents via central differences (forward/backward at endpoints)
    std::vector<Vec3> T (n);
    T[0] = W[1] - W[0];
    T[n - 1] = W[n - 1] - W[n - 2];

    for (std::size_t i = 1; i < n - 1; ++i)
      T[i] = (W[i + 1] - W[i - 1]) * 0.5f;

    // Build cubic Bezier segments with Catmull-Rom control points
    for (std::size_t i = 0; i < n - 1; ++i)
    {
      Vec3 p0 = W[i];
      Vec3 p1 = W[i] + T[i] * (1.0f / 3.0f);
      Vec3 p2 = W[i + 1] - T[i + 1] * (1.0f / 3.0f);
      Vec3 p3 = W[i + 1];

      segments_.emplace_back (p0, p1, p2, p3);
    }

    // Precompute cumulative arc-length table
    cumulative_lengths_.resize (segments_.size () + 1);
    cumulative_lengths_[0] = 0.0f;

    for (std::size_t i = 0; i < segments_.size (); ++i)
      cumulative_lengths_[i + 1] = cumulative_lengths_[i] + segments_[i].arc_length (64);

    total_length_ = cumulative_lengths_.back ();
  }

  /**
   * Sample position at arc-length distance s along the path
   */
  Vec3 sample (float s) const
  {
    auto [seg, t] = locate (s);
    return segments_[seg].evaluate (t);
  }

  /**
   * Sample tangent at arc-length distance s along the path
   */
  Vec3 sample_tangent (float s) const
  {
    auto [seg, t] = locate (s);
    return segments_[seg].tangent (t);
  }

  float total_length () const { return total_length_; }

private:
  std::vector<CubicBezier> segments_;
  std::vector<float> cumulative_lengths_;
  float total_length_ = 0.0f;

  /**
   * Map arc-length distance s to {segment_index, local t} via bisection
   */
  std::pair<std::size_t, float> locate (float s) const
  {
    s = std::clamp (s, 0.0f, total_length_);

    // Find which segment contains this arc-length
    std::size_t seg = 0;
    for (std::size_t i = 0; i < segments_.size (); ++i)
    {
      if (s <= cumulative_lengths_[i + 1])
      {
        seg = i;
        break;
      }
    }

    // Local arc-length within this segment
    float seg_len = cumulative_lengths_[seg + 1] - cumulative_lengths_[seg];
    float local_s = s - cumulative_lengths_[seg];

    if (seg_len < 1e-6f)
      return {seg, 0.0f};

    // Bisection to invert arc-length → t (16 iterations)
    float target = local_s;
    float lo = 0.0f, hi = 1.0f;

    for (int iter = 0; iter < 16; ++iter)
    {
      float mid = (lo + hi) * 0.5f;

      // Compute arc-length from 0 to mid
      float len = 0.0f;
      Vec3 prev = segments_[seg].evaluate (0.0f);
      constexpr int steps = 32;

      for (int j = 1; j <= steps; ++j)
      {
        float tj = mid * static_cast<float> (j) / static_cast<float> (steps);
        Vec3 cur = segments_[seg].evaluate (tj);
        len += (cur - prev).length ();
        prev = cur;
      }

      if (len < target)
        lo = mid;
      else
        hi = mid;
    }

    return {seg, (lo + hi) * 0.5f};
  }
};

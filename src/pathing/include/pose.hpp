#pragma once
#include <mutex>

struct Pose
{
  float x, y, z;      // position (meters)
  float vx, vy, vz;   // velocity (meters/sec)
  float yaw;          // heading (radians)
};

/**
 * Mutex guarded pose
 */
class SharedPose
{
  public:
    /**
     * Get pose
     */
    Pose get () const
    {
      std::lock_guard<std::mutex> lock (mutex_);
      return pose_;
    }

    /**
     * Set position
     */
    void set_position (float x, float y, float z)
    {
      std::lock_guard<std::mutex> lock (mutex_);
      pose_.x = x;
      pose_.y = y;
      pose_.z = z;
    }

    /**
     * Set velocity
     */
    void set_velocity (float vx, float vy, float vz)
    {
      std::lock_guard<std::mutex> lock (mutex_);
      pose_.vx = vx;
      pose_.vy = vy;
      pose_.vz = vz;
    }

    void set_yaw (float yaw)
    {
      std::lock_guard<std::mutex> lock (mutex_);
      pose_.yaw = yaw;
    }

  private:
    Pose pose_;
    mutable std::mutex mutex_;
};
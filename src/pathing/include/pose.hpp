#pragma once
#include <mutex>
#include <cmath>

/**
 * Holds basic 3D spatial data
 */
class Pose
{
  public:
    float x, y, z;      // position (meters)
    float vx, vy, vz;   // velocity (meters/sec)
    float yaw;          // heading (radians)

    /**
     * Returns Euclidian distance between two vec3s
     */
    static float dist3 (float x1, float y1, float z1, 
                        float x2, float y2, float z2)
    {
      float dx = x2 - x1;
      float dy = y2 - y1;
      float dz = z2 - z1;

      return std::sqrt (std::pow (dx, 2) + std::pow (dy, 2) + std::pow (dz, 2));
    }

    /**
     * Returns true if two poses are equal
     */
    bool equals (const Pose& other) const { return approx (other, 0, 0, 0); }

    /**
     * Returns true if two poses are equal within tolerance
     * 
     */
    bool approx (const Pose& other, float pos_tol, float vel_tol, float yaw_tol) const
    {
      if (dist3 (x, y, z, other.x, other.y, other.z) > pos_tol)
        return false;
      
      if (dist3 (vx, vy, vz, other.vx, other.vy, other.vz) > vel_tol)
        return false;
      
      if (std::abs (yaw - other.yaw) > yaw_tol)
        return false;
      
      return true;
    }
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

    /**
     * Set yaw
     */
    void set_yaw (float yaw)
    {
      std::lock_guard<std::mutex> lock (mutex_);
      pose_.yaw = yaw;
    }

  private:
    Pose pose_;
    mutable std::mutex mutex_;
};
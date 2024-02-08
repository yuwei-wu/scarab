#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>



/// @brief for 2d trajectory generation
class Trajectory
{
  using Vec2f = Eigen::Vector2f;
  using vec_Vec2f = std::vector<Vec2f, Eigen::aligned_allocator<Vec2f>>;

private:
  std::vector<Eigen::MatrixX2f, Eigen::aligned_allocator<Eigen::MatrixX2f>> coefficients_;
  std::vector<float> waypoint_times_;
public:

  Trajectory() {}
  Trajectory(std::vector<Eigen::MatrixX2f, Eigen::aligned_allocator<Eigen::MatrixX2f>> coefficients, 
             std::vector<float> waypoint_times)
  {

    coefficients_ = coefficients;
    waypoint_times_ = waypoint_times;
  }
  ~Trajectory() {}

  bool empty() const {
    return coefficients_.size() <=0  || waypoint_times_.size() <=0;
  }
  void clear() { coefficients_.clear(); waypoint_times_.clear(); }
  bool getCommand(const float time, Vec2f &pos, Vec2f &vel, Vec2f &acc, Vec2f &jrk) const;
  bool getPosition(const float time, Vec2f &pos) const;
  bool getVelocity(const float time, Vec2f &vel) const;
  bool getAcceleration(const float time, Vec2f &acc) const;
  float getTotalTime() const;
  int getIndex(const float time) const;

};



class TrajectoryGenerator
{
 public:
  using Vec2f = Eigen::Vector2f;
  using vec_Vec2f = std::vector<Vec2f, Eigen::aligned_allocator<Vec2f>>;

  /**
   * @brief
   *
   * @param continuous_derivative_order The highest derivative that is continous
   * @param minimize_derivative The derivative to minimize
   */
  TrajectoryGenerator(unsigned int continuous_derivative_order, unsigned int minimize_derivative);

  void setInitialConditions(const Vec2f &position, const vec_Vec2f &derivatives);
  void addWaypoint(const Vec2f &position);  // Waypoint is X, Y
  void addWaypoints(const vec_Vec2f &waypoints) { waypoints_ = waypoints; }
  void clearWaypoints(void);
  std::vector<float> computeTimesTrapezoidSpeed(float vel_des, float acc_des) const;
  std::vector<float> computeTimesConstantSpeed(float avg_speed) const;
  bool calculate(const std::vector<float> &waypoint_times_);

  void calcMaxPerSegment(std::vector<float> &max_vel, std::vector<float> &max_acc, std::vector<float> &max_jrk) const;

  Trajectory optimizeWaypointTimes(const float max_vel, const float max_acc, const float max_jrk);

  const std::vector<float> &getWaypointTimes() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const unsigned int N_;
  const unsigned int R_;
  vec_Vec2f waypoints_, initial_derivatives_;
  std::vector<Eigen::MatrixX2f, Eigen::aligned_allocator<Eigen::MatrixX2f>> coefficients_;
  std::vector<float> waypoint_times_;
};

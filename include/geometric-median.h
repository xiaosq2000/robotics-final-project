
#ifndef _GEOMETRIC_MEDIAN_H_
#define _GEOMETRIC_MEDIAN_H_

#include "Eigen/Dense"

double DistSum(Eigen::Vector2d p, const std::vector<Eigen::Vector2d> &points);
bool ForbiddenZoneCheck(const Eigen::Vector2d p, const std::vector<Eigen::Vector2d> &points, const double &forbidden_zone_radius);
void GeometricMedian(const std::vector<Eigen::Vector2d> &points, Eigen::Vector2d &center, double &min_dist, const double &forbidden_zone_radius, const double &lower_limit = 0.001);

#endif

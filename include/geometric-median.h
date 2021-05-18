
#ifndef _GEOMETRIC_MEDIAN_H_
#define _GEOMETRIC_MEDIAN_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

double distSum(Vector2d p, const vector<Vector2d> &points);
bool forbiddenZoneCheck(const Vector2d p, const vector<Vector2d> &points, const double &forbidden_zone_radius);
void geometricMedian(const vector<Vector2d> &points, Vector2d &center, double &min_dist, const double &lower_limit, const double &forbidden_zone_radius);

#endif

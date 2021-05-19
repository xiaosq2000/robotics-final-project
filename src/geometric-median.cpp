/**
 * @file geometric_median.cpp
 * @author 肖书奇
 * @brief 利用数值方法计算2D点集的几何中值
 * @version 0.1
 * @date 2021-05-17
 * 
 */

#include "geometric-median.h"
#include "brick.h"

/**
 * @brief Calculate the sum of Euclidean distances between a point and a set of points. 
 * 
 * @param p - a certain point
 * @param points - a set of points 
 * @return double - the sum of Euclidean distances
 */
double DistSum(Vector2d p, const vector<Vector2d> &points)
{
    double sum = 0;
    vector<Vector2d>::const_iterator it;
    for (it = points.begin(); it != points.end(); ++it)
    {
        sum += (p - *it).norm();
    }
    return sum;
}

/**
 * @brief Check whether the Euclidean distance between a point and each point of a point set is too close. 
 * 
 * @param p - a certain point
 * @param points - a set of points 
 * @param forbidden_zone_radius - threshold for closeness
 * @return true - NOT too close
 * @return false - too close
 */
bool ForbiddenZoneCheck(const Vector2d p, const vector<Vector2d> &points, const double &forbidden_zone_radius)
{
    bool flag = 1;
    vector<Vector2d>::const_iterator it;
    for (it = points.begin(); it != points.end(); ++it)
    {
        if ((p - *it).norm() < forbidden_zone_radius)
        {
            flag = 0;
        }
    }
    return flag;
}

/**
 * @brief Get the geometric median of a set of 2D points
 * 
 * @param points - the point set as input.
 * @param center - the geometric median as output.
 * @param min_dist - the sum of Euclidean distances between a certain point and a set of points as output. 
 * @param lower_limit - this value affects accuracy, and 0.01 is recommended.
 * @param forbidden_zone_radius - this value helps to exclude candidate center points that are too close to some points in the set.
 */
void GeometricMedian(const vector<Vector2d> &points, Vector2d &center, double &min_dist, const double &lower_limit, const double &forbidden_zone_radius)
{
    // Choose the center of gravity of equal discrete mass distribution as the initial point.
    Vector2d current_point;
    vector<Vector2d>::const_iterator it;
    for (it = points.begin(); it != points.end(); ++it)
    {
        current_point += *it;
    }
    current_point /= points.size();
    while (!ForbiddenZoneCheck(current_point, points, forbidden_zone_radius))
    {
        current_point = 2 * current_point + Vector2d(1, 1);
    }
    min_dist = DistSum(current_point, points);

    // Assume test_distance to be 1000
    double test_distance = 1000;
    bool flag = 0;

    while (test_distance > lower_limit)
    {
        flag = 0;

        // Traversing over all 4 neighbours
        for (int i = 0; i < 4; i++)
        {
            Vector2d new_point;
            Vector2d directions[4] = {Vector2d(-1, 0), Vector2d(1, 0), Vector2d(0, -1), Vector2d(0, 1)};
            new_point = current_point + (double)test_distance * directions[i];
            new_point = current_point + (double)test_distance * directions[i];

            double new_dist = DistSum(new_point, points);

            if (new_dist < min_dist && ForbiddenZoneCheck(new_point, points, forbidden_zone_radius))
            {
                min_dist = new_dist;
                current_point = new_point;
                current_point = new_point;
                flag = 1;
                break;
            }
        }

        if (flag == 0)
        {
            test_distance /= 2;
        }
    }
    center = current_point;
}

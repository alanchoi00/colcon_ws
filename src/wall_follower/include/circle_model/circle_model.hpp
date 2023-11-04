#ifndef CIRCLE_MODEL_HPP_
#define CIRCLE_MODEL_HPP_

#include <vector>
#include <algorithm>
#include <experimental/algorithm>
#include <random>
#include <cmath>
#include <limits>
#include <iterator>

struct Point2D {
  double distance;
  double angle;
};

struct CartesianPoint {
  CartesianPoint() = default;
  CartesianPoint(double a, double b) : x{a}, y{b} {};
  double x, y;
};

class CircleModel : public CartesianPoint {
public:
  // constructors
  CircleModel() = default;
  CircleModel(CartesianPoint center, double radius) : center_{center.x, center.y}, radius_{radius} {};

  CircleModel circleDetection(const std::vector<CartesianPoint>& points, int iterations, int sampleSize, double inlinerTolerance);

private:
  CartesianPoint convertToCartesian(const Point2D& polarPoint);
  bool isPointAnInlier(const CartesianPoint& point, const CircleModel& model, double tolerance);
  CircleModel fitCircleModelToPoints(const std::vector<CartesianPoint>& points);

  CartesianPoint center_{};
  double radius_{};
};

#endif
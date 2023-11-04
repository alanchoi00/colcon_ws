#include "circle_model/circle_model.hpp"
#include <algorithm>
#include <random>
#include <cmath>
#include <iterator>
#include <stdexcept>

CartesianPoint CircleModel::convertToCartesian(const Point2D& polarPoint) {
  CartesianPoint cartesian;
  cartesian.x = polarPoint.distance * cos(polarPoint.angle);
  cartesian.y = polarPoint.distance * sin(polarPoint.angle);
  return cartesian;
}

// Manual implementation of Pratt's method
CircleModel CircleModel::fitCircleModelToPoints(const std::vector<CartesianPoint>& points) {
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0, sum_x3 = 0, sum_y3 = 0;
    double sum_xy = 0, sum_x1y2 = 0, sum_x2y1 = 0;

    for (const auto& p : points) {
        double x = p.x;
        double y = p.y;
        double x2 = x * x;
        double y2 = y * y;

        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C = points.size() * sum_x2 - sum_x * sum_x;
    double D = points.size() * sum_xy - sum_x * sum_y;
    double E = points.size() * sum_x3 + points.size() * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    double G = points.size() * sum_y2 - sum_y * sum_y;
    double H = points.size() * sum_x2y1 + points.size() * sum_y3 - (sum_x2 + sum_y2) * sum_y;

    double denominator = C * G - D * D;
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Denominator in Pratt's method is zero.");
    }

    double a = (E * G - H * D) / denominator;
    double b = (C * H - E * D) / denominator;
    double c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / points.size();

    double centerX = a / (-2);
    double centerY = b / (-2);
    double radius = std::sqrt(a * a + b * b - 4 * c) / 2;

    return CircleModel{ { centerX, centerY }, radius };
}

bool CircleModel::isPointAnInlier(
  const CartesianPoint& point, const CircleModel& model, double tolerance
) {
  double distanceToCenter = std::hypot(point.x - model.center_.x, point.y - model.center_.y);
  return std::abs(distanceToCenter - model.radius_) <= tolerance;
}

// Custom sample function for C++14
template<typename InputIt, typename OutputIt, typename RNG>
OutputIt sample(InputIt first, InputIt last, OutputIt out, size_t n, RNG&& rng) {
  size_t remaining = std::distance(first, last);
  size_t m = std::min(n, remaining);

  while (m > 0) {
    if (static_cast<size_t>(std::uniform_int_distribution<>(0, --remaining)(rng)) < m) {
      *out++ = *first;
      --m;
    }
    ++first;
  }
  return out;
}

CircleModel CircleModel::circleDetection(
  const std::vector<CartesianPoint>& points,
  int iterations, int sampleSize, double inlierTolerance
) {
  int bestInlierCount = 0;
  CircleModel bestModel;

  std::random_device rd;
  std::mt19937 rng(rd());

  for (int i = 0; i < iterations; ++i) {
    std::vector<CartesianPoint> randomSample;
    sample(points.begin(), points.end(), std::back_inserter(randomSample), sampleSize, rng);

    CircleModel model = fitCircleModelToPoints(randomSample);

    int inlierCount = 0;
    for (auto it = points.begin(); it != points.end(); ++it) {
      if (isPointAnInlier(*it, model, inlierTolerance)) {
        inlierCount++;
        // Early termination if the best count can't be exceeded
        if (points.size() - (std::distance(it, points.end()) + inlierCount) < static_cast<long unsigned int>(bestInlierCount))
          break;
      }
    }

    if (inlierCount > bestInlierCount) {
      bestInlierCount = inlierCount;
      bestModel = model;
    }
  }

  return bestModel;
}
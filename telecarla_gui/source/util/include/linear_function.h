#ifndef TELECARLA_GUI_LINEAR_FUNCTION_H
#define TELECARLA_GUI_LINEAR_FUNCTION_H

#include <utility>

namespace lmt::util
{
class LinearFunction
{
  public:
    constexpr LinearFunction(double gradient, double yIntercept) noexcept : gradient_(gradient), yIntercept_(yIntercept)
    {
    }

    constexpr LinearFunction(const std::pair<double, double>& pointA, const std::pair<double, double>& pointB) noexcept
        : gradient_((pointB.second - pointA.second) / (pointB.first - pointA.first)),
          yIntercept_(pointA.second - gradient_ * pointA.first)
    {
    }

    constexpr double operator()(double value) const noexcept { return gradient_ * value + yIntercept_; }

  private:
    double gradient_{0.0};
    double yIntercept_{0.0};
};
}  // namespace lmt::util

#endif  // TELECARLA_GUI_LINEAR_FUNCTION_H

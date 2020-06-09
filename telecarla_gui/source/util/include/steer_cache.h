#ifndef TELECARLA_GUI_STEER_CACHE_H
#define TELECARLA_GUI_STEER_CACHE_H

namespace lmt::util
{
class SteerCache
{
  public:
    void left() noexcept;
    void right() noexcept;
    void set(double value) noexcept;
    void reset() noexcept;

    [[nodiscard]] double get() const noexcept;

  private:
    static double normalize(double value) noexcept;

  private:
    double cache_{0.0};

    static constexpr double maxSteerCache_{0.7};
    static constexpr double minSteerCache_{-1 * maxSteerCache_};
    static constexpr double steerIncrement_{0.005};
};
}  // namespace lmt::util

#endif  // TELECARLA_GUI_STEER_CACHE_H

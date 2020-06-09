#include "steer_cache.h"

#include <algorithm>

using namespace lmt::util;

void SteerCache::left() noexcept
{
    set(cache_ - steerIncrement_);
}

void SteerCache::right() noexcept
{
    set(cache_ + steerIncrement_);
}

void SteerCache::reset() noexcept
{
    cache_ = 0;
}

double SteerCache::get() const noexcept
{
    return cache_;
}

double SteerCache::normalize(double value) noexcept
{
    return std::min(maxSteerCache_, std::max(minSteerCache_, value));
}

void SteerCache::set(double value) noexcept
{
    cache_ = normalize(value);
}
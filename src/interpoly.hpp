#pragma once

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

using XY = std::pair<int16_t, int16_t>;
using Direction = std::pair<int16_t XY::*, int16_t XY::*>;
inline static const Direction F2S{&XY::first, &XY::second}, S2F{&XY::second, &XY::first};
using PolyXY = std::vector<XY>;
using Order = std::function<bool(int16_t, int16_t)>;
inline static const Order ASC{std::less<int16_t>()}, DESC{std::greater<int16_t>()};

auto lerpi(int16_t x, const XY &a, const XY &b, const Direction &direction) {
  const auto &[from, to] = direction;
  return static_cast<int16_t>( //
      static_cast<int32_t>(x - a.*from) * (b.*to - a.*to) / (b.*from - a.*from) + a.*to);
}

auto interpoly(int16_t x, const PolyXY &poly, const Direction &direction, const Order &order) {
  if (poly.empty())
    return x;

  const auto &[from, to] = direction;
  const auto &begin = poly.cbegin();
  auto it = begin;
  for (; it != poly.cend(); ++it) {
    if ((it != begin) && !order(*(it - 1).*from, *it.*from))
      break; // discountinuity
    if ((it == begin) || order(*it.*from, x))
      continue;
    return lerpi(x, *it, *(it - 1), direction);
  }
  return it == (begin + 1) ? *begin.*to : lerpi(x, *(it - 2), *(it - 1), direction);
}

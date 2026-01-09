#pragma once

#include <Preferences.h>

#include "interpoly.hpp"

void loadPoly(PolyXY &poly, const char *key, Preferences &preferences) {
  if (!preferences.isKey(key)) {
    poly.clear();
    return;
  }
  poly.resize(preferences.getBytesLength(key) / sizeof(PolyXY::value_type));
  preferences.getBytes(key, poly.data(), poly.size() * sizeof(PolyXY::value_type));
}

bool storePoly(const PolyXY &poly, const char *key, Preferences &preferences) {
  const auto len = poly.size() * sizeof(PolyXY::value_type);
  return preferences.putBytes(key, poly.data(), len) == len;
}

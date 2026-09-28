/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
      : std::true_type                                                                                                 \
  {                                                                                                                    \
  };

DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
DEFINE_MEMBER_CHECKER(feature)
DEFINE_MEMBER_CHECKER(return_type)
DEFINE_MEMBER_CHECKER(channel)
DEFINE_MEMBER_CHECKER(azimuth)
DEFINE_MEMBER_CHECKER(elevation)
DEFINE_MEMBER_CHECKER(distance)
DEFINE_MEMBER_CHECKER(time)

#define RS_HAS_MEMBER(C, member) has_##member<C>::value

namespace detail
{
template <typename T_Point>
inline void updateSphericalFields(T_Point& point)
{
#if __cplusplus >= 201703L
  if constexpr (RS_HAS_MEMBER(T_Point, x) && RS_HAS_MEMBER(T_Point, y) && RS_HAS_MEMBER(T_Point, z) &&
                (RS_HAS_MEMBER(T_Point, distance) || RS_HAS_MEMBER(T_Point, azimuth) || RS_HAS_MEMBER(T_Point, elevation)))
  {
    const float planar = std::sqrt(point.x * point.x + point.y * point.y);
    if constexpr (RS_HAS_MEMBER(T_Point, distance))
    {
      point.distance = std::sqrt(planar * planar + point.z * point.z);
    }
    if constexpr (RS_HAS_MEMBER(T_Point, azimuth))
    {
      point.azimuth = std::atan2(point.y, point.x);
    }
    if constexpr (RS_HAS_MEMBER(T_Point, elevation))
    {
      point.elevation = std::atan2(point.z, planar);
    }
  }
#endif
}
}  // namespace detail

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
  point.z = value;
  detail::updateSphericalFields(point);
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
  point.ring = value;
#if __cplusplus >= 201703L
  if constexpr (RS_HAS_MEMBER(T_Point, channel))
  {
    point.channel = value;
  }
  if constexpr (RS_HAS_MEMBER(T_Point, return_type))
  {
    point.return_type = 0;
  }
#endif
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                     const double& value)
{
  point.timestamp = value;
#if __cplusplus >= 201703L
  if constexpr (RS_HAS_MEMBER(T_Point, time))
  {
    double fractional = value - std::floor(value);
    if (fractional < 0.0)
    {
      fractional += 1.0;
    }
    uint64_t time_ns = static_cast<uint64_t>(std::llround(fractional * 1e9));
    if (time_ns > std::numeric_limits<uint32_t>::max())
    {
      time_ns = std::numeric_limits<uint32_t>::max();
    }
    point.time = static_cast<uint32_t>(time_ns);
  }
#endif
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, feature)>::type setFeature(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, feature)>::type setFeature(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.feature = value;
}

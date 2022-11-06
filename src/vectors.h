#ifndef VECTORS_H_
#define VECTORS_H_

#include <math.h>
#include <vector>

#define MATH_PI_TWO             (6.28318530717958647693) // MATH_PI * 2
#define MATH_PI_ONE_AND_A_HALF  (4.71238898038468985769) // MATH_PI * 1.5
#define MATH_PI                 (3.14159265358979323846) // MATH_PI * 1
#define MATH_PI_SEVEN_EIGHTHS   (2.74889357189106908365) // MATH_PI * 7/8
#define MATH_PI_THREE_QUARTERS  (2.35619449019234492885) // MATH_PI * 3/4
#define MATH_PI_FIVE_EIGHTHS    (1.96349540849362077404) // MATH_PI * 5/8
#define MATH_PI_HALF            (1.57079632679489661923) // MATH_PI * 1/2
#define MATH_PI_THREE_EIGHTHS   (1.17809724509617246442) // MATH_PI * 3/8
#define MATH_PI_QUARTER         (0.78539816339744830962) // MATH_PI * 1/4
#define MATH_PI_EIGHTH          (0.39269908169872415481) // MATH_PI * 1/8

#define RADS_TO_DEGS(_rad) ((_rad) * (180.0 / MATH_PI))
#define DEGS_TO_RADS(_deg) ((_deg) * (MATH_PI / 180.0))

namespace math {

typedef double radians_t;

enum class eDirection
{
  None = 0x00,
  Left = 0x01,
  Up = 0x02,
  Right = 0x04,
  Down = 0x08,
  UpLeft = Up | Left,
  LeftUp = UpLeft,
  UpRight = Up | Right,
  RightUp = UpRight,
  DownRight = Down | Right,
  RightDown = DownRight,
  DownLeft = Down | Left,
  LeftDown = DownLeft,
};

template <typename T>
class Vec2
{
public:
  Vec2<T>() : x(0), y(0) {}
  Vec2<T>(const Vec2& v) : x(v.x), y(v.y) {}
  Vec2<T>(T _x, T _y) : x(x), y(y) {}
  Vec2<T>(T _v) : x(_v), y(_v) {}

  Vec2<T> operator-() const { return Vec2<T>(x * -1.0, y * -1.0); }
  Vec2<T> operator-(const Vec2<T>& _v) const { return Vec2(x - _v.x, y - _v.y); }
  Vec2<T> operator-=(const Vec2<T>& _v) { x -= _v.x; y -= _v.y; return *this; }
  Vec2<T> operator+(const Vec2<T>& _v) const { return Vec2(x + _v.x, y + _v.y); }
  Vec2<T> operator+=(const Vec2<T>& _v) { x += _v.x; y += _v.y; return *this; }
  Vec2<T> operator*(T _scalar) const { return Vec2(x * _scalar, y * _scalar); }
  Vec2<T> operator*(const Vec2<T>& _v) const { return Vec2(x * _v.x, y * _v.y); }
  Vec2<T> operator/(T _denom) const { return Vec2(x / _denom, y / _denom); }
  Vec2<T> operator/(const Vec2<T>& _v) const { return Vec2(x / _v.x, y / _v.y); }
  bool operator==(const Vec2<T>& _v) const { return x == _v.x && y == _v.y; }
  bool operator!=(const Vec2<T>& _v) const { return x != _v.x || y != _v.y; }

  T length() const { return fabs(x) + fabs(y); }

  static Vec2<T> cast(const Vec2<T>& _from, const Vec2<T>& _towards, T _distance) {
    T vecDist = distanceBetween<T>(_from, _towards);
    T scale = _distance / (vecDist != 0 ? vecDist : 1.0);
    Vec2<T> pos = _towards - _from;
    pos = pos * scale;
    pos = pos + _from;
    return pos;
  }

  static Vec2<T> cast(const Vec2<T>& _from, radians_t _radians, T _distance) {
    Vec2<T> pos = _from;
    if (_radians >= -MATH_PI_EIGHTH && _radians <= MATH_PI_EIGHTH) {
      // UP = 0
      pos.y -= 1.0;
    }
    else if (_radians >= -MATH_PI_THREE_EIGHTHS && _radians <= -MATH_PI_EIGHTH) {
      // UP RIGHT = -0.25 pi
      pos.x += 1.0;
      pos.y -= 1.0;
    }
    else if (_radians >= -MATH_PI_FIVE_EIGHTHS && _radians <= -MATH_PI_THREE_EIGHTHS) {
      // RIGHT = -0.5 pi
      pos.x += 1.0;
    }
    else if (_radians >= -MATH_PI_SEVEN_EIGHTHS && _radians <= -MATH_PI_FIVE_EIGHTHS) {
      // DOWN RIGHT = -0.75 pi
      pos.x += 1.0;
      pos.y += 1.0;
    }
    else if ((_radians >= MATH_PI_SEVEN_EIGHTHS && _radians <= MATH_PI) || (_radians <= -MATH_PI_SEVEN_EIGHTHS && _radians >= -MATH_PI)) {
      // DOWN = pi
      pos.y += 1.0;
    }
    else if (_radians >= MATH_PI_FIVE_EIGHTHS && _radians <= MATH_PI_SEVEN_EIGHTHS) {
      // DOWN LEFT = 0.75 pi
      pos.x += -1.0;
      pos.y += 1.0;
    }
    else if (_radians >= MATH_PI_THREE_EIGHTHS && _radians <= MATH_PI_FIVE_EIGHTHS) {
      // LEFT = 0.5 = pi
      pos.x += -1.0;
    }
    else if (_radians >= MATH_PI_EIGHTH && _radians <= MATH_PI_THREE_EIGHTHS) {
      // UP LEFT = 0.25 pi
      pos.x += -1.0;
      pos.y -= 1.0;
    }
    return cast(_from, pos, _distance);
  }

  T x;
  T y;
};

template <typename T>
class Vec3
{
public:
  Vec3<T>() : x(0), y(0), z(0) {}
  Vec3<T>(const Vec3<T>& _v) : x(_v.x), y(_v.y), z(_v.z) {}
  Vec3<T>(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
  Vec3<T>(T _v) : x(_v), y(_v), z(_v) {}

  Vec3<T> operator-() const { return Vec3<T>(x * -1.0, y * -1.0, z * -1.0); }
  Vec3<T> operator-(const Vec3<T>& _v) const { return Vec3<T>(x - _v.x, y - _v.y, z - _v.z); }
  Vec3<T> operator-=(const Vec3<T>& _v) { x -= _v.x; y -= _v.y; z -= _v.z; return *this; }
  Vec3<T> operator+(const Vec3<T>& _v) const { return Vec3<T>(x + _v.x, y + _v.y, z + _v.z); }
  Vec3<T> operator+=(const Vec3<T>& _v) { x += _v.x; y += _v.y; z += _v.z; return *this; }
  Vec3<T> operator*(T _scalar) const { return Vec3<T>(x * _scalar, y * _scalar, z * _scalar); }
  Vec3<T> operator*(const Vec3<T>& _v) const { return Vec3<T>(x * _v.x, y * _v.y, z * _v.z); }
  Vec3<T> operator/(const Vec3<T>& _v) const { return Vec3<T>(x / _v.x, y / _v.y, z / _v.z); }
  Vec3<T> operator/(T _denom) const { return Vec3<T>(x / _denom, y / _denom, z / _denom); }
  bool operator==(const Vec3<T>& _v) const { return x == _v.x && y == _v.y && z == _v.z; }
  bool operator!=(const Vec3<T>& _v) const { return x != _v.x || y != _v.y || z != _v.z; }
  Vec3<T>& operator*=(T _scalar) { x *= _scalar; y *= _scalar; z *= _scalar; return *this; }
  Vec3<T>& operator*=(const Vec3<T>& _v) { x *= _v.x; y *= _v.y; z *= _v.z; return *this; }

  Vec2<T> xz() const { return Vec2<T>(x, z); }
  Vec2<T> xy() const { return Vec2<T>(x, y); }

  T normalize(T _unitLength = 1.0) {
    T d = sqrt((x * x) + (y * y) + (z * z));
    // a larger value causes normalizations errors in a scaled down model with camera extremely close
    if (d > 1.0e-35) {
      d = sqrtf(d);
      T invSqrt = unitLength / d;
      x = x * invSqrt;
      y = y * invSqrt;
      z = z * invSqrt;
    }
    else {
      x = y = z = 0;
      d = 0;
    }
    return d;
  }

  std::vector<Vec3<T>> getPerimeter() const {
    std::vector<Vec3<T>> vecs;
    vecs.push_back(Vec3<T>(x - 1.0, y, z)); // left
    vecs.push_back(Vec3<T>(x - 1.0, y, z - 1.0)); // top left
    vecs.push_back(Vec3<T>(x, y, z - 1.0)); // top
    vecs.push_back(Vec3<T>(x + 1.0, y, z - 1.0)); // top right
    vecs.push_back(Vec3<T>(x + 1.0, y, z)); // right
    vecs.push_back(Vec3<T>(x + 1.0, y, z + 1.0)); // down right
    vecs.push_back(Vec3<T>(x, y, z + 1.0)); // down
    vecs.push_back(Vec3<T>(x - 1.0, y, z + 1.0)); // down left
    return vecs;
  }

  T length() const { return fabs(x) + fabs(y) + fabs(z); }

  static Vec3<T> normalize(const Vec3<T>& _v) {
    Vec3<T> res = _v;
    res.normalize();
    return res;
  }

  static Vec3<T> cross(const Vec3& _v0, const Vec3& _v1) {
    return Vec3<T>(
      (v0.y * v1.z) - (v1.y * v0.z),
      (v0.z * v1.x) - (v1.z * v0.x),
      (v0.x * v1.y) - (v1.x * v0.y)
    );
  }

  static T dot(const Vec3<T>& _v0, const Vec3<T>& _v1) {
    Vec3<T> tmp = _v0 * _v1;
    return tmp.x + tmp.y + tmp.z;
  }

  T x;
  T y;
  T z;
};

template <typename T>
class Vec4
{
public:
  Vec4<T>() : x(0.f), y(0.f), z(0.f), w(0.f) {}
  Vec4<T>(const Vec4<T>& _v) : x(_v.x), y(_v.y), z(_v.z), w(_v.w) {}
  Vec4<T>(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w) {}
  Vec4<T>(T _v) : x(_v), y(_v), z(_v), w(_v) {}

  Vec4<T> operator-() const { return Vec4<T>(x * -1.0, y * -1.0, z * -1.0, w * -1.0); }
  Vec4<T> operator-(const Vec4<T>& _v) const { return Vec4<T>(x - _v.x, y - _v.y, z - _v.z, w - _v.w); }
  Vec4<T> operator-=(const Vec4<T>& _v) { x -= _v.x; y -= _v.y; z -= _v.z; w -= _v.w; return *this; }
  Vec4<T> operator+(const Vec4<T>& _v) const { return Vec4<T>(x + _v.x, y + _v.y, z + _v.z, w + _v.w); }
  Vec4<T> operator+=(const Vec4<T>& _v) { x += _v.x; y += _v.y; z += _v.z; w += _v.w; return *this; }
  Vec4<T> operator*(T _scalar) const { return Vec4<T>(x * _scalar, y * _scalar, z * _scalar, w * _scalar); }
  Vec4<T> operator*(const Vec4<T>& _v) const { return Vec4<T>(x * _v.x, y * _v.y, z * _v.z, w * _v.w); }
  Vec4<T> operator/(T _denom) const { return Vec4<T>(x / _denom, y / _denom, z / _denom, w / _denom); }
  Vec4<T> operator/(const Vec4<T>& _v) const { return Vec4<T>(x / _v.x, y / _v.y, z / _v.z, w / _v.w); }
  bool operator==(const Vec4<T>& _v) const { return x == _v.x && y == _v.y && z == _v.z && w == _v.w; }
  bool operator!=(const Vec4<T>& _v) const { return x != _v.x || y != _v.y || z != _v.z || w != _v.w; }

  Vec3<T> xyz() const { return Vec3<T>(x, y, z); }

  T x;
  T y;
  T z;
  T w;
};

template <typename T>
const char* findNextFloatValueFromString(const char* _str, T& _value) {
  const char* ptr = _str;
  // get past any whitespace (non digit character)
  while (*ptr != '\0' && !((*ptr >= '0' && *ptr <= '9') || *ptr == '.' || *ptr == '-')) ptr++;
  // parse the string until the end of the value
  const char* start = ptr;
  while (*ptr != '\0' && *ptr != ',' && *ptr != ' ') ptr++;
  _value = (T)atof(start);
  return ptr;
}

template <typename T>
void vec3FromString(Vec3<T>& _v, const char* _strValue) {
  if (!_strValue) return;

  const char* ptr = _strValue;
  ptr = findNextFloatValueFromString(ptr, _v.X);
  ptr = findNextFloatValueFromString(ptr, _v.Y);
  ptr = findNextFloatValueFromString(ptr, _v.Z);
}

template <typename T>
void vec4FromString(Vec4<T>& _v, const char* _strValue) {
  if (!_strValue) return;

  const char* ptr = _strValue;
  ptr = findNextFloatValueFromString(ptr, _v.X);
  ptr = findNextFloatValueFromString(ptr, _v.Y);
  ptr = findNextFloatValueFromString(ptr, _v.Z);
  ptr = findNextFloatValueFromString(ptr, _v.W);
}

radians_t directionToRadians(eDirection _direction) {
  radians_t rads = 0.f;
  switch (_direction) {
  case eDirection::Up:
    rads = 0;
    break;
  case eDirection::UpRight:
    rads = -MATH_PI_QUARTER;
    break;
  case eDirection::Right:
    rads = -MATH_PI_HALF;
    break;
  case eDirection::DownRight:
    rads = -MATH_PI_THREE_QUARTERS;
    break;
  case eDirection::Down:
    rads = MATH_PI;
    break;
  case eDirection::DownLeft:
    rads = MATH_PI_THREE_QUARTERS;
    break;
  case eDirection::Left:
    rads = MATH_PI_HALF;
    break;
  case eDirection::UpLeft:
    rads = MATH_PI_QUARTER;
    break;
  case eDirection::None:
    break;
  }
  return rads;
}

template <typename T>
Vec3<T> quatToEuler(const Vec4<T>& quat) {
  T sqw = q1.w * q1.w;
  T sqx = q1.x * q1.x;
  T sqy = q1.y * q1.y;
  T sqz = q1.z * q1.z;
  T unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
  T test = q1.x * q1.y + q1.z * q1.w;
  if (test > 0.499 * unit) { // singularity at north pole
    T heading = 2.0 * atan2(q1.X, q1.W);
    T attitude = MATH_PI_HALF;
    T bank = 0;
    return Vec3<T>(bank, heading, attitude);
  }
  if (test < -0.499 * unit) { // singularity at south pole
    T heading = -2.0 * atan2(q1.x, q1.w);
    T attitude = -MATH_PI_HALF;
    T bank = 0;
    return Vec3<T>(bank, heading, attitude);
  }
  T heading = atan2(2.0 * q1.y * q1.w - 2.0 * q1.x * q1.z, sqx - sqy - sqz + sqw);
  T attitude = asin(2.0 * test / unit);
  T bank = atan2(2.0 * q1.x * q1.w - 2.0 * q1.y * q1.z, -sqx + sqy - sqz + sqw);
  return Vec3<T>(bank, heading, attitude);
}

template <typename T>
Vec4<T> eulerToQuat(const Vec3<T>& _euler) {
  // Assuming the angles are in radians
  T heading = _euler.Y;
  T attitude = _euler.Z;
  T bank = _euler.X;
  T c1 = cos(heading / 2.0);
  T s1 = sin(heading / 2.0);
  T c2 = cos(attitude / 2.0);
  T s2 = sin(attitude / 2.0);
  T c3 = cos(bank / 2.0);
  T s3 = sin(bank / 2.0);
  T c1c2 = c1 * c2;
  T s1s2 = s1 * s2;
  Vec4<T> q;
  q.w = c1c2 * c3 - s1s2 * s3;
  q.x = c1c2 * s3 + s1s2 * c3;
  q.y = s1 * c2 * c3 + c1 * s2 * s3;
  q.z = c1 * s2 * c3 - s1 * c2 * s3;
  return q;
}

template <typename T>
T distanceBetween(const Vec3<T>& _from, const Vec3<T>& _to) {
  Vec3<T> dist = _to - _from;
  dist = dist * dist;
  return sqrt(dist.x + dist.y + dist.z);
}

template <typename T>
T distanceBetween(const Vec2<T>& _from, const Vec2<T>& _to) {
  Vec2<T> dist = _to - _from;
  dist = dist * dist;
  return sqrt(dist.x + dist.y);
}

typedef Vec2<int> Vec2i;
typedef Vec3<int> Vec3i;
typedef Vec4<int> Vec4i;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;

typedef Vec2<double> Vec2d;
typedef Vec3<double> Vec3d;
typedef Vec4<double> Vec4d;

} // namespace math

#endif // VECTORS_H_
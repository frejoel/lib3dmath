#ifndef MAT3_H_
#define MAT3_H_

#include "vectors.h"
#include <string.h>

namespace math {

/*
 * Row-major 3x3 Matrix.
 *
 * All logic in this class is following row-major.
 * In a column-major Matrix, this matrix can be conceptualized with the following indices:
 * [ 0, 3, 6 ]
 * [ 1, 4, 7 ]
 * [ 2, 5, 8 ]
 */
template <typename T>
class Mat3 {
public:
  enum {
    m11 = 0,
    m12 = 3,
    m13 = 6,
    m21 = 1,
    m22 = 4,
    m23 = 7,
    m31 = 2,
    m32 = 5,
    m33 = 8,
  };

  Mat3<T>() { setIdentity(); }
  Mat3<T>(const T _value) { for (int i = 0; i < 9; ++i) data[i] = _value; }
  Mat3<T>(const Mat3<T>& _copy) { memcpy(data, _copy.data, sizeof(float) * 9); }
  Mat3<T>(const T* _copy) { memcpy(data, _copy, sizeof(float) * 9); }

  Mat3<T>(const Vec3<T>& _axis, const radians_t _angle) {
    Vec3<T> normalizedAxis = _axis;
    if (normalizedAxis.normalize(1.f) == 0.f) {
      setIdentity();
    }
    else {
      T angleCos = cosf(_angle);
      T angleSin = sinf(_angle);
      T nsi[3], ico;
      T n00, n01, n11, n02, n12, n22;

      ico = (1.f - angleCos);
      nsi[0] = normalizedAxis.x * angleSin;
      nsi[1] = normalizedAxis.y * angleSin;
      nsi[2] = normalizedAxis.z * angleSin;

      n00 = (normalizedAxis.x * normalizedAxis.x) * ico;
      n01 = (normalizedAxis.x * normalizedAxis.y) * ico;
      n11 = (normalizedAxis.y * normalizedAxis.y) * ico;
      n02 = (normalizedAxis.x * normalizedAxis.z) * ico;
      n12 = (normalizedAxis.y * normalizedAxis.z) * ico;
      n22 = (normalizedAxis.z * normalizedAxis.z) * ico;

      data[m11] = n00 + angleCos;
      data[m12] = n01 + nsi[2];
      data[m13] = n02 - nsi[1];
      data[m21] = n01 - nsi[2];
      data[m22] = n11 + angleCos;
      data[m23] = n12 + nsi[0];
      data[m31] = n02 + nsi[1];
      data[m32] = n12 - nsi[0];
      data[m33] = n22 + angleCos;
    }
  }

  Mat3<T>(const char _axis, const radians_t _angle) {
    const T angleCos = cos(_angle);
    const T angleSin = sin(_angle);

    switch (_axis) {
    case 'x':
    case 'X': /* rotation around X */
      data[m11] = 1.0;
      data[m12] = 0.0;
      data[m13] = 0.0;
      data[m21] = 0.0;
      data[m22] = angleCos;
      data[m23] = angleSin;
      data[m31] = 0.0;
      data[m32] = -angleSin;
      data[m33] = angleCos;
      break;
    case 'y':
    case 'Y': /* rotation around Y */
      data[m11] = angleCos;
      data[m12] = 0.0;
      data[m13] = -angleSin;
      data[m21] = 0.0;
      data[m22] = 1.0;
      data[m23] = 0.0;
      data[m31] = angleSin;
      data[m32] = 0.0;
      data[m33] = angleCos;
      break;
    case 'z':
    case 'Z': /* rotation around Z */
      data[m11] = angleCos;
      data[m12] = angleSin;
      data[m13] = 0.0;
      data[m21] = -angleSin;
      data[m22] = angleCos;
      data[m23] = 0.0;
      data[m31] = 0.0;
      data[m32] = 0.0;
      data[m33] = 1.0;
      break;
    default:
      break;
    }
  }

  Mat3<T>(const Vec3<T>& _a, const Vec3<T>& _b, const Vec3<T>& _c) {
    data[m11] = _a.x;
    data[m12] = _a.y;
    data[m13] = _a.z;
    data[m21] = _b.x;
    data[m22] = _b.y;
    data[m23] = _b.z;
    data[m31] = _c.x;
    data[m32] = _c.y;
    data[m33] = _c.z;
  }

  Mat3<T>(const T _v0, const T _v1, const T _v2, const T _v3, const T _v4, const T _v5, const T _v6, const T _v7, const T _v8) {
    data[m11] = _v0;
    data[m12] = _v1;
    data[m13] = _v2;
    data[m21] = _v3;
    data[m22] = _v4;
    data[m23] = _v5;
    data[m31] = _v6;
    data[m32] = _v7;
    data[m33] = _v8;
  }

  void operator=(const Mat3<T>&) { memcpy(data, rhs.m_data, sizeof(T) * 9); }
  void operator=(const T*) { memcpy(data, rhs, sizeof(T) * 9); }
  
  Mat3<T> operator*(const Mat3<T>& _rhs) {
    Mat3<T> ret;
    ret.data[m11] = (data[m11] * _rhs.data[m11]) + (data[m12] * _rhs.data[m21]) + (data[m13] * _rhs.data[m31]);
    ret.data[m12] = (data[m11] * _rhs.data[m12]) + (data[m12] * _rhs.data[m22]) + (data[m13] * _rhs.data[m32]);
    ret.data[m13] = (data[m11] * _rhs.data[m13]) + (data[m12] * _rhs.data[m23]) + (data[m13] * _rhs.data[m33]);
    ret.data[m21] = (data[m21] * _rhs.data[m11]) + (data[m22] * _rhs.data[m21]) + (data[m23] * _rhs.data[m31]);
    ret.data[m22] = (data[m21] * _rhs.data[m12]) + (data[m22] * _rhs.data[m22]) + (data[m23] * _rhs.data[m32]);
    ret.data[m23] = (data[m21] * _rhs.data[m13]) + (data[m22] * _rhs.data[m23]) + (data[m23] * _rhs.data[m33]);
    ret.data[m31] = (data[m31] * _rhs.data[m11]) + (data[m32] * _rhs.data[m21]) + (data[m33] * _rhs.data[m31]);
    ret.data[m32] = (data[m31] * _rhs.data[m12]) + (data[m32] * _rhs.data[m22]) + (data[m33] * _rhs.data[m32]);
    ret.data[m33] = (data[m31] * _rhs.data[m13]) + (data[m32] * _rhs.data[m23]) + (data[m33] * _rhs.data[m33]);
    return ret;
  }

  Vec3<T> operator*(const Vec3<T>& _v) {
    Vec3 ret;
    ret.x = (data[m11] * _v.x) + (data[m12] * _v.y) + (data[m13] * _v.z);
    ret.y = (data[m21] * _v.x) + (data[m22] * _v.y) + (data[m23] * _v.z);
    ret.z = (data[m31] * _v.x) + (data[m32] * _v.y) + (data[m33] * _v.z);
    return ret;
  }

  Mat3<T> operator*(T _scalar) {
    for (int i = 0; i < 9; ++i) {
      res.m_data[i] = data[i] * _scalar;
    }
    return res;
  }

  Mat3<T> operator+(const Mat3<T>& _rhs) {
    Mat3<T> res;
    for (int i = 0; i < 9; ++i) {
      res.data[i] = data[i] + _rhs.data[i];
    }
    return res;
  }

  Mat3<T> operator-(const Mat3<T>& _rhs) {
    Mat3<T> res;
    for (int i = 0; i < 9; ++i) {
      res.data[i] = data[i] - _rhs.data[i];
    }
    return res;
  }

  void operator*=(const Mat3<T>& _m) {
    Mat3<T> res = *this * _m;
    memcpy(data, res.data, sizeof(T) * 9);
  }

  void operator*=(T _v) {
    Mat3<T> res = *this * _v;
    memcpy(data, res.data, sizeof(T) * 9);
  }

  bool operator==(const Mat3<T>& _rhs) {
    for (int i = 0; i < 9; ++i) {
      if (_rhs.data[i] != data[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const Mat3<T>& _rhs) {
    for (int i = 0; i < 9; ++i) {
      if (_rhs.data[i] != data[i]) {
        return true;
      }
    }
    return false;
  }

  T operator[](int _idx) const { return data[_idx]; }
  void set(int _idx, T _value) { data[_idx] = _value; }
  
  void setIdentity() {
    memset(data, 0, sizeof(T) * 9);
    data[m11] = 1.0;
    data[m22] = 1.0;
    data[m33] = 1.0;
  }

  void transpose() {
    // copy the original data so we can swap values around
    T tmp[9];
    memcpy(&tmp, data, sizeof(T) * 9);
    //m_data[m11] = tmp[m11];
    data[m12] = tmp[m21];
    data[m13] = tmp[m31];
    data[m21] = tmp[m12];
    //m_data[m22] = tmp[m22];
    data[m23] = tmp[m32];
    data[m31] = tmp[m13];
    data[m32] = tmp[m23];
    //m_data[m33] = tmp[m33];
  }

  void print() const { Mat3::print(data); }

  Vec3<T> getEuler() const {
    Vec3<T> eul1, eul2;
    const T cy = hypot(data[m11], data[m12]);
    if (cy > 16.0 * FLT_EPSILON) {
      eul1.x = atan2(data[m23], data[m33]);
      eul1.y = atan2(-data[m13], cy);
      eul1.z = atan2(data[m12], data[m11]);
      eul2.x = atan2(-data[m23], -data[m33]);
      eul2.y = atan2(-data[m13], -cy);
      eul2.z = atan2(-data[m12], -data[m11]);
    }
    else {
      eul1.x = atan2f(-data[m32], data[m22]);
      eul1.y = atan2f(-data[m13], cy);
      eul1.z = 0.0f;
      eul2 = eul1;
    }
    // return best, which is just the one with lowest values in it
    return eul1.length() > eul2.length() ? eul2 : eul1;
  }

  static Mat3<T> transpose(const Mat3<T>& _m) {
    Mat3<T> res = _m;
    res.transpose();
    return res;
  }

  static bool isEqual(const T* _m0, const T* _m1, T _delta) {
    for (int i = 0; i < 9; ++i) {
      if (_m0[i] < _m1[i] - _delta || _m0[i] > _m1[i] + _delta) {
        return false;
      }
    }
    return true;
  }

  static void print(const T* m_data) {
    printf("    %f, %f, %f,\n", m_data[m11], m_data[m12], m_data[m13]);
    printf("    %f, %f, %f,\n", m_data[m21], m_data[m22], m_data[m23]);
    printf("    %f, %f, %f,\n", m_data[m31], m_data[m32], m_data[m33]);
  }

  T data[9];
};

} // namespace joge

#endif // MAT3_H_
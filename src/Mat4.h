#ifndef MAT4_H_
#define MAT4_H_

#include "vertex.h"
#include "Mat3.h"

namespace math {

/*
 * Column-major 4x4 Matrix.
 *
 * All logic in this class is following column-major.
 * In a Row-major Matrix, this matrix can be conceptualized with the following indices:
 * [ 0, 4,  8, 12 ]
 * [ 1, 5,  9, 13 ]
 * [ 2, 6, 10, 14 ]
 * [ 3, 7, 11, 15 ]
 */
template <typename T>
class Mat4 {
public:
  enum {
    m11 = 0,
    m12 = 1,
    m13 = 2,
    m14 = 3,
    m21 = 4,
    m22 = 5,
    m23 = 6,
    m24 = 7,
    m31 = 8,
    m32 = 9,
    m33 = 10,
    m34 = 11,
    m41 = 12,
    m42 = 13,
    m43 = 14,
    m44 = 15,
  };

  Mat4<T>() { setIdentity(); }
  Mat4<T>(const T _value) { for (int i = 0; i < 16; ++i) data[i] = _value; }

  Mat4<T>(const Mat3<T>& _m3) {
    data[m11] = _m3[Mat3::m11];
    data[m12] = _m3[Mat3::m12];
    data[m13] = _m3[Mat3::m13];
    data[m14] = 0.0;
    data[m21] = _m3[Mat3::m21];
    data[m22] = _m3[Mat3::m22];
    data[m23] = _m3[Mat3::m23];
    data[m24] = 0.0;
    data[m31] = _m3[Mat3::m31];
    data[m32] = _m3[Mat3::m32];
    data[m33] = _m3[Mat3::m33];
    data[m34] = 0.0;
    data[m41] = 0.0;
    data[m42] = 0.0;
    data[m43] = 0.0;
    data[m44] = 1.0;
  }

  Mat4<T>(const Mat4<T>& _copy) { memcpy(data, _copy.data, sizeof(T) * 16); }
  Mat4<T>(const T* _copy) { memcpy(data, _copy, sizeof(T) * 16); }

  Mat4<T>(const Vec3<T>& _axis, const radians_t _angle) {
    Mat3<T> m3(_axis, _angle);
    data[m11] = m3[0];
    data[m12] = m3[1];
    data[m13] = m3[2];
    data[m14] = 1.0;
    data[m21] = m3[3];
    data[m22] = m3[4];
    data[m23] = m3[5];
    data[m24] = 1.0;
    data[m31] = m3[6];
    data[m32] = m3[7];
    data[m33] = m3[8];
    data[m34] = 1.0;
    data[m41] = 0.0;
    data[m42] = 0.0;
    data[m43] = 0.0;
    data[m44] = 1.0;
  }

  Mat4<T>(const char _axis, const radians_t _angle) {
    Mat3<T> m3(_axis, _angle);
    data[m11] = m3[Mat3::m11];
    data[m12] = m3[Mat3::m12];
    data[m13] = m3[Mat3::m13];
    data[m14] = 0.0;
    data[m21] = m3[Mat3::m21];
    data[m22] = m3[Mat3::m22];
    data[m23] = m3[Mat3::m23];
    data[m24] = 0.0;
    data[m31] = m3[Mat3::m31];
    data[m32] = m3[Mat3::m32];
    data[m33] = m3[Mat3::m33];
    data[m34] = 0.0;
    data[m41] = 0.0;
    data[m42] = 0.0;
    data[m43] = 0.0;
    data[m44] = 1.0;
  }

  Mat4<T>(const Vec4<T>& _quat) {
    setIdentity();
    T qxx = _quat.x * _quat.x;
    T qyy = _quat.y * _quat.y;
    T qzz = _quat.z * _quat.z;
    T qxz = _quat.x * _quat.z;
    T qxy = _quat.x * _quat.y;
    T qyz = _quat.y * _quat.z;
    T qwx = _quat.w * _quat.x;
    T qwy = _quat.w * _quat.y;
    T qwz = _quat.w * _quat.z;
    data[m11] = 1.0 - 2.0 * (qyy + qzz);
    data[m12] = 2.0 * (qxy + qwz);
    data[m13] = 2.0 * (qxz - qwy);
    data[m21] = 2.0 * (qxy - qwz);
    data[m22] = 1.0 - 2.0 * (qxx + qzz);
    data[m23] = 2.0 * (qyz + qwx);
    data[m31] = 2.0 * (qxz + qwy);
    data[m32] = 2.0 * (qyz - qwx);
    data[m33] = 1.0 - 2.0 * (qxx + qyy);
  }

  Mat4<T>(const Vec4<T>& _a, const Vec4<T>& _b, const Vec4<T>& _c, const Vec4<T>& _d) {
    data[m11] = _a.x;
    data[m12] = _a.y;
    data[m13] = _a.z;
    data[m14] = _a.w;
    data[m21] = _b.x;
    data[m22] = _b.y;
    data[m23] = _b.z;
    data[m24] = _b.w;
    data[m31] = _c.x;
    data[m32] = _c.y;
    data[m33] = _c.z;
    data[m34] = _c.w;
    data[m41] = _d.x;
    data[m42] = _d.y;
    data[m43] = _d.z;
    data[m44] = _d.w;
  }

  Mat4<T>(const T _v0, const T _v1, const T _v2, const T _v3, const T _v4, const T _v5, const T _v6, const T _v7, const T _v8, const T _v9, const T _v10, const T _v11, const T _v12, const T _v13, const T _v14, const T _v15) {
    data[m11] = _v0;
    data[m12] = _v1;
    data[m13] = _v2;
    data[m14] = _v3;
    data[m21] = _v4;
    data[m22] = _v5;
    data[m23] = _v6;
    data[m24] = _v7;
    data[m31] = _v8;
    data[m32] = _v9;
    data[m33] = _v10;
    data[m34] = _v11;
    data[m41] = _v12;
    data[m42] = _v13;
    data[m43] = _v14;
    data[m44] = _v15;
  }

  void operator=(const Mat4<T>& _rhs) { memcpy(data, _rhs.data, sizeof(T) * 16); }
  void operator=(const T* _rhs) { memcpy(data, _rhs, sizeof(T) * 16); }

  Mat4 operator*(const Mat4<T>& _rhs) const {
    Mat4<T> ret;
    ret.data[m11] = (data[m11] * _rhs.data[m11]) + (data[m21] * _rhs.data[m12]) + (data[m31] * _rhs.data[m13]) + (data[m41] * _rhs.data[m14]);
    ret.data[m12] = (data[m12] * _rhs.data[m11]) + (data[m22] * _rhs.data[m12]) + (data[m32] * _rhs.data[m13]) + (data[m42] * _rhs.data[m14]);
    ret.data[m13] = (data[m13] * _rhs.data[m11]) + (data[m23] * _rhs.data[m12]) + (data[m33] * _rhs.data[m13]) + (data[m43] * _rhs.data[m14]);
    ret.data[m14] = (data[m14] * _rhs.data[m11]) + (data[m24] * _rhs.data[m12]) + (data[m34] * _rhs.data[m13]) + (data[m44] * _rhs.data[m14]);
    ret.data[m21] = (data[m11] * _rhs.data[m21]) + (data[m21] * _rhs.data[m22]) + (data[m31] * _rhs.data[m23]) + (data[m41] * _rhs.data[m24]);
    ret.data[m22] = (data[m12] * _rhs.data[m21]) + (data[m22] * _rhs.data[m22]) + (data[m32] * _rhs.data[m23]) + (data[m42] * _rhs.data[m24]);
    ret.data[m23] = (data[m13] * _rhs.data[m21]) + (data[m23] * _rhs.data[m22]) + (data[m33] * _rhs.data[m23]) + (data[m43] * _rhs.data[m24]);
    ret.data[m24] = (data[m14] * _rhs.data[m21]) + (data[m24] * _rhs.data[m22]) + (data[m34] * _rhs.data[m23]) + (data[m44] * _rhs.data[m24]);
    ret.data[m31] = (data[m11] * _rhs.data[m31]) + (data[m21] * _rhs.data[m32]) + (data[m31] * _rhs.data[m33]) + (data[m41] * _rhs.data[m34]);
    ret.data[m32] = (data[m12] * _rhs.data[m31]) + (data[m22] * _rhs.data[m32]) + (data[m32] * _rhs.data[m33]) + (data[m42] * _rhs.data[m34]);
    ret.data[m33] = (data[m13] * _rhs.data[m31]) + (data[m23] * _rhs.data[m32]) + (data[m33] * _rhs.data[m33]) + (data[m43] * _rhs.data[m34]);
    ret.data[m34] = (data[m14] * _rhs.data[m31]) + (data[m24] * _rhs.data[m32]) + (data[m34] * _rhs.data[m33]) + (data[m44] * _rhs.data[m34]);
    ret.data[m41] = (data[m11] * _rhs.data[m41]) + (data[m21] * _rhs.data[m42]) + (data[m31] * _rhs.data[m43]) + (data[m41] * _rhs.data[m44]);
    ret.data[m42] = (data[m12] * _rhs.data[m41]) + (data[m22] * _rhs.data[m42]) + (data[m32] * _rhs.data[m43]) + (data[m42] * _rhs.data[m44]);
    ret.data[m43] = (data[m13] * _rhs.data[m41]) + (data[m23] * _rhs.data[m42]) + (data[m33] * _rhs.data[m43]) + (data[m43] * _rhs.data[m44]);
    ret.data[m44] = (data[m14] * _rhs.data[m41]) + (data[m24] * _rhs.data[m42]) + (data[m34] * _rhs.data[m43]) + (data[m44] * _rhs.data[m44]);
    return ret;
  }

  Vec3 operator*(const Vec3<T>& _v) const {
    Vec3<T> ret;
    ret.x = (data[m11] * _v.x) + (data[m21] * _v.y) + (data[m31] * _v.z) + data[m41];
    ret.y = (data[m12] * _v.x) + (data[m22] * _v.y) + (data[m32] * _v.z) + data[m42];
    ret.z = (data[m13] * _v.x) + (data[m23] * _v.y) + (data[m33] * _v.z) + data[m43];
    return ret;
  }

  Vec4 operator*(const Vec4<T>& _v) const {
    Vec4 ret;
    ret.x = (data[m11] * _v.x) + (data[m21] * _v.y) + (data[m31] * _v.z) + (data[m41] * _v.w);
    ret.y = (data[m12] * _v.x) + (data[m22] * _v.y) + (data[m32] * _v.z) + (data[m42] * _v.w);
    ret.z = (data[m13] * _v.x) + (data[m23] * _v.y) + (data[m33] * _v.z) + (data[m43] * _v.w);
    ret.w = (data[m14] * _v.x) + (data[m24] * _v.y) + (data[m34] * _v.z) + (data[m44] * _v.w);
    return ret;
  }

  Mat4 operator*(T _scalar) const {
    Mat4<T> res;
    for (int i = 0; i < 16; ++i) {
      res.data[i] = data[i] * _scalar;
    }
    return res;
  }

  Mat4 operator+(const Mat4<T>& _rhs) const {
    Mat4<T> res;
    for (int i = 0; i < 16; ++i) {
      res.data[i] = data[i] + _rhs.data[i];
    }
    return res;
  }

  Mat4 operator-(const Mat4<T>& _rhs) const {
    Mat4<T> res;
    for (int i = 0; i < 16; ++i) {
      res.data[i] = data[i] - _rhs.data[i];
    }
    return res;
  }

  void operator*=(const Mat4<T>& _m) {
    Mat4<T> res = *this * _m;
    memcpy(data, res.data, sizeof(T) * 16);
  }

  void operator*=(T _v) {
    Mat4<T> res = *this * _v;
    memcpy(data, res.data, sizeof(T) * 16);
  }

  bool operator==(const Mat4<T>& _rhs) const {
    for (int i = 0; i < 16; ++i) {
      if (_rhs.data[i] != data[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const Mat4<T>& rhs) const {
    for (int i = 0; i < 16; ++i) {
      if (_rhs.m_data[i] != data[i]) {
        return true;
      }
    }
    return false;
  }

  T operator[](int _idx) const { return data[_idx]; }
  void set(int _idx, T _value) { data[idx] = value; }

  void setIdentity() {
    memset(data, 0, sizeof(T) * 16);
    data[m11] = 1.0;
    data[m22] = 1.0;
    data[m33] = 1.0;
    data[m44] = 1.0;
  }

  void translate(const Vec3<T>& _v) {
    data[m41] = (data[m11] * _v.x) + (data[m21] * _v.y) + (data[m31] * _v.z) + data[m41];
    data[m42] = (data[m12] * _v.x) + (data[m22] * _v.y) + (data[m32] * _v.z) + data[m42];
    data[m43] = (data[m13] * _v.x) + (data[m23] * _v.y) + (data[m33] * _v.z) + data[m43];
    data[m44] = (data[m14] * _v.x) + (data[m24] * _v.y) + (data[m34] * _v.z) + data[m44];
    /*
    m_data[m14] = (m_data[m11] * v.x) + (m_data[m12] * v.y) + (m_data[m13] * v.z) + m_data[m14];
    m_data[m24] = (m_data[m21] * v.x) + (m_data[m22] * v.y) + (m_data[m23] * v.z) + m_data[m24];
    m_data[m34] = (m_data[m31] * v.x) + (m_data[m32] * v.y) + (m_data[m33] * v.z) + m_data[m34];
    m_data[m44] = (m_data[m41] * v.x) + (m_data[m42] * v.y) + (m_data[m43] * v.z) + m_data[m44];
    */
  }

  void rotate(radians_t _angle, const Vec3<T>& _v) {
    const T a = _angle;
    const T c = cos(a);
    const T s = sin(a);
    Vec3<T> axis = _v;
    axis.normalize();
    Vec3<T> tmp(axis * (1.0 - c));
    Mat4<T> rotate;
    rotate.data[m11] = c + tmp.x * axis.x;
    rotate.data[m12] = tmp.x * axis.y + s * axis.z;
    rotate.data[m13] = tmp.x * axis.z - s * axis.y;
    rotate.data[m21] = tmp.y * axis.x - s * axis.z;
    rotate.data[m22] = c + tmp.y * axis.y;
    rotate.data[m23] = tmp.y * axis.z + s * axis.x;
    rotate.data[m31] = tmp.z * axis.x + s * axis.y;
    rotate.data[m32] = tmp.z * axis.y - s * axis.x;
    rotate.data[m33] = c + tmp.z * axis.z;
    Vec4<T> col0(data[m11], data[m12], data[m13], data[m14]);
    Vec4<T> col1(data[m21], data[m22], data[m23], data[m24]);
    Vec4<T> col2(data[m31], data[m32], data[m33], data[m34]);
    Vec4<T> newCol0 = col0 * rotate.data[m11] + col1 * rotate.data[m12] + col2 * rotate.data[m13];
    Vec4<T> newCol1 = col0 * rotate.data[m21] + col1 * rotate.data[m22] + col2 * rotate.data[m23];
    Vec4<T> newCol2 = col0 * rotate.data[m31] + col1 * rotate.data[m32] + col2 * rotate.data[m33];
    data[m11] = newCol0.x;
    data[m12] = newCol0.y;
    data[m13] = newCol0.z;
    data[m14] = newCol0.w;
    data[m21] = newCol1.x;
    data[m22] = newCol1.y;
    data[m23] = newCol1.z;
    data[m24] = newCol1.w;
    data[m31] = newCol2.x;
    data[m32] = newCol2.y;
    data[m33] = newCol2.z;
    data[m34] = newCol2.w;
  }

  void rotate(const Vec3<T>& _rotation) {
    if (_rotation.x != 0.0) rotate(_rotation.x, Vec3<T>(1.0, 0.0, 0.0));
    if (_rotation.y != 0.0) rotate(_rotation.y, Vec3<T>(0.0, 1.0, 0.0));
    if (_rotation.z != 0.0) rotate(_rotation.z, Vec3<T>(0.0, 0.0, 1.0));
  }

  void scale(const Vec3<T>& _v) {
    data[m11] = data[m11] * _v.x;
    data[m12] = data[m12] * _v.x;
    data[m13] = data[m13] * _v.x;
    data[m14] = data[m14] * _v.x;
    data[m21] = data[m21] * _v.y;
    data[m22] = data[m22] * _v.y;
    data[m23] = data[m23] * _v.y;
    data[m24] = data[m24] * _v.y;
    data[m31] = data[m31] * _v.z;
    data[m32] = data[m32] * _v.z;
    data[m33] = data[m33] * _v.z;
    data[m34] = data[m34] * _v.z;
  }

  void transpose() {
    // copy the original data so we can swap values around
    T tmp[16];
    memcpy(&tmp, data, sizeof(T) * 16);
    //m_data[m11] = tmp[m11];
    data[m12] = tmp[m21];
    data[m13] = tmp[m31];
    data[m14] = tmp[m41];
    data[m21] = tmp[m12];
    //m_data[m22] = tmp[m22];
    data[m23] = tmp[m32];
    data[m24] = tmp[m42];
    data[m31] = tmp[m13];
    data[m32] = tmp[m23];
    //m_data[m33] = tmp[m33];
    data[m34] = tmp[m43];
    data[m41] = tmp[m14];
    data[m42] = tmp[m24];
    data[m43] = tmp[m34];
    //m_data[m44] = tmp[m44];
  }

  T determinant() const {
    return
      data[m11] * data[m22] * data[m33] * data[m44] - data[m11] * data[m22] * data[m34] * data[m43] + data[m11] * data[m23] * data[m34] * data[m42] - data[m11] * data[m23] * data[m32] * data[m44]
      + data[m11] * data[m24] * data[m32] * data[m43] - data[m11] * data[m24] * data[m33] * data[m42] - data[m12] * data[m23] * data[m34] * data[m41] + data[m12] * data[m23] * data[m31] * data[m44]
      - data[m12] * data[m24] * data[m31] * data[m43] + data[m12] * data[m24] * data[m33] * data[m41] - data[m12] * data[m21] * data[m33] * data[m44] + data[m12] * data[m21] * data[m34] * data[m43]
      + data[m13] * data[m24] * data[m31] * data[m42] - data[m13] * data[m24] * data[m32] * data[m41] + data[m13] * data[m21] * data[m32] * data[m44] - data[m13] * data[m21] * data[m34] * data[m42]
      + data[m13] * data[m22] * data[m34] * data[m41] - data[m13] * data[m22] * data[m31] * data[m44] - data[m14] * data[m21] * data[m32] * data[m43] + data[m14] * data[m21] * data[m33] * data[m42]
      - data[m14] * data[m22] * data[m33] * data[m41] + data[m14] * data[m22] * data[m31] * data[m43] - data[m14] * data[m23] * data[m31] * data[m42] + data[m14] * data[m23] * data[m32] * data[m41];
  }

  void inverse() {
    T coef00 = data[m33] * data[m44] - data[m43] * data[m34];
    T coef02 = data[m23] * data[m44] - data[m43] * data[m24];
    T coef03 = data[m23] * data[m34] - data[m33] * data[m24];
    T coef04 = data[m32] * data[m44] - data[m42] * data[m34];
    T coef06 = data[m22] * data[m44] - data[m42] * data[m24];
    T coef07 = data[m22] * data[m34] - data[m32] * data[m24];
    T coef08 = data[m32] * data[m43] - data[m42] * data[m33];
    T coef10 = data[m22] * data[m43] - data[m42] * data[m23];
    T coef11 = data[m22] * data[m33] - data[m32] * data[m23];
    T coef12 = data[m31] * data[m44] - data[m41] * data[m34];
    T coef14 = data[m21] * data[m44] - data[m41] * data[m24];
    T coef15 = data[m21] * data[m34] - data[m31] * data[m24];
    T coef16 = data[m31] * data[m43] - data[m41] * data[m33];
    T coef18 = data[m21] * data[m43] - data[m41] * data[m23];
    T coef19 = data[m21] * data[m33] - data[m31] * data[m23];
    T coef20 = data[m31] * data[m42] - data[m41] * data[m32];
    T coef22 = data[m21] * data[m42] - data[m41] * data[m22];
    T coef23 = data[m21] * data[m32] - data[m31] * data[m22];
    Vec4<T> fac0(coef00, coef00, coef02, coef03);
    Vec4<T> fac1(coef04, coef04, coef06, coef07);
    Vec4<T> fac2(coef08, coef08, coef10, coef11);
    Vec4<T> fac3(coef12, coef12, coef14, coef15);
    Vec4<T> fac4(coef16, coef16, coef18, coef19);
    Vec4<T> fac5(coef20, coef20, coef22, coef23);
    Vec4<T> vec0(data[m21], data[m11], data[m11], data[m11]);
    Vec4<T> vec1(data[m22], data[m12], data[m12], data[m12]);
    Vec4<T> vec2(data[m23], data[m13], data[m13], data[m13]);
    Vec4<T> vec3(data[m24], data[m14], data[m14], data[m14]);
    Vec4<T> inv0(vec1 * fac0 - vec2 * fac1 + vec3 * fac2);
    Vec4<T> inv1(vec0 * fac0 - vec2 * fac3 + vec3 * fac4);
    Vec4<T> inv2(vec0 * fac1 - vec1 * fac3 + vec3 * fac5);
    Vec4<T> inv3(vec0 * fac2 - vec1 * fac4 + vec2 * fac5);
    Vec4<T> signA(+1, -1, +1, -1);
    Vec4<T> signB(-1, +1, -1, +1);
    Mat4<T> inverse(inv0 * signA, inv1 * signB, inv2 * signA, inv3 * signB);
    Vec4<T> row0(inverse.data[m11], inverse.data[m21], inverse.data[m31], inverse.data[m41]);
    Vec4<T> m0(data[m11], data[m12], data[m13], data[m14]);
    Vec4<T> dot0(m0 * row0);
    T dot1 = (dot0.x + dot0.y) + (dot0.z + dot0.w);
    T oneOverDeterminant = 1.0 / dot1;
    Mat4 tmp = inverse * oneOverDeterminant;
    memcpy(data, tmp.data, sizeof(T) * 16);
  }

  void print() const { Mat4::print(data); }

  Mat3<T> getRotationMatrix() const {
    Mat3<T> m3;
    m3.set(Mat3::m11, data[m11]);
    m3.set(Mat3::m12, data[m12]);
    m3.set(Mat3::m13, data[m13]);
    m3.set(Mat3::m21, data[m21]);
    m3.set(Mat3::m22, data[m22]);
    m3.set(Mat3::m23, data[m23]);
    m3.set(Mat3::m31, data[m31]);
    m3.set(Mat3::m32, data[m32]);
    m3.set(Mat3::m33, data[m33]);
    return m3;
  }

  Vec3<T> getTranslation() const { return Vec3<T>(data[m41], data[m42], data[m43]); }

  static Mat4<T> inverse(const Mat4<T>& _m) {
    Mat4<T> res = _m;
    res.inverse();
    return res;
  }

  static Mat4<T> transpose(const Mat4<T>& _m) {
    Mat4 res = _m;
    res.transpose();
    return res;
  }

  static Mat4<T> translate(const Mat4<T>& _m, const Vec3<T>& _v) {
    Mat4 res = _m;
    res.translate(_v);
    return res;
  }

  static Mat4<T> rotate(const Mat4<T>& _m, radians_t _angle, const Vec3& _v) {
    Mat4 res = _m;
    res.rotate(_angle, _v);
    return res;
  }

  static Mat4<T> scale(const Mat4<T>& _m, const Vec3<T>& _v) {
    Mat4 res = _m;
    res.scale(_v);
    return res;
  }

  static Mat4<T> perspective(T _fovy, T _aspectRatio, T _zNear, T _zFar) {
    const T tanHalfFovy = tan(_fovy / 2.0);
    Mat4<T> res(0.0);
    res.data[m11] = 1.0 / (_aspectRatio * tanHalfFovy);
    res.data[m22] = 1.0 / (tanHalfFovy);
    res.data[m33] = -(_zFar + _zNear) / (_zFar - _zNear);
    res.data[m34] = -1.0;
    res.data[m43] = -(2.0 * _zFar * _zNear) / (_zFar - _zNear);
    return res;
  }

  static Mat4<T> lookAt(const Vec3<T>& _eye, const Vec3<T>& _center, const Vec3<T>& _up) {
    Vec3<T> f = _center - _eye;
    f.normalize();
    Vec3<T> s = Vec3::cross(f, _up);
    s.normalize();
    Vec3<T> u = Vec3::cross(s, f);
    Mat4<T> res;
    res.data[m11] = s.x;
    res.data[m21] = s.y;
    res.data[m31] = s.z;
    res.data[m12] = u.x;
    res.data[m22] = u.y;
    res.data[m32] = u.z;
    res.data[m13] = -f.x;
    res.data[m23] = -f.y;
    res.data[m33] = -f.z;
    res.data[m41] = -Vec3<T>::dot(s, _eye);
    res.data[m42] = -Vec3<T>::dot(u, _eye);
    res.data[m43] = Vec3<T>::dot(f, _eye);
    return res;
  }

  static Mat4<T> ortho(T _left, T _right, T _bottom, T _top, T _zNear, T _zFar) {
    Mat4<T> res;
    res.data[m11] = 2.0 / (_right - _left);
    res.data[m22] = 2.0 / (_top - _bottom);
    res.data[m33] = -2.0 / (_zFar - _zNear);
    res.data[m41] = -(_right + _left) / (_right - _left);
    res.data[m42] = -(_top + _bottom) / (_top - _bottom);
    res.data[m43] = -(_zFar + _zNear) / (_zFar - _zNear);
    return res;
  }

  static Mat4<T> fromQuat(const Vec4<T>& _quat) {
    T x = _quat.x;
    T y = _quat.y;
    T z = _quat.z;
    T w = _quat.w;
    T x2 = x + x;
    T y2 = y + y;
    T z2 = z + z;
    T xx = x * x2;
    T yx = y * x2;
    T yy = y * y2;
    T zx = z * x2;
    T zy = z * y2;
    T zz = z * z2;
    T wx = w * x2;
    T wy = w * y2;
    T wz = w * z2;
    Mat4<T> out;
    out.data[m11] = 1.0 - yy - zz;
    out.data[m12] = yx + wz;
    out.data[m13] = zx - wy;
    out.data[m14] = 0.0;
    out.data[m21] = yx - wz;
    out.data[m22] = 1.0 - xx - zz;
    out.data[m23] = zy + wx;
    out.data[m24] = 0.0;
    out.data[m31] = zx + wy;
    out.data[m32] = zy - wx;
    out.data[m33] = 1.0 - xx - yy;
    out.data[m34] = 0.-;
    out.data[m41] = 0.0;
    out.data[m42] = 0.0;
    out.data[m43] = 0.0;
    out.data[m44] = 1.0;
    return out;
  }

  static void print(const T* _data) {
    printf("    %f, %f, %f, %f,\n", _data[m11], _data[m12], _data[m13], _data[m14]);
    printf("    %f, %f, %f, %f,\n", _data[m21], _data[m22], _data[m23], _data[m24]);
    printf("    %f, %f, %f, %f,\n", _data[m31], _data[m32], _data[m33], _data[m34]);
    printf("    %f, %f, %f, %f,\n", _data[m41], _data[m42], _data[m43], _data[m44]);
  }

  static bool isEqual(const T* _m0, const T* _m1, T _delta) {
    for (int i = 0; i < 16; ++i) {
      if (_m0[i] < _m1[i] - _delta || _m0[i] > _m1[i] + _delta) {
        return false;
      }
    }
    return true;
  }

  float data[16];
};

typedef Mat4<double> Mat4d;
typedef Mat4<float> Mat4f;
typedef Mat4<int> Mat4i;

} // namespace math

#endif // MAT4_H_

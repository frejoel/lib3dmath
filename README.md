# lib3dmath
C++ header library containing useful 3D math for games.

## Integrating
Copy the `*.h` files from `./src/` into your project.

## Classes
All classes are placed within the `math` namespace.

### Vec2 (vertex.h)
Vertex with 2 dimensions (x,y)
* Vec2<float> = Vec2f
* Vec2<double> = Vec2d
* Vec2<int> = Vec2i

### Vec3 (vertex.h)
Vertex with 3 dimensions (x,y,z)
* Vec3<float> = Vec3f
* Vec3<double> = Vec3d
* Vec3<int> = Vec3i

### Vec4 (vertex.h)
Vertex with 4 dimensions (x,y,z,w)
* Vec4<float> = Vec4f
* Vec4<double> = Vec4d
* Vec4<int> = Vec4i

### Mat3 (Mat3.h)
3x3 Matrices ordered by column-major
* Mat3<float> = Mat3f
* Mat3<double> = Mat3d
* Mat3<int> = Mat3i

### Mat4 (Mat4.h)
4x4 Matrices ordered by column-major
* Mat4<float> = Mat4f
* Mat4<double> = Mat4d
* Mat4<int> = Mat4i

## Example usage

```
#include <stdio.h>
#include "vertex.h"
#include "Mat4.h"

using namespace math

Vec4<float> v4_1(0.1f, 0.2f, 0.3f, 0.4f);
Vec4<float> v4_2(0.9f, 0.8f, 0.7f, 0.6f);

Vec4f v4_3 = v1 * v2 + v1;
Vec3f v3 = v3.xyz();

v3.normalize();

Mat4<double> m;
m.transpose();

Mat4f p = Mat4f::perspective(0.3, 1.4, 0.1, 100);

if (p != m) printf("matrices are not the same\n");
```

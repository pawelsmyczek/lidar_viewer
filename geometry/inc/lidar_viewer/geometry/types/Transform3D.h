#ifndef LIDAR_VIEWER_TRANSFORM3D_H
#define LIDAR_VIEWER_TRANSFORM3D_H

#include "Static2DMatrix.h"
#include "Vector.h"

#include <algorithm>
#include <cmath>

namespace lidar_viewer::geometry::types
{

/// Rigid body transform in 3D: maps a point as `p' = R * p + t`, where `R` is a rotation matrix and
/// `t` a translation. It is the pose of a sensor: `pose(p)` takes a point from the sensor frame to
/// the world frame. Transforms are immutable values, every operation returns a new one.
/// @tparam T floating point coordinate type; use double for poses that are accumulated
template<class T>
class Transform3D
{
public:
    /// identity transform: no rotation, no translation
    Transform3D()
        : rotation_{decltype(rotation_)::identity()}
        , translation_{}
    { }

    /// pure translation by `tra`, without rotation
    explicit Transform3D(const Vector3D<T>& tra)
    : rotation_{decltype(rotation_)::identity()}
    , translation_{tra}
    { }

    /// @param rot rotation matrix, not validated (see isRotation())
    /// @param tra translation
    Transform3D(const Static2DMatrix<T, 3, 3>& rot, const Vector3D<T>& tra)
        : rotation_{rot}
        , translation_{tra}
    { }

    /// the rotation matrix `R`, a reference into the transform
    const Static2DMatrix<T,3,3>& rotation() const
    {
        return rotation_;
    }
    /// the translation `t`, a reference into the transform
    const Vector3D<T>& translation() const
    {
        return translation_;
    }
    /// True when the rotation part is a proper rotation: orthonormal (`R * Rᵀ == I`) with
    /// determinant +1, each element within `eps`. Products of many transforms drift, so use it to
    /// check a long chain. The translation does not matter.
    bool isRotation(T eps = T{1e-5}) const
    {
        const auto close = [eps](const T a, const T b) { return std::abs(a - b) <= eps; };

        const auto product = rotation_ * rotation_.transpose();
        const auto identity = Static2DMatrix<T, 3, 3>::identity();
        if (!std::equal(product.begin(), product.end(), identity.begin(), close))
        {
            return false;
        }

        const auto& r = rotation_;   // r(col, row)
        const T determinant = r(0, 0) * (r(1, 1) * r(2, 2) - r(2, 1) * r(1, 2))
                              - r(1, 0) * (r(0, 1) * r(2, 2) - r(2, 1) * r(0, 2))
                              + r(2, 0) * (r(0, 1) * r(1, 2) - r(1, 1) * r(0, 2));
        return close(determinant, T{1});
    }

    /// Composition: `(a * b)(p) == a(b(p))`, so `rhs` is applied first and this transform second.
    /// The result has rotation `Ra * Rb` and translation `Ra * tb + ta`. Not commutative.
    Transform3D operator*(const Transform3D& rhs) const
    {
        const auto Rp_plus_t = (*this)(rhs.translation_);
        return Transform3D{rotation_ * rhs.rotation_, Rp_plus_t};
    }

    /// Rotates a vector without translating it: `R * v`. Use it for directions and for differences
    /// of points, use operator()() for positions.
    Vector3D<T> rotate(Vector3D<T> v) const
    {
        return rotation_ * v;
    }

    /// Applies the transform to a position: `R * p + t`.
    /// @param p the position
    /// @return the transformed position, `p` is not modified
    Vector3D<T> operator()(const Vector3D<T>& p) const
    {
        auto rotated = rotate(p);
        return rotated + translation_;
    }

    /// The transform that undoes this one: rotation `Rᵀ` and translation `-Rᵀ * t`. Valid as long as
    /// `R` is a rotation (see isRotation()).
    Transform3D inverse() const
    {
        const Transform3D rt{rotation_.transpose(), Vector3D<T>{}};   // rotation only
        const auto t = rt.rotate(translation_);                       // Rᵀ·t
        return Transform3D{rt.rotation_, Vector3D<T>{{-t[0], -t[1], -t[2]}}};
    }

    /// identity transform, same as the default constructed one
    static Transform3D identity()
    {
        return Transform3D{};
    }
    /// transform that only translates by `t`
    static Transform3D fromTranslation(const Vector3D<T>& t)
    {
        return Transform3D{decltype(rotation_)::identity(), t};
    }

    /// Rotation by `angle` radians around `axis` (Rodrigues' formula), counterclockwise when looking
    /// against the axis, without translation.
    /// @param axis the axis, it need not be a unit vector; a zero axis gives the identity
    /// @param angle angle in radians
    static Transform3D fromAxisAngle(const Vector3D<T>& axis, T angle)
    {
        const T norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (norm == T{0})
        {
            return {};
        }
        const T x = axis[0] / norm, y = axis[1] / norm, z = axis[2] / norm;
        const T c = std::cos(angle), s= std::sin(angle), v = T{1} - c;
        return {Static2DMatrix<T, 3, 3>{
                    {
                        c + x*x*v,      x*y*v - z*s,    x*z*v + y*s,
                        y*x*v + z*s,    c + y*y*v,      y*z*v - x*s,
                        z*x*v - y*s,    z*y*v + x*s,    c + z*z*v
                    }
        }, Vector3D<T>{}};
    }

    /// Rotation `Rz(yaw) * Ry(pitch) * Rx(roll)`, i.e. roll around x is applied first, then pitch
    /// around y, then yaw around z. All angles in radians, no translation.
    static Transform3D fromRollPitchYaw(const T roll, const T pitch, const T yaw)
    {
        return fromAxisAngle(Vector3D<T>{{0, 0, 1}}, yaw)
               * fromAxisAngle(Vector3D<T>{{0, 1, 0}}, pitch)
               * fromAxisAngle(Vector3D<T>{{1, 0, 0}}, roll);
    }
private:
    Static2DMatrix<T, 3, 3> rotation_;
    Vector3D<T> translation_;
};
} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_TRANSFORM3D_H

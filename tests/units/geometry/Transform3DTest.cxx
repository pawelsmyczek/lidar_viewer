#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/Static2DMatrix.h"
#include "lidar_viewer/geometry/types/Transform3D.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

// API assumed by these tests (Transform3D<T> maps a point as `R * p + t`):
//   Transform3D()                                   identity
//   Transform3D(Static2DMatrix<T,3,3> R, Point3D<T> t)   R is not validated
//   static identity()
//   static fromTranslation(Point3D<T> t)
//   static fromAxisAngle(Point3D<T> axis, T angle)  axis need not be a unit vector, a zero axis or
//                                                   a zero angle gives the identity rotation
//   static fromRollPitchYaw(T roll, T pitch, T yaw) rotation Rz(yaw) * Ry(pitch) * Rx(roll)
//   const Static2DMatrix<T,3,3>& rotation() const
//   const Point3D<T>& translation() const
//   Point3D<T> operator()(Point3D<T> p) const       R * p + t
//   Point3D<T> rotate(Point3D<T> v) const           R * v (no translation)
//   Transform3D operator*(const Transform3D&) const (a * b)(p) == a(b(p))
//   Transform3D inverse() const
//   bool isRotation(T eps = 1e-5) const             R orthonormal with determinant +1

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Point3D;
using lidar_viewer::geometry::types::Static2DMatrix;
using lidar_viewer::geometry::types::Transform3D;

namespace
{

constexpr double Pi = std::numbers::pi;
constexpr double Tolerance = 1e-9;

template <typename T = double>
Point3D<T> point(T x, T y, T z)
{
    return Point3D<T>{std::array<T, 3>{x, y, z}};
}

/// 3x3 matrix from values given row by row
Static2DMatrix<double, 3, 3> matrix3(const std::array<double, 9>& values)
{
    Static2DMatrix<double, 3, 3> m;
    for (size_t row = 0; row < 3; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
        {
            m(col, row) = values[row * 3 + col];
        }
    }
    return m;
}

template <typename P1, typename P2>
void expectPointNear(const P1& actual, const P2& expected, double tolerance = Tolerance)
{
    EXPECT_NEAR(actual[0], expected[0], tolerance) << "x";
    EXPECT_NEAR(actual[1], expected[1], tolerance) << "y";
    EXPECT_NEAR(actual[2], expected[2], tolerance) << "z";
}

template <typename TransformT>
void expectRotationNear(const TransformT& transform, const std::array<double, 9>& expected,
                        double tolerance = Tolerance)
{
    for (size_t row = 0; row < 3; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
        {
            EXPECT_NEAR(transform.rotation()(col, row), expected[row * 3 + col], tolerance)
                << "row " << row << ", col " << col;
        }
    }
}

template <typename TransformT>
void expectTransformNear(const TransformT& actual, const TransformT& expected,
                         double tolerance = Tolerance)
{
    for (size_t row = 0; row < 3; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
        {
            EXPECT_NEAR(actual.rotation()(col, row), expected.rotation()(col, row), tolerance)
                << "row " << row << ", col " << col;
        }
    }
    expectPointNear(actual.translation(), expected.translation(), tolerance);
}

constexpr std::array<double, 9> IdentityRotation{1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1};

double norm(const Point3D<double>& p)
{
    return std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

Transform3D<double> rotationZ(double angle)
{
    return Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 1.0), angle);
}

/// a general rigid transform used by the algebra tests
Transform3D<double> samplePose()
{
    return Transform3D<double>::fromAxisAngle(point(1.0, 2.0, 3.0), 0.7)
           * Transform3D<double>::fromTranslation(point(1.5, -2.0, 0.25));
}

Transform3D<double> otherSamplePose()
{
    return Transform3D<double>::fromRollPitchYaw(0.3, -0.5, 1.1)
           * Transform3D<double>::fromTranslation(point(-4.0, 0.5, 2.0));
}

} // namespace

// ------------------------------------------------------------------ construction

TEST(Transform3DTest, DefaultConstructedIsIdentity)
{
    const Transform3D<double> t;

    expectRotationNear(t, IdentityRotation);
    expectPointNear(t.translation(), point(0.0, 0.0, 0.0));
}

TEST(Transform3DTest, IdentityFactoryIsIdentity)
{
    const auto t = Transform3D<double>::identity();

    expectRotationNear(t, IdentityRotation);
    expectPointNear(t.translation(), point(0.0, 0.0, 0.0));
}

TEST(Transform3DTest, IdentityLeavesPointsUnchanged)
{
    expectPointNear(Transform3D<double>::identity()(point(1.0, -2.0, 3.5)), point(1.0, -2.0, 3.5));
}

TEST(Transform3DTest, FromTranslationHasIdentityRotation)
{
    const auto t = Transform3D<double>::fromTranslation(point(1.0, 2.0, 3.0));

    expectRotationNear(t, IdentityRotation);
    expectPointNear(t.translation(), point(1.0, 2.0, 3.0));
}

TEST(Transform3DTest, FromTranslationShiftsPoints)
{
    const auto t = Transform3D<double>::fromTranslation(point(1.0, 2.0, 3.0));

    expectPointNear(t(point(10.0, 20.0, 30.0)), point(11.0, 22.0, 33.0));
}

TEST(Transform3DTest, ConstructedFromRotationAndTranslationStoresBoth)
{
    const auto rotation = matrix3({0, -1, 0,
                                   1,  0, 0,
                                   0,  0, 1});
    const Transform3D<double> t{rotation, point(4.0, 5.0, 6.0)};

    expectRotationNear(t, {0, -1, 0,
                           1,  0, 0,
                           0,  0, 1});
    expectPointNear(t.translation(), point(4.0, 5.0, 6.0));
}

TEST(Transform3DTest, AxisAngleAroundZRotatesXTowardsY)
{
    const auto t = Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 1.0), Pi / 2);

    expectPointNear(t(point(1.0, 0.0, 0.0)), point(0.0, 1.0, 0.0));
    expectPointNear(t(point(0.0, 1.0, 0.0)), point(-1.0, 0.0, 0.0));
    expectPointNear(t(point(0.0, 0.0, 1.0)), point(0.0, 0.0, 1.0));
}

TEST(Transform3DTest, AxisAngleAroundXRotatesYTowardsZ)
{
    const auto t = Transform3D<double>::fromAxisAngle(point(1.0, 0.0, 0.0), Pi / 2);

    expectPointNear(t(point(0.0, 1.0, 0.0)), point(0.0, 0.0, 1.0));
    expectPointNear(t(point(1.0, 0.0, 0.0)), point(1.0, 0.0, 0.0));
}

TEST(Transform3DTest, AxisAngleAroundYRotatesZTowardsX)
{
    const auto t = Transform3D<double>::fromAxisAngle(point(0.0, 1.0, 0.0), Pi / 2);

    expectPointNear(t(point(0.0, 0.0, 1.0)), point(1.0, 0.0, 0.0));
    expectPointNear(t(point(0.0, 1.0, 0.0)), point(0.0, 1.0, 0.0));
}

TEST(Transform3DTest, AxisAngleHalfTurnReversesPerpendicularVectors)
{
    const auto t = rotationZ(Pi);

    expectPointNear(t(point(1.0, 0.0, 0.0)), point(-1.0, 0.0, 0.0));
    expectPointNear(t(point(0.0, 2.0, 5.0)), point(0.0, -2.0, 5.0));
}

TEST(Transform3DTest, AxisAngleNegativeAngleRotatesTheOtherWay)
{
    expectPointNear(rotationZ(-Pi / 2)(point(1.0, 0.0, 0.0)), point(0.0, -1.0, 0.0));
}

TEST(Transform3DTest, AxisAngleAxisIsNormalised)
{
    const auto unit = Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 1.0), 0.8);
    const auto scaled = Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 5.0), 0.8);

    expectTransformNear(scaled, unit);
}

TEST(Transform3DTest, AxisAngleAroundDiagonalCyclesTheAxes)
{
    // a third of a turn around (1, 1, 1) maps x -> y -> z -> x
    const auto t = Transform3D<double>::fromAxisAngle(point(1.0, 1.0, 1.0), 2 * Pi / 3);

    expectPointNear(t(point(1.0, 0.0, 0.0)), point(0.0, 1.0, 0.0));
    expectPointNear(t(point(0.0, 1.0, 0.0)), point(0.0, 0.0, 1.0));
    expectPointNear(t(point(0.0, 0.0, 1.0)), point(1.0, 0.0, 0.0));
}

TEST(Transform3DTest, AxisAngleZeroAngleIsIdentity)
{
    expectTransformNear(Transform3D<double>::fromAxisAngle(point(1.0, 2.0, 3.0), 0.0),
                        Transform3D<double>::identity());
}

TEST(Transform3DTest, AxisAngleZeroAxisIsIdentity)
{
    expectTransformNear(Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 0.0), 1.0),
                        Transform3D<double>::identity());
}

TEST(Transform3DTest, AxisAngleHasNoTranslationAndIsARotation)
{
    const auto t = Transform3D<double>::fromAxisAngle(point(1.0, -2.0, 0.5), 1.234);

    expectPointNear(t.translation(), point(0.0, 0.0, 0.0));
    EXPECT_TRUE(t.isRotation());
}

TEST(Transform3DTest, RollPitchYawPureYawRotatesAroundZ)
{
    const auto t = Transform3D<double>::fromRollPitchYaw(0.0, 0.0, Pi / 2);

    expectPointNear(t(point(1.0, 0.0, 0.0)), point(0.0, 1.0, 0.0));
    expectTransformNear(t, rotationZ(Pi / 2));
}

TEST(Transform3DTest, RollPitchYawPurePitchRotatesAroundY)
{
    const auto t = Transform3D<double>::fromRollPitchYaw(0.0, Pi / 2, 0.0);

    expectPointNear(t(point(0.0, 0.0, 1.0)), point(1.0, 0.0, 0.0));
    expectTransformNear(t, Transform3D<double>::fromAxisAngle(point(0.0, 1.0, 0.0), Pi / 2));
}

TEST(Transform3DTest, RollPitchYawPureRollRotatesAroundX)
{
    const auto t = Transform3D<double>::fromRollPitchYaw(Pi / 2, 0.0, 0.0);

    expectPointNear(t(point(0.0, 1.0, 0.0)), point(0.0, 0.0, 1.0));
    expectTransformNear(t, Transform3D<double>::fromAxisAngle(point(1.0, 0.0, 0.0), Pi / 2));
}

TEST(Transform3DTest, RollPitchYawAppliesRollFirstThenPitchThenYaw)
{
    const double roll = 0.3;
    const double pitch = -0.5;
    const double yaw = 1.1;

    const auto expected = rotationZ(yaw)
                          * Transform3D<double>::fromAxisAngle(point(0.0, 1.0, 0.0), pitch)
                          * Transform3D<double>::fromAxisAngle(point(1.0, 0.0, 0.0), roll);

    expectTransformNear(Transform3D<double>::fromRollPitchYaw(roll, pitch, yaw), expected);
}

TEST(Transform3DTest, RollPitchYawIsARotationWithoutTranslation)
{
    const auto t = Transform3D<double>::fromRollPitchYaw(0.4, 1.2, -2.0);

    expectPointNear(t.translation(), point(0.0, 0.0, 0.0));
    EXPECT_TRUE(t.isRotation());
}

// ------------------------------------------------------------------ applying

TEST(Transform3DTest, ApplyRotatesThenTranslates)
{
    const Transform3D<double> t{rotationZ(Pi / 2).rotation(), point(1.0, 2.0, 3.0)};

    // (1, 0, 0) -> (0, 1, 0) after the rotation, then + (1, 2, 3)
    expectPointNear(t(point(1.0, 0.0, 0.0)), point(1.0, 3.0, 3.0));
}

TEST(Transform3DTest, ApplyToOriginGivesTranslation)
{
    expectPointNear(samplePose()(point(0.0, 0.0, 0.0)), samplePose().translation());
}

TEST(Transform3DTest, RotateIgnoresTranslation)
{
    const Transform3D<double> t{rotationZ(Pi / 2).rotation(), point(10.0, 20.0, 30.0)};

    expectPointNear(t.rotate(point(1.0, 0.0, 0.0)), point(0.0, 1.0, 0.0));
}

TEST(Transform3DTest, RotatePreservesLength)
{
    const auto v = point(1.0, -2.0, 3.0);

    EXPECT_NEAR(norm(samplePose().rotate(v)), norm(v), Tolerance);
}

TEST(Transform3DTest, ApplyPreservesDistancesBetweenPoints)
{
    const auto a = point(1.0, 2.0, 3.0);
    const auto b = point(-4.0, 0.5, 2.0);
    const auto pose = samplePose();
    const auto pa = pose(a);
    const auto pb = pose(b);

    EXPECT_NEAR(norm(point(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2])),
                norm(point(a[0] - b[0], a[1] - b[1], a[2] - b[2])), Tolerance);
}

TEST(Transform3DTest, ApplyDoesNotModifyTheTransform)
{
    const auto pose = samplePose();
    const auto before = pose;

    pose(point(1.0, 2.0, 3.0));

    expectTransformNear(pose, before, 0.0);
}

// ------------------------------------------------------------------ composition

TEST(Transform3DTest, CompositionAppliesTheRightOperandFirst)
{
    const auto shift = Transform3D<double>::fromTranslation(point(1.0, 0.0, 0.0));
    const auto turn = rotationZ(Pi / 2);
    const auto p = point(1.0, 0.0, 0.0);

    // turn first: (1, 0, 0) -> (0, 1, 0), then the shift: (1, 1, 0)
    expectPointNear((shift * turn)(p), point(1.0, 1.0, 0.0));
    // shift first: (2, 0, 0), then the turn: (0, 2, 0)
    expectPointNear((turn * shift)(p), point(0.0, 2.0, 0.0));
}

TEST(Transform3DTest, CompositionEqualsApplyingTheTransformsOneAfterAnother)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();
    const auto p = point(0.3, -1.2, 4.0);

    expectPointNear((a * b)(p), a(b(p)));
    expectPointNear((b * a)(p), b(a(p)));
}

TEST(Transform3DTest, CompositionIsNotCommutative)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();
    const auto p = point(1.0, 0.0, 0.0);
    const auto ab = (a * b)(p);
    const auto ba = (b * a)(p);

    EXPECT_GT(std::abs(ab[0] - ba[0]) + std::abs(ab[1] - ba[1]) + std::abs(ab[2] - ba[2]), 1e-3);
}

TEST(Transform3DTest, CompositionIsAssociative)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();
    const auto c = Transform3D<double>::fromRollPitchYaw(-0.2, 0.9, 0.1);

    expectTransformNear((a * b) * c, a * (b * c));
}

TEST(Transform3DTest, IdentityIsNeutralOnBothSides)
{
    const auto pose = samplePose();
    const auto identity = Transform3D<double>::identity();

    expectTransformNear(identity * pose, pose);
    expectTransformNear(pose * identity, pose);
}

TEST(Transform3DTest, CompositionOfRotationsAddsTheAngles)
{
    expectTransformNear(rotationZ(Pi / 2) * rotationZ(Pi / 2), rotationZ(Pi));
    expectTransformNear(rotationZ(0.4) * rotationZ(0.3), rotationZ(0.7));
}

TEST(Transform3DTest, CompositionOfTranslationsAddsThem)
{
    const auto t = Transform3D<double>::fromTranslation(point(1.0, 2.0, 3.0))
                   * Transform3D<double>::fromTranslation(point(10.0, 20.0, 30.0));

    expectRotationNear(t, IdentityRotation);
    expectPointNear(t.translation(), point(11.0, 22.0, 33.0));
}

TEST(Transform3DTest, ComposedTranslationIsLeftOperandAppliedToRightTranslation)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();

    expectPointNear((a * b).translation(), a(b.translation()));
}

TEST(Transform3DTest, CompositionDoesNotModifyTheOperands)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();
    const auto aBefore = a;
    const auto bBefore = b;

    const auto ab = a * b;
    (void) ab;

    expectTransformNear(a, aBefore, 0.0);
    expectTransformNear(b, bBefore, 0.0);
}

// ------------------------------------------------------------------ inverse

TEST(Transform3DTest, InverseOfIdentityIsIdentity)
{
    expectTransformNear(Transform3D<double>::identity().inverse(), Transform3D<double>::identity());
}

TEST(Transform3DTest, InverseOfTranslationIsTheOppositeTranslation)
{
    const auto inverse = Transform3D<double>::fromTranslation(point(1.0, -2.0, 3.0)).inverse();

    expectRotationNear(inverse, IdentityRotation);
    expectPointNear(inverse.translation(), point(-1.0, 2.0, -3.0));
}

TEST(Transform3DTest, InverseOfRotationIsTheTransposedRotation)
{
    const auto inverse = rotationZ(Pi / 2).inverse();

    expectRotationNear(inverse, {0, 1, 0,
                                 -1, 0, 0,
                                 0, 0, 1});
    expectPointNear(inverse.translation(), point(0.0, 0.0, 0.0));
}

TEST(Transform3DTest, InverseUndoesTheTransform)
{
    const auto pose = samplePose();
    const auto p = point(0.3, -1.2, 4.0);

    expectPointNear(pose.inverse()(pose(p)), p);
    expectPointNear(pose(pose.inverse()(p)), p);
}

TEST(Transform3DTest, ComposingWithTheInverseGivesIdentity)
{
    const auto pose = samplePose();

    expectTransformNear(pose * pose.inverse(), Transform3D<double>::identity());
    expectTransformNear(pose.inverse() * pose, Transform3D<double>::identity());
}

TEST(Transform3DTest, InverseOfInverseIsTheOriginal)
{
    expectTransformNear(samplePose().inverse().inverse(), samplePose());
}

TEST(Transform3DTest, InverseOfCompositionIsReversedCompositionOfInverses)
{
    const auto a = samplePose();
    const auto b = otherSamplePose();

    expectTransformNear((a * b).inverse(), b.inverse() * a.inverse());
}

TEST(Transform3DTest, InverseDoesNotModifyTheTransform)
{
    const auto pose = samplePose();
    const auto before = pose;

    const auto inverse = pose.inverse();
    (void) inverse;

    expectTransformNear(pose, before, 0.0);
}

// ------------------------------------------------------------------ isRotation

TEST(Transform3DTest, IdentityIsARotation)
{
    EXPECT_TRUE(Transform3D<double>::identity().isRotation());
}

TEST(Transform3DTest, TranslationDoesNotAffectIsRotation)
{
    EXPECT_TRUE(Transform3D<double>::fromTranslation(point(100.0, -50.0, 3.0)).isRotation());
    EXPECT_TRUE(samplePose().isRotation());
}

TEST(Transform3DTest, ScaledMatrixIsNotARotation)
{
    const Transform3D<double> t{matrix3({2, 0, 0,
                                         0, 2, 0,
                                         0, 0, 2}),
                                point(0.0, 0.0, 0.0)};

    EXPECT_FALSE(t.isRotation());
}

TEST(Transform3DTest, ReflectionIsNotARotation)
{
    // orthonormal, but the determinant is -1
    const Transform3D<double> t{matrix3({1, 0,  0,
                                         0, 1,  0,
                                         0, 0, -1}),
                                point(0.0, 0.0, 0.0)};

    EXPECT_FALSE(t.isRotation());
}

TEST(Transform3DTest, ShearIsNotARotation)
{
    const Transform3D<double> t{matrix3({1, 1, 0,
                                         0, 1, 0,
                                         0, 0, 1}),
                                point(0.0, 0.0, 0.0)};

    EXPECT_FALSE(t.isRotation());
}

TEST(Transform3DTest, ZeroMatrixIsNotARotation)
{
    const Transform3D<double> t{Static2DMatrix<double, 3, 3>{}, point(0.0, 0.0, 0.0)};

    EXPECT_FALSE(t.isRotation());
}

TEST(Transform3DTest, IsRotationToleranceCanBeAdjusted)
{
    // a rotation about z with the (0, 0) element off by 1e-3
    const Transform3D<double> t{matrix3({1.001, 0, 0,
                                         0,     1, 0,
                                         0,     0, 1}),
                                point(0.0, 0.0, 0.0)};

    EXPECT_FALSE(t.isRotation());
    EXPECT_FALSE(t.isRotation(1e-6));
    EXPECT_TRUE(t.isRotation(1e-2));
}

TEST(Transform3DTest, LongChainOfSmallRotationsStaysARotation)
{
    const auto step = Transform3D<double>::fromRollPitchYaw(0.01, 0.02, 0.03);
    auto accumulated = Transform3D<double>::identity();
    for (int i = 0; i < 1000; ++i)
    {
        accumulated = step * accumulated;
    }

    EXPECT_TRUE(accumulated.isRotation());
}

// ------------------------------------------------------------------ float

TEST(Transform3DTest, WorksWithFloatCoordinates)
{
    const auto t = Transform3D<float>::fromAxisAngle(point(0.0f, 0.0f, 1.0f), static_cast<float>(Pi / 2))
                   * Transform3D<float>::fromTranslation(point(1.0f, 2.0f, 3.0f));

    // translation first: (2, 2, 3), then the turn: (-2, 2, 3)
    expectPointNear(t(point(1.0f, 0.0f, 0.0f)), point(-2.0, 2.0, 3.0), 1e-5);
    expectPointNear(t.inverse()(t(point(1.0f, 0.0f, 0.0f))), point(1.0, 0.0, 0.0), 1e-5);
    EXPECT_TRUE(t.isRotation());
}

} // namespace lidar_viewer::tests::units

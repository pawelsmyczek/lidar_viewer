#include "lidar_viewer/geometry/types/DynamicMatrix.h"
#include "lidar_viewer/geometry/types/Static2DMatrix.h"
#include "lidar_viewer/geometry/types/Vector.h"

#include <gtest/gtest.h>

#include <type_traits>
#include <utility>
#include <vector>

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::DynamicMatrix;
using lidar_viewer::geometry::types::outer;
using lidar_viewer::geometry::types::Static2DMatrix;
using lidar_viewer::geometry::types::Vector;

namespace
{

/// true when `MatrixT::fromRows(vectors...)` compiles
template <typename MatrixT, typename... Vectors>
constexpr bool canBuildFromRows = requires { MatrixT::fromRows(std::declval<Vectors>()...); };

/// true when `MatrixT::fromColumns(vectors...)` compiles
template <typename MatrixT, typename... Vectors>
constexpr bool canBuildFromColumns = requires { MatrixT::fromColumns(std::declval<Vectors>()...); };

/// true when `matrix * vector` compiles (a template, so a failed expression gives false)
template <typename MatrixT, typename VectorT>
constexpr bool canMultiply = requires(MatrixT m, VectorT v) { m * v; };

// Matrices are addressed as (x, y) = (column, row), element index is `row * cols + col`.

/// fills a matrix with `values` given in row-major order
template <typename MatrixT>
void fill(MatrixT& m, const size_t rows, const size_t cols, const std::vector<double>& values)
{
    ASSERT_EQ(values.size(), rows * cols);
    for (size_t r = 0; r < rows; ++r)
    {
        for (size_t c = 0; c < cols; ++c)
        {
            m(c, r) = values[r * cols + c];
        }
    }
}

/// expects `m` to hold `values` given in row-major order
template <typename MatrixT>
void expectEqual(const MatrixT& m, const size_t rows, const size_t cols, const std::vector<double>& values)
{
    ASSERT_EQ(values.size(), rows * cols);
    for (size_t r = 0; r < rows; ++r)
    {
        for (size_t c = 0; c < cols; ++c)
        {
            EXPECT_DOUBLE_EQ(m(c, r), values[r * cols + c]) << "row " << r << ", col " << c;
        }
    }
}

} // namespace

// ---------------------------------------------------------------- Static2DMatrix

TEST(Static2DMatrixTest, IsZeroInitialisedAfterDefaultConstruction)
{
    Static2DMatrix<double, 3, 2> m;
    expectEqual(m, 3, 2, {0, 0,
                          0, 0,
                          0, 0});
}

TEST(Static2DMatrixTest, StoresElementsRowMajor)
{
    Static2DMatrix<double, 2, 3> m;
    m(2, 0) = 7.0; // column 2, row 0
    m(0, 1) = 9.0; // column 0, row 1

    EXPECT_DOUBLE_EQ(m.get(0 * 3 + 2), 7.0);
    EXPECT_DOUBLE_EQ(m.get(1 * 3 + 0), 9.0);
}

TEST(Static2DMatrixTest, ConstAccessReadsElements)
{
    Static2DMatrix<double, 2, 2> m;
    fill(m, 2, 2, {1, 2,
                   3, 4});
    const auto& cm = m;

    EXPECT_DOUBLE_EQ(cm(0, 0), 1);
    EXPECT_DOUBLE_EQ(cm(1, 0), 2);
    EXPECT_DOUBLE_EQ(cm(0, 1), 3);
    EXPECT_DOUBLE_EQ(cm(1, 1), 4);
}

TEST(Static2DMatrixTest, AccessPastTheEndThrows)
{
    Static2DMatrix<double, 2, 3> m;

    EXPECT_THROW(m.get(6), std::runtime_error);
    EXPECT_THROW(m(3, 1), std::runtime_error); // index 1 * 3 + 3 == 6
    EXPECT_NO_THROW(m(2, 1));                  // last element
    EXPECT_THROW(m(3, 0), std::runtime_error); // column past the end, must not wrap to row 1
    EXPECT_THROW(m(0, 2), std::runtime_error); // row past the end
}

TEST(Static2DMatrixTest, Addition)
{
    Static2DMatrix<double, 2, 3> a;
    Static2DMatrix<double, 2, 3> b;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 2, 3, {10, 20, 30,
                   40, 50, 60});

    expectEqual(a + b, 2, 3, {11, 22, 33,
                              44, 55, 66});
}

TEST(Static2DMatrixTest, Subtraction)
{
    Static2DMatrix<double, 2, 3> a;
    Static2DMatrix<double, 2, 3> b;
    fill(a, 2, 3, {10, 20, 30,
                   40, 50, 60});
    fill(b, 2, 3, {1, 2, 3,
                   4, 5, 6});

    expectEqual(a - b, 2, 3, {9, 18, 27,
                              36, 45, 54});
}

TEST(Static2DMatrixTest, MultiplicationOfSquareMatricesIsNotCommutative)
{
    Static2DMatrix<double, 2, 2> a;
    Static2DMatrix<double, 2, 2> b;
    fill(a, 2, 2, {1, 2,
                   3, 4});
    fill(b, 2, 2, {0, 1,
                   1, 0});

    expectEqual(a * b, 2, 2, {2, 1,
                              4, 3});
    expectEqual(b * a, 2, 2, {3, 4,
                              1, 2});
}

TEST(Static2DMatrixTest, MultiplicationByIdentityKeepsMatrix)
{
    Static2DMatrix<double, 3, 3> a;
    Static2DMatrix<double, 3, 3> identity;
    fill(a, 3, 3, {1, 2, 3,
                   4, 5, 6,
                   7, 8, 9});
    fill(identity, 3, 3, {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1});

    expectEqual(a * identity, 3, 3, {1, 2, 3,
                                     4, 5, 6,
                                     7, 8, 9});
    expectEqual(identity * a, 3, 3, {1, 2, 3,
                                     4, 5, 6,
                                     7, 8, 9});
}

TEST(Static2DMatrixTest, MultiplicationOfNonSquareMatrices)
{
    Static2DMatrix<double, 2, 3> a;
    Static2DMatrix<double, 3, 2> b;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 3, 2, {7, 8,
                   9, 10,
                   11, 12});

    expectEqual(a * b, 2, 2, {58, 64,
                              139, 154});
    expectEqual(b * a, 3, 3, {39, 54, 69,
                              49, 68, 87,
                              59, 82, 105});
}

TEST(Static2DMatrixTest, CopyConstructedMatrixHasTheSameValuesAndIsIndependent)
{
    Static2DMatrix<double, 2, 2> original;
    fill(original, 2, 2, {1, 2,
                          3, 4});

    Static2DMatrix<double, 2, 2> copy{original};
    expectEqual(copy, 2, 2, {1, 2,
                             3, 4});

    copy(0, 0) = 100;
    EXPECT_DOUBLE_EQ(original(0, 0), 1) << "copy shares storage with the original";
    EXPECT_DOUBLE_EQ(copy(0, 0), 100);
}

TEST(Static2DMatrixTest, CopyAssignedMatrixHasTheSameValuesAndIsIndependent)
{
    Static2DMatrix<double, 2, 2> original;
    fill(original, 2, 2, {1, 2,
                          3, 4});

    Static2DMatrix<double, 2, 2> target = original;
    expectEqual(target, 2, 2, {1, 2,
                               3, 4});

    target(1, 1) = 100;
    EXPECT_DOUBLE_EQ(original(1, 1), 4) << "copy shares storage with the original";
}

TEST(Static2DMatrixTest, MovedMatrixKeepsValuesAndStaysUsable)
{
    Static2DMatrix<double, 2, 2> source;
    fill(source, 2, 2, {1, 2,
                        3, 4});

    Static2DMatrix<double, 2, 2> moved{source};
    expectEqual(moved, 2, 2, {1, 2,
                              3, 4});

    // access goes through the base class pointer, it must point into `moved`, not `source`
    moved(1, 0) = 20;
    EXPECT_DOUBLE_EQ(moved.get(1), 20);
}

TEST(Static2DMatrixTest, CopyOfCopyKeepsAccessThroughOwnStorage)
{
    Static2DMatrix<double, 2, 2> a;
    fill(a, 2, 2, {1, 2,
                   3, 4});
    auto b = a;
    auto c = b;
    b(0, 0) = -1;

    EXPECT_DOUBLE_EQ(c.get(0), 1);
    EXPECT_DOUBLE_EQ(b.get(0), -1);
}

TEST(Static2DMatrixTest, TransposeSwapsRowsAndColumns)
{
    Static2DMatrix<double, 2, 3> a;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    const auto t = a.transpose();

    static_assert(std::is_same_v<std::remove_const_t<decltype(t)>, Static2DMatrix<double, 3, 2>>);
    expectEqual(t, 3, 2, {1, 4,
                          2, 5,
                          3, 6});
}

TEST(Static2DMatrixTest, TransposeOfSquareMatrix)
{
    Static2DMatrix<double, 3, 3> a;
    fill(a, 3, 3, {1, 2, 3,
                   4, 5, 6,
                   7, 8, 9});

    expectEqual(a.transpose(), 3, 3, {1, 4, 7,
                                      2, 5, 8,
                                      3, 6, 9});
}

TEST(Static2DMatrixTest, TransposeDoesNotModifyTheOriginal)
{
    Static2DMatrix<double, 2, 3> a;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    auto t = a.transpose();
    t(0, 0) = 100;

    expectEqual(a, 2, 3, {1, 2, 3,
                          4, 5, 6});
}

TEST(Static2DMatrixTest, TransposingTwiceGivesTheOriginal)
{
    Static2DMatrix<double, 2, 3> a;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    expectEqual(a.transpose().transpose(), 2, 3, {1, 2, 3,
                                                  4, 5, 6});
}

TEST(Static2DMatrixTest, TransposeOfProductIsProductOfTransposesInReverseOrder)
{
    Static2DMatrix<double, 2, 3> a;
    Static2DMatrix<double, 3, 2> b;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 3, 2, {7, 8,
                   9, 10,
                   11, 12});

    expectEqual((a * b).transpose(), 2, 2, {58, 139,
                                            64, 154});
    expectEqual(b.transpose() * a.transpose(), 2, 2, {58, 139,
                                                      64, 154});
}

TEST(Static2DMatrixTest, IdentityHasOnesOnTheDiagonalAndZerosElsewhere)
{
    expectEqual(Static2DMatrix<double, 3, 3>::identity(), 3, 3, {1, 0, 0,
                                                                  0, 1, 0,
                                                                  0, 0, 1});
}

TEST(Static2DMatrixTest, IdentityOfOneByOneMatrix)
{
    expectEqual(Static2DMatrix<double, 1, 1>::identity(), 1, 1, {1});
}

TEST(Static2DMatrixTest, MultiplyingByIdentityFromEitherSideKeepsTheMatrix)
{
    Static2DMatrix<double, 2, 3> a;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    expectEqual(Static2DMatrix<double, 2, 2>::identity() * a, 2, 3, {1, 2, 3,
                                                                      4, 5, 6});
    expectEqual(a * Static2DMatrix<double, 3, 3>::identity(), 2, 3, {1, 2, 3,
                                                                      4, 5, 6});
}

TEST(Static2DMatrixTest, IdentityIsItsOwnTranspose)
{
    expectEqual(Static2DMatrix<double, 3, 3>::identity().transpose(), 3, 3, {1, 0, 0,
                                                                              0, 1, 0,
                                                                              0, 0, 1});
}

TEST(Static2DMatrixTest, MultiplyingByVectorGivesTheDotProductsOfTheRows)
{
    Static2DMatrix<double, 2, 3> a;
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    const Vector<double, 3> v{{1.0, 0.0, -1.0}};

    const auto result = a * v;

    static_assert(std::is_same_v<std::remove_const_t<decltype(result)>, Vector<double, 2>>);
    EXPECT_DOUBLE_EQ(result[0], -2.0);
    EXPECT_DOUBLE_EQ(result[1], -2.0);
}

TEST(Static2DMatrixTest, MultiplyingByVectorOfNonSquareMatrixChangesTheDimension)
{
    Static2DMatrix<double, 3, 2> a;
    fill(a, 3, 2, {1, 2,
                   3, 4,
                   5, 6});
    const Vector<double, 2> v{{1.0, 10.0}};

    const auto result = a * v;

    static_assert(std::is_same_v<std::remove_const_t<decltype(result)>, Vector<double, 3>>);
    EXPECT_DOUBLE_EQ(result[0], 21.0);
    EXPECT_DOUBLE_EQ(result[1], 43.0);
    EXPECT_DOUBLE_EQ(result[2], 65.0);
}

TEST(Static2DMatrixTest, IdentityTimesVectorIsTheVector)
{
    const Vector<double, 3> v{{1.5, -2.0, 3.25}};

    const auto result = Static2DMatrix<double, 3, 3>::identity() * v;

    EXPECT_DOUBLE_EQ(result[0], 1.5);
    EXPECT_DOUBLE_EQ(result[1], -2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.25);
}

TEST(Static2DMatrixTest, RotationMatrixTimesVectorRotatesIt)
{
    Static2DMatrix<double, 3, 3> quarterTurnAroundZ;
    fill(quarterTurnAroundZ, 3, 3, {0, -1, 0,
                                    1,  0, 0,
                                    0,  0, 1});

    const auto result = quarterTurnAroundZ * Vector<double, 3>{{1.0, 0.0, 0.0}};

    EXPECT_DOUBLE_EQ(result[0], 0.0);
    EXPECT_DOUBLE_EQ(result[1], 1.0);
    EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST(Static2DMatrixTest, MultiplyingTheZeroVectorGivesTheZeroVector)
{
    Static2DMatrix<double, 3, 3> a;
    fill(a, 3, 3, {1, 2, 3,
                   4, 5, 6,
                   7, 8, 9});

    const auto result = a * Vector<double, 3>{};

    EXPECT_DOUBLE_EQ(result[0], 0.0);
    EXPECT_DOUBLE_EQ(result[1], 0.0);
    EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST(Static2DMatrixTest, MatrixTimesVectorIsConsistentWithTheMatrixProduct)
{
    Static2DMatrix<double, 3, 3> a;
    Static2DMatrix<double, 3, 3> b;
    fill(a, 3, 3, {2, -3, 1,
                   2,  0, -1,
                   1,  4, 5});
    fill(b, 3, 3, {1, 2, 0,
                   0, 1, 3,
                   4, 0, 1});
    const Vector<double, 3> v{{1.0, -2.0, 0.5}};

    const auto twoSteps = a * (b * v);
    const auto oneStep = (a * b) * v;

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(twoSteps[i], oneStep[i], 1e-12) << "coordinate " << i;
    }
}

TEST(Static2DMatrixTest, MatrixTimesVectorIsLinear)
{
    Static2DMatrix<double, 3, 3> a;
    fill(a, 3, 3, {2, -3, 1,
                   2,  0, -1,
                   1,  4, 5});
    Vector<double, 3> u{{1.0, 2.0, 3.0}};
    Vector<double, 3> v{{-1.0, 0.5, 4.0}};

    const auto sumFirst = a * (u + v);
    const auto au = a * u;
    const auto av = a * v;

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(sumFirst[i], au[i] + av[i], 1e-12) << "coordinate " << i;
    }
}

TEST(Static2DMatrixTest, MultiplyingByVectorDoesNotModifyTheOperands)
{
    Static2DMatrix<double, 2, 2> a;
    fill(a, 2, 2, {1, 2,
                   3, 4});
    const Vector<double, 2> v{{5.0, 6.0}};

    const auto result = a * v;
    (void) result;

    expectEqual(a, 2, 2, {1, 2,
                          3, 4});
    EXPECT_DOUBLE_EQ(v[0], 5.0);
    EXPECT_DOUBLE_EQ(v[1], 6.0);
}

TEST(Static2DMatrixTest, MultiplyingByVectorOfWrongSizeDoesNotCompile)
{
    static_assert(canMultiply<Static2DMatrix<double, 2, 3>, Vector<double, 3>>);
    static_assert(!canMultiply<Static2DMatrix<double, 2, 3>, Vector<double, 2>>);
    SUCCEED();
}

TEST(Static2DMatrixTest, MultiplyingByVectorWorksForFloat)
{
    Static2DMatrix<float, 2, 2> a;
    a(0, 0) = 1.0f; a(1, 0) = 2.0f;
    a(0, 1) = 3.0f; a(1, 1) = 4.0f;

    const auto result = a * Vector<float, 2>{{1.0f, 1.0f}};

    EXPECT_FLOAT_EQ(result[0], 3.0f);
    EXPECT_FLOAT_EQ(result[1], 7.0f);
}

TEST(Static2DMatrixTest, FromRowsPutsEachVectorInARow)
{
    const auto m = Static2DMatrix<double, 2, 3>::fromRows(Vector<double, 3>{{1.0, 2.0, 3.0}},
                                                          Vector<double, 3>{{4.0, 5.0, 6.0}});

    expectEqual(m, 2, 3, {1, 2, 3,
                          4, 5, 6});
}

TEST(Static2DMatrixTest, FromColumnsPutsEachVectorInAColumn)
{
    const auto m = Static2DMatrix<double, 2, 3>::fromColumns(Vector<double, 2>{{1.0, 4.0}},
                                                             Vector<double, 2>{{2.0, 5.0}},
                                                             Vector<double, 2>{{3.0, 6.0}});

    expectEqual(m, 2, 3, {1, 2, 3,
                          4, 5, 6});
}

TEST(Static2DMatrixTest, FromRowsAndFromColumnsOfTheSameVectorsAreTransposes)
{
    const Vector<double, 3> a{{1.0, 2.0, 3.0}};
    const Vector<double, 3> b{{4.0, 5.0, 6.0}};
    const Vector<double, 3> c{{7.0, 8.0, 10.0}};

    const auto rows = Static2DMatrix<double, 3, 3>::fromRows(a, b, c);
    const auto columns = Static2DMatrix<double, 3, 3>::fromColumns(a, b, c);

    expectEqual(rows, 3, 3, {1, 2, 3,
                             4, 5, 6,
                             7, 8, 10});
    expectEqual(columns.transpose(), 3, 3, {1, 2, 3,
                                            4, 5, 6,
                                            7, 8, 10});
}

TEST(Static2DMatrixTest, FromRowsOfTheUnitVectorsIsTheIdentity)
{
    const auto m = Static2DMatrix<double, 3, 3>::fromRows(Vector<double, 3>{{1.0, 0.0, 0.0}},
                                                          Vector<double, 3>{{0.0, 1.0, 0.0}},
                                                          Vector<double, 3>{{0.0, 0.0, 1.0}});

    expectEqual(m, 3, 3, {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1});
}

TEST(Static2DMatrixTest, MatrixMapsTheUnitVectorsToItsColumns)
{
    const Vector<double, 3> imageOfX{{0.0, 1.0, 0.0}};
    const Vector<double, 3> imageOfY{{-1.0, 0.0, 0.0}};
    const Vector<double, 3> imageOfZ{{0.0, 0.0, 1.0}};
    const auto m = Static2DMatrix<double, 3, 3>::fromColumns(imageOfX, imageOfY, imageOfZ);

    const auto x = m * Vector<double, 3>{{1.0, 0.0, 0.0}};
    const auto y = m * Vector<double, 3>{{0.0, 1.0, 0.0}};
    const auto z = m * Vector<double, 3>{{0.0, 0.0, 1.0}};

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_DOUBLE_EQ(x[i], imageOfX[i]);
        EXPECT_DOUBLE_EQ(y[i], imageOfY[i]);
        EXPECT_DOUBLE_EQ(z[i], imageOfZ[i]);
    }
}

TEST(Static2DMatrixTest, MultiplyingMatrixFromRowsByVectorGivesTheDotProducts)
{
    const auto m = Static2DMatrix<double, 2, 3>::fromRows(Vector<double, 3>{{1.0, 2.0, 3.0}},
                                                          Vector<double, 3>{{4.0, 5.0, 6.0}});

    const auto result = m * Vector<double, 3>{{1.0, 0.0, -1.0}};

    EXPECT_DOUBLE_EQ(result[0], -2.0);
    EXPECT_DOUBLE_EQ(result[1], -2.0);
}

TEST(Static2DMatrixTest, FromRowsAndFromColumnsDoNotModifyTheVectors)
{
    const Vector<double, 2> a{{1.0, 2.0}};
    const Vector<double, 2> b{{3.0, 4.0}};

    const auto rows = Static2DMatrix<double, 2, 2>::fromRows(a, b);
    const auto columns = Static2DMatrix<double, 2, 2>::fromColumns(a, b);
    (void) rows;
    (void) columns;

    EXPECT_DOUBLE_EQ(a[0], 1.0);
    EXPECT_DOUBLE_EQ(a[1], 2.0);
    EXPECT_DOUBLE_EQ(b[0], 3.0);
    EXPECT_DOUBLE_EQ(b[1], 4.0);
}

TEST(Static2DMatrixTest, FromRowsAndFromColumnsCheckTheVectorsAtCompileTime)
{
    using Matrix23 = Static2DMatrix<double, 2, 3>;
    using Row = Vector<double, 3>;
    using Column = Vector<double, 2>;

    static_assert(canBuildFromRows<Matrix23, Row, Row>);
    static_assert(!canBuildFromRows<Matrix23, Row>);                 // too few rows
    static_assert(!canBuildFromRows<Matrix23, Row, Row, Row>);       // too many rows
    static_assert(!canBuildFromRows<Matrix23, Column, Column>);      // wrong length
    static_assert(!canBuildFromRows<Matrix23, Vector<float, 3>, Vector<float, 3>>);  // wrong type

    static_assert(canBuildFromColumns<Matrix23, Column, Column, Column>);
    static_assert(!canBuildFromColumns<Matrix23, Column, Column>);   // too few columns
    static_assert(!canBuildFromColumns<Matrix23, Row, Row, Row>);    // wrong length
    SUCCEED();
}

TEST(Static2DMatrixTest, OuterProductMultipliesEveryPairOfCoordinates)
{
    const Vector<double, 3> a{{1.0, 2.0, 3.0}};
    const Vector<double, 2> b{{4.0, 5.0}};

    const auto m = outer(a, b);

    static_assert(std::is_same_v<std::remove_const_t<decltype(m)>, Static2DMatrix<double, 3, 2>>);
    expectEqual(m, 3, 2, {4, 5,
                          8, 10,
                          12, 15});
}

TEST(Static2DMatrixTest, OuterProductOfUnitVectorsHasASingleOne)
{
    const auto m = outer(Vector<double, 3>{{0.0, 1.0, 0.0}}, Vector<double, 3>{{0.0, 0.0, 1.0}});

    // a[1] * b[2]: row 1, column 2
    expectEqual(m, 3, 3, {0, 0, 0,
                          0, 0, 1,
                          0, 0, 0});
}

TEST(Static2DMatrixTest, OuterProductWithTheZeroVectorIsTheZeroMatrix)
{
    const auto m = outer(Vector<double, 3>{{1.0, 2.0, 3.0}}, Vector<double, 3>{});

    expectEqual(m, 3, 3, {0, 0, 0,
                          0, 0, 0,
                          0, 0, 0});
}

TEST(Static2DMatrixTest, OuterProductOfAVectorWithItselfIsSymmetric)
{
    const Vector<double, 3> a{{1.0, -2.0, 3.0}};

    const auto m = outer(a, a);

    expectEqual(m.transpose(), 3, 3, {1, -2, 3,
                                      -2, 4, -6,
                                      3, -6, 9});
    expectEqual(m, 3, 3, {1, -2, 3,
                          -2, 4, -6,
                          3, -6, 9});
}

TEST(Static2DMatrixTest, TransposeOfOuterProductSwapsTheVectors)
{
    const Vector<double, 3> a{{1.0, 2.0, 3.0}};
    const Vector<double, 2> b{{4.0, 5.0}};

    expectEqual(outer(a, b).transpose(), 2, 3, {4, 8, 12,
                                                5, 10, 15});
    expectEqual(outer(b, a), 2, 3, {4, 8, 12,
                                    5, 10, 15});
}

TEST(Static2DMatrixTest, OuterProductTimesVectorScalesTheLeftVectorByADotProduct)
{
    const Vector<double, 3> a{{1.0, 2.0, 3.0}};
    const Vector<double, 3> b{{4.0, -1.0, 2.0}};
    const Vector<double, 3> v{{0.5, 2.0, -1.0}};

    const auto result = outer(a, b) * v;   // a * (b . v)
    const double dot = b[0] * v[0] + b[1] * v[1] + b[2] * v[2];

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(result[i], a[i] * dot, 1e-12) << "coordinate " << i;
    }
}

TEST(Static2DMatrixTest, SumOfOuterProductsEqualsProductOfColumnMatrices)
{
    // the cross-covariance of point sets: sum of p_i * q_iᵀ == P * Qᵀ with the points as columns
    const Vector<double, 3> p1{{1.0, 2.0, 3.0}};
    const Vector<double, 3> p2{{-1.0, 0.5, 2.0}};
    const Vector<double, 3> q1{{0.0, 1.0, 4.0}};
    const Vector<double, 3> q2{{2.0, -3.0, 1.0}};

    const auto summed = outer(p1, q1) + outer(p2, q2);
    const auto product = Static2DMatrix<double, 3, 2>::fromColumns(p1, p2)
                         * Static2DMatrix<double, 3, 2>::fromColumns(q1, q2).transpose();

    for (size_t row = 0; row < 3; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
        {
            EXPECT_NEAR(summed(col, row), product(col, row), 1e-12) << "row " << row << ", col " << col;
        }
    }
}

TEST(Static2DMatrixTest, OuterProductWorksForFloat)
{
    const auto m = outer(Vector<float, 2>{{1.0f, 2.0f}}, Vector<float, 2>{{3.0f, 4.0f}});

    EXPECT_FLOAT_EQ(m(0, 0), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 6.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 8.0f);
}

// ---------------------------------------------------------------- DynamicMatrix

TEST(DynamicMatrixTest, ReportsItsResolution)
{
    const DynamicMatrix<double> m{3, 2};

    EXPECT_EQ(m.resolution(), std::make_pair(size_t{3}, size_t{2}));
}

TEST(DynamicMatrixTest, IsZeroInitialisedAfterConstruction)
{
    const DynamicMatrix<double> m{3, 2};

    expectEqual(m, 3, 2, {0, 0,
                          0, 0,
                          0, 0});
}

TEST(DynamicMatrixTest, StoresElementsRowMajor)
{
    DynamicMatrix<double> m{2, 3};
    m(2, 0) = 7.0; // column 2, row 0
    m(0, 1) = 9.0; // column 0, row 1

    EXPECT_DOUBLE_EQ(m.get(0 * 3 + 2), 7.0);
    EXPECT_DOUBLE_EQ(m.get(1 * 3 + 0), 9.0);
}

TEST(DynamicMatrixTest, ConstAccessReadsElements)
{
    DynamicMatrix<double> m{2, 2};
    fill(m, 2, 2, {1, 2,
                   3, 4});
    const auto& cm = m;

    EXPECT_DOUBLE_EQ(cm(0, 0), 1);
    EXPECT_DOUBLE_EQ(cm(1, 0), 2);
    EXPECT_DOUBLE_EQ(cm(0, 1), 3);
    EXPECT_DOUBLE_EQ(cm(1, 1), 4);
}

TEST(DynamicMatrixTest, AccessPastTheEndThrows)
{
    DynamicMatrix<double> m{2, 3};

    EXPECT_THROW(m.get(6), std::runtime_error);
    EXPECT_THROW(m(3, 1), std::runtime_error); // index 1 * 3 + 3 == 6
    EXPECT_NO_THROW(m(2, 1));                  // last element
    EXPECT_THROW(m(3, 0), std::runtime_error); // column past the end, must not wrap to row 1
    EXPECT_THROW(m(0, 2), std::runtime_error); // row past the end
}

TEST(DynamicMatrixTest, Addition)
{
    DynamicMatrix<double> a{2, 3};
    DynamicMatrix<double> b{2, 3};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 2, 3, {10, 20, 30,
                   40, 50, 60});

    expectEqual(a + b, 2, 3, {11, 22, 33,
                              44, 55, 66});
}

TEST(DynamicMatrixTest, Subtraction)
{
    DynamicMatrix<double> a{2, 3};
    DynamicMatrix<double> b{2, 3};
    fill(a, 2, 3, {10, 20, 30,
                   40, 50, 60});
    fill(b, 2, 3, {1, 2, 3,
                   4, 5, 6});

    expectEqual(a - b, 2, 3, {9, 18, 27,
                              36, 45, 54});
}

TEST(DynamicMatrixTest, MultiplicationOfSquareMatricesIsNotCommutative)
{
    DynamicMatrix<double> a{2, 2};
    DynamicMatrix<double> b{2, 2};
    fill(a, 2, 2, {1, 2,
                   3, 4});
    fill(b, 2, 2, {0, 1,
                   1, 0});

    expectEqual(a * b, 2, 2, {2, 1,
                              4, 3});
    expectEqual(b * a, 2, 2, {3, 4,
                              1, 2});
}

TEST(DynamicMatrixTest, MultiplicationOfNonSquareMatrices)
{
    DynamicMatrix<double> a{2, 3};
    DynamicMatrix<double> b{3, 2};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 3, 2, {7, 8,
                   9, 10,
                   11, 12});

    const auto ab = a * b;
    EXPECT_EQ(ab.resolution(), std::make_pair(size_t{2}, size_t{2}));
    expectEqual(ab, 2, 2, {58, 64,
                           139, 154});

    const auto ba = b * a;
    EXPECT_EQ(ba.resolution(), std::make_pair(size_t{3}, size_t{3}));
    expectEqual(ba, 3, 3, {39, 54, 69,
                           49, 68, 87,
                           59, 82, 105});
}

TEST(DynamicMatrixTest, OperationsOnMismatchedDimensionsThrow)
{
    DynamicMatrix<double> a{2, 3};
    DynamicMatrix<double> b{3, 2};

    EXPECT_THROW(a + b, std::runtime_error);
    EXPECT_THROW(a - b, std::runtime_error);
    EXPECT_THROW(a * a, std::runtime_error); // 2x3 * 2x3
    EXPECT_NO_THROW(a * b);
}

TEST(DynamicMatrixTest, CopyConstructedMatrixHasTheSameValuesAndIsIndependent)
{
    DynamicMatrix<double> original{2, 2};
    fill(original, 2, 2, {1, 2,
                          3, 4});

    DynamicMatrix<double> copy{original};
    EXPECT_EQ(copy.resolution(), original.resolution());
    expectEqual(copy, 2, 2, {1, 2,
                             3, 4});

    copy(0, 0) = 100;
    EXPECT_DOUBLE_EQ(original(0, 0), 1) << "copy shares storage with the original";
    EXPECT_DOUBLE_EQ(copy(0, 0), 100);
}

TEST(DynamicMatrixTest, CopyAssignedMatrixHasTheSameValuesAndIsIndependent)
{
    DynamicMatrix<double> original{2, 2};
    DynamicMatrix<double> target{2, 2};
    fill(original, 2, 2, {1, 2,
                          3, 4});

    target = original;
    expectEqual(target, 2, 2, {1, 2,
                               3, 4});

    target(1, 1) = 100;
    EXPECT_DOUBLE_EQ(original(1, 1), 4) << "copy shares storage with the original";
}

TEST(DynamicMatrixTest, MovedMatrixKeepsValuesAndStaysUsable)
{
    DynamicMatrix<double> source{2, 2};
    fill(source, 2, 2, {1, 2,
                        3, 4});

    DynamicMatrix<double> moved{std::move(source)};
    EXPECT_EQ(moved.resolution(), std::make_pair(size_t{2}, size_t{2}));
    expectEqual(moved, 2, 2, {1, 2,
                              3, 4});

    // access goes through the base class pointer, it must point into `moved`'s storage
    moved(1, 0) = 20;
    EXPECT_DOUBLE_EQ(moved.get(1), 20);
}

TEST(DynamicMatrixTest, ResizingCopyTargetTakesTheSourceDimensions)
{
    DynamicMatrix<double> small{1, 1};
    DynamicMatrix<double> big{2, 3};
    fill(big, 2, 3, {1, 2, 3,
                     4, 5, 6});

    small = big;

    EXPECT_EQ(small.resolution(), std::make_pair(size_t{2}, size_t{3}));
    expectEqual(small, 2, 3, {1, 2, 3,
                              4, 5, 6});
}

TEST(DynamicMatrixTest, TransposeSwapsRowsAndColumns)
{
    DynamicMatrix<double> a{2, 3};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    const auto t = a.transpose();

    EXPECT_EQ(t.resolution(), std::make_pair(size_t{3}, size_t{2}));
    expectEqual(t, 3, 2, {1, 4,
                          2, 5,
                          3, 6});
}

TEST(DynamicMatrixTest, TransposeOfSquareMatrix)
{
    DynamicMatrix<double> a{3, 3};
    fill(a, 3, 3, {1, 2, 3,
                   4, 5, 6,
                   7, 8, 9});

    expectEqual(a.transpose(), 3, 3, {1, 4, 7,
                                      2, 5, 8,
                                      3, 6, 9});
}

TEST(DynamicMatrixTest, TransposeDoesNotModifyTheOriginal)
{
    DynamicMatrix<double> a{2, 3};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    auto t = a.transpose();
    t(0, 0) = 100;

    EXPECT_EQ(a.resolution(), std::make_pair(size_t{2}, size_t{3}));
    expectEqual(a, 2, 3, {1, 2, 3,
                          4, 5, 6});
}

TEST(DynamicMatrixTest, TransposingTwiceGivesTheOriginal)
{
    DynamicMatrix<double> a{2, 3};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    const auto t = a.transpose().transpose();

    EXPECT_EQ(t.resolution(), a.resolution());
    expectEqual(t, 2, 3, {1, 2, 3,
                          4, 5, 6});
}

TEST(DynamicMatrixTest, TransposeOfProductIsProductOfTransposesInReverseOrder)
{
    DynamicMatrix<double> a{2, 3};
    DynamicMatrix<double> b{3, 2};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});
    fill(b, 3, 2, {7, 8,
                   9, 10,
                   11, 12});

    expectEqual((a * b).transpose(), 2, 2, {58, 139,
                                            64, 154});
    expectEqual(b.transpose() * a.transpose(), 2, 2, {58, 139,
                                                      64, 154});
}

TEST(DynamicMatrixTest, TransposeOfRowVectorIsColumnVector)
{
    DynamicMatrix<double> row{1, 4};
    fill(row, 1, 4, {1, 2, 3, 4});

    const auto column = row.transpose();

    EXPECT_EQ(column.resolution(), std::make_pair(size_t{4}, size_t{1}));
    expectEqual(column, 4, 1, {1, 2, 3, 4});
}

TEST(DynamicMatrixTest, IdentityHasOnesOnTheDiagonalAndZerosElsewhere)
{
    const auto identity = DynamicMatrix<double>::identity(3);

    EXPECT_EQ(identity.resolution(), std::make_pair(size_t{3}, size_t{3}));
    expectEqual(identity, 3, 3, {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1});
}

TEST(DynamicMatrixTest, IdentityOfSizeZeroIsEmpty)
{
    const auto identity = DynamicMatrix<double>::identity(0);

    EXPECT_EQ(identity.resolution(), std::make_pair(size_t{0}, size_t{0}));
    EXPECT_EQ(identity.size(), 0u);
}

TEST(DynamicMatrixTest, MultiplyingByIdentityFromEitherSideKeepsTheMatrix)
{
    DynamicMatrix<double> a{2, 3};
    fill(a, 2, 3, {1, 2, 3,
                   4, 5, 6});

    expectEqual(DynamicMatrix<double>::identity(2) * a, 2, 3, {1, 2, 3,
                                                                4, 5, 6});
    expectEqual(a * DynamicMatrix<double>::identity(3), 2, 3, {1, 2, 3,
                                                                4, 5, 6});
}

TEST(DynamicMatrixTest, MultiplyingByIdentityOfWrongSizeThrows)
{
    DynamicMatrix<double> a{2, 3};

    EXPECT_THROW(a * DynamicMatrix<double>::identity(2), std::runtime_error);
}

TEST(DynamicMatrixTest, IdentityIsItsOwnTranspose)
{
    expectEqual(DynamicMatrix<double>::identity(3).transpose(), 3, 3, {1, 0, 0,
                                                                        0, 1, 0,
                                                                        0, 0, 1});
}

} // namespace lidar_viewer::tests::units

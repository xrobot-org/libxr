#pragma once

#ifndef LIBXR_NO_EIGEN

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

template <typename Scalar = DefaultScalar>
class Position;
template <typename Scalar = DefaultScalar>
class Axis;
template <typename Scalar = DefaultScalar>
class RotationMatrix;
template <typename Scalar = DefaultScalar>
class EulerAngle;
template <typename Scalar = DefaultScalar>
class Quaternion;

/**
 * @class Position
 * @brief 三维空间中的位置向量 / 3D position vector
 *
 * 该类基于 Eigen::Matrix<Scalar, 3, 1>，用于表示三维坐标中的位置，并支持基本的
 * 旋转、缩放和转换运算。
 * This class extends Eigen::Matrix<Scalar, 3, 1> to represent a position in 3D space,
 * supporting basic rotation, scaling, and transformation operations.
 *
 * @tparam Scalar 数值类型（默认使用 DefaultScalar） / Numeric type (default:
 * DefaultScalar)
 */
template <typename Scalar>
class Position : public Eigen::Matrix<Scalar, 3, 1>
{
 public:
  /**
   * @brief 默认构造函数，初始化为 (0,0,0) / Default constructor initializing to
   * (0,0,0)
   */
  Position() : Eigen::Matrix<Scalar, 3, 1>(0, 0, 0) {}

  /**
   * @brief 构造函数，指定 x, y, z 坐标 / Constructor specifying x, y, z
   * coordinates
   * @param x X 坐标 / X coordinate
   * @param y Y 坐标 / Y coordinate
   * @param z Z 坐标 / Z coordinate
   */
  Position(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<Scalar, 3, 1>(x, y, z) {}

  /**
   * @brief 从 Eigen 3x1 向量构造 / Constructs from an Eigen 3x1 vector
   * @param p Eigen 3x1 向量 / Eigen 3x1 vector
   */
  Position(const Eigen::Matrix<Scalar, 3, 1>& p) : Eigen::Matrix<Scalar, 3, 1>(p) {}

  /**
   * @brief 复制构造函数 / Copy constructor
   * @param p 另一个 Position 实例 / Another Position instance
   */
  Position(const Position& p) : Eigen::Matrix<Scalar, 3, 1>(p) {}

  /**
   * @brief 从 3 元素数组构造 / Constructs from a 3-element array
   * @tparam T 元素类型，可为 Scalar、float 或 double / Element type: Scalar, float or
   * double
   * @param data 存储 (x, y, z) 的数组 / Array storing (x, y, z)
   */
  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  Position(const T (&data)[3]) : Eigen::Matrix<Scalar, 3, 1>(data)
  {
  }

  /**
   * @brief 赋值运算符，将 Eigen 向量赋值给 Position / Assignment operator to assign an
   * Eigen vector to Position
   * @param p Eigen 3x1 向量 / Eigen 3x1 vector
   * @return 赋值后的 Position / Updated Position object
   */
  Position& operator=(const Eigen::Matrix<Scalar, 3, 1>& p)
  {
    this->data()[0] = p[0];
    this->data()[1] = p[1];
    this->data()[2] = p[2];
    return *this;
  }

  /**
   * @brief 拷贝赋值运算符 / Copy assignment operator
   * @param p 另一个 Position / Another Position object
   * @return 赋值后的 Position / Updated Position object
   */
  Position& operator=(const Position& p)
  {
    if (this != &p)
    {
      this->data()[0] = p[0];
      this->data()[1] = p[1];
      this->data()[2] = p[2];
    }

    return *this;
  }

  /**
   * @brief 返回旋转作用于当前向量的结果 R * p，不修改当前对象 / Returns the rotated
   * vector R * p without modifying this object
   * @tparam Rotation 旋转类型：RotationMatrix、Eigen 3x3 矩阵、Quaternion 或 Eigen
   * 四元数 / Rotation type: RotationMatrix, Eigen 3x3 matrix, Quaternion or Eigen
   * quaternion
   * @param R 旋转 / Rotation
   * @return 旋转后的向量 / Rotated vector
   */
  template <
      typename Rotation,
      std::enable_if_t<std::is_same<Rotation, RotationMatrix<Scalar>>::value ||
                           std::is_same<Rotation, Eigen::Matrix<Scalar, 3, 3>>::value ||
                           std::is_same<Rotation, Quaternion<Scalar>>::value ||
                           std::is_same<Rotation, Eigen::Quaternion<Scalar>>::value,
                       int> = 0>
  Eigen::Matrix<Scalar, 3, 1> operator*(const Rotation& R) const
  {
    return R * (*this);
  }

  /**
   * @brief 旋转并更新当前向量 / Rotate and update the current vector
   * @tparam Rotation 旋转类型，可为旋转矩阵、四元数等 / Rotation type (can be a rotation
   * matrix, quaternion, etc.)
   * @param R 旋转矩阵或四元数 / Rotation matrix or quaternion
   * @return 更新后的 Position / Updated Position object
   */
  template <
      typename Rotation,
      std::enable_if_t<std::is_same<Rotation, RotationMatrix<Scalar>>::value ||
                           std::is_same<Rotation, Eigen::Matrix<Scalar, 3, 3>>::value ||
                           std::is_same<Rotation, Quaternion<Scalar>>::value ||
                           std::is_same<Rotation, Eigen::Quaternion<Scalar>>::value,
                       int> = 0>
  const Position& operator*=(const Rotation& R)
  {
    *this = R * (*this);
    return *this;
  }

  /**
   * @brief 旋转矩阵的逆变换 / Inverse transformation using a rotation matrix
   * @param R 旋转矩阵 / Rotation matrix
   * @return 变换后的向量 / Transformed vector
   */
  Eigen::Matrix<Scalar, 3, 1> operator/(const RotationMatrix<Scalar>& R) const
  {
    return (-R) * (*this);
  }

  /**
   * @brief 逆四元数旋转并更新当前向量 / Apply inverse quaternion rotation and update the
   * current vector
   * @param q 四元数 / Quaternion
   * @return 更新后的 Position / Updated Position object
   */
  const Position& operator/=(const Quaternion<Scalar>& q)
  {
    *this = (-q) * (*this);
    return *this;
  }

  /**
   * @brief 逆旋转矩阵变换并更新当前向量 / Apply inverse rotation matrix transformation
   * and update the current vector
   * @param R 旋转矩阵 / Rotation matrix
   * @return 更新后的 Position / Updated Position object
   */
  const Position& operator/=(const Eigen::Matrix<Scalar, 3, 3>& R)
  {
    *this = R.transpose() * Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>>(this->data());
    return *this;
  }

  /**
   * @brief 逆四元数变换并更新当前向量 / Apply inverse quaternion transformation and
   * update the current vector
   * @param q 四元数 / Quaternion
   * @return 更新后的 Position / Updated Position object
   */
  const Position& operator/=(const Eigen::Quaternion<Scalar>& q)
  {
    *this = q.conjugate() * Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>>(this->data());
    return *this;
  }

  /**
   * @brief 按标量缩放当前向量 / Scale the current vector by a scalar
   * @param s 缩放因子 / Scaling factor
   * @return 更新后的 Position / Updated Position object
   */
  const Position& operator*=(Scalar s)
  {
    *this = s * (*this);
    return *this;
  }

  /**
   * @brief 按标量除法缩放当前向量 / Scale the current vector by dividing with a scalar
   * @param s 缩放因子 / Scaling factor
   * @return 更新后的 Position / Updated Position object
   */
  const Position& operator/=(Scalar s)
  {
    (*this)[0] /= s;
    (*this)[1] /= s;
    (*this)[2] /= s;
    return *this;
  }

  /**
   * @brief 计算从另一个 Position 到当前 Position 的旋转四元数 / Compute the quaternion
   * rotation from another Position to the current Position
   * @param p 参考 Position / Reference Position
   * @return 旋转四元数 / Quaternion rotation
   */
  Eigen::Quaternion<Scalar> operator/(const Position<>& p)
  {
    return Eigen::Quaternion<Scalar>::FromTwoVectors(p, *this);
  }
};

/**
 * @brief 三维坐标轴类，继承自 Eigen::Matrix<Scalar, 3, 1>。
 *        A 3D axis class, inheriting from Eigen::Matrix<Scalar, 3, 1>.
 *
 * 该类用于表示三维空间中的单位向量或方向向量，
 * 并提供了一些常见的轴向定义（X、Y、Z）。
 * This class represents unit vectors or directional vectors in 3D space
 * and provides common axis definitions (X, Y, Z).
 *
 * @tparam Scalar 数据类型，如 `float` 或 `double`。
 *                The data type, such as `float` or `double`.
 */
template <typename Scalar>
class Axis : public Eigen::Matrix<Scalar, 3, 1>
{
 public:
  /**
   * @brief 默认构造函数，将向量初始化为 (0,0,0)。
   *        Default constructor initializing the vector to (0,0,0).
   */
  Axis() : Eigen::Matrix<Scalar, 3, 1>(0, 0, 0) {}

  /**
   * @brief 通过 (x, y, z) 坐标值构造轴向量。
   *        Constructs an axis vector using (x, y, z) coordinates.
   *
   * @param x X 轴分量 / The X-axis component.
   * @param y Y 轴分量 / The Y-axis component.
   * @param z Z 轴分量 / The Z-axis component.
   */
  Axis(Scalar x, Scalar y, Scalar z) : Eigen::Matrix<Scalar, 3, 1>(x, y, z) {}

  /**
   * @brief 拷贝构造函数，复制另一个 `Axis` 对象。
   *        Copy constructor to duplicate another `Axis` object.
   *
   * @param p 要复制的 `Axis` 对象 / The `Axis` object to be copied.
   */
  Axis(const Axis& p) : Eigen::Matrix<Scalar, 3, 1>(p) {}

  /**
   * @brief 返回 X 轴单位向量 (1,0,0)。
   *        Returns the unit vector along the X-axis (1,0,0).
   *
   * @return X 轴单位向量 / The unit vector along the X-axis.
   */
  static Axis X() { return Axis(1., 0., 0.); }

  /**
   * @brief 返回 Y 轴单位向量 (0,1,0)。
   *        Returns the unit vector along the Y-axis (0,1,0).
   *
   * @return Y 轴单位向量 / The unit vector along the Y-axis.
   */
  static Axis Y() { return Axis(0., 1., 0.); }

  /**
   * @brief 返回 Z 轴单位向量 (0,0,1)。
   *        Returns the unit vector along the Z-axis (0,0,1).
   *
   * @return Z 轴单位向量 / The unit vector along the Z-axis.
   */
  static Axis Z() { return Axis(0., 0., 1.); }

  /**
   * @brief 赋值运算符重载，将 `Eigen::Matrix<Scalar, 3, 1>` 赋值给 `Axis` 对象。
   *        Overloaded assignment operator to assign an `Eigen::Matrix<Scalar, 3, 1>` to
   * an `Axis` object.
   *
   * 该操作直接拷贝 `Eigen` 矩阵的数据到 `Axis`，保证对象数据一致性。
   * This operation directly copies the data from an `Eigen` matrix to `Axis`, ensuring
   * data consistency.
   *
   * @param p 赋值的 `Eigen::Matrix<Scalar, 3, 1>` 矩阵。
   *          The `Eigen::Matrix<Scalar, 3, 1>` matrix to be assigned.
   * @return 返回赋值后的 `Axis` 对象引用。
   *         Returns a reference to the assigned `Axis` object.
   */
  Axis<Scalar>& operator=(const Eigen::Matrix<Scalar, 3, 1>& p)
  {
    this->data()[0] = p[0];
    this->data()[1] = p[1];
    this->data()[2] = p[2];
    return *this;
  }

  /**
   * @brief 赋值运算符重载，将另一个 `Axis` 对象赋值给当前对象。
   *        Overloaded assignment operator to assign another `Axis` object to this object.
   *
   * 该操作检查自赋值，并确保数据正确拷贝。
   * This operation checks for self-assignment and ensures correct data copying.
   *
   * @param p 赋值的 `Axis` 对象 / The `Axis` object to be assigned.
   * @return 返回赋值后的 `Axis` 对象引用。
   *         Returns a reference to the assigned `Axis` object.
   */
  Axis<Scalar>& operator=(const Axis& p)
  {
    if (this != &p)
    {
      this->data()[0] = p[0];
      this->data()[1] = p[1];
      this->data()[2] = p[2];
    }
    return *this;
  }
};

/**
 * @class EulerAngle
 * @brief 表示欧拉角的类，用于描述3D旋转。
 *        Class representing Euler angles for 3D rotation.
 *
 * 三个分量依次为 roll（绕 X 轴）、pitch（绕 Y 轴）和 yaw（绕 Z 轴），单位为弧度。
 * `ToRotationMatrixABC()` 返回 R_A·R_B·R_C，其中 R_X、R_Y、R_Z 是绕对应坐标轴分别转过
 * roll、pitch、yaw 的基本旋转矩阵，例如 `ToRotationMatrixXYZ()` 等于
 * Rx(roll)·Ry(pitch)·Rz(yaw)。作用于列向量时，这相当于先绕 A 轴、再绕转动后的 B 轴、
 * 最后绕转动后的 C 轴的内旋，也相当于依次绕固定的 C、B、A 轴的外旋。默认顺序 ZYX 即
 * 先 yaw、再 pitch、最后 roll 的内旋。`ToQuaternionABC()` 表示同一个旋转；
 * `RotationMatrix` 和 `Quaternion` 的 `ToEulerAngleABC()` 按同一约定求回三个角。
 *
 * The three components are roll (about X), pitch (about Y) and yaw (about Z), in
 * radians. `ToRotationMatrixABC()` returns R_A·R_B·R_C, where R_X, R_Y and R_Z are the
 * elementary rotations by roll, pitch and yaw about the corresponding axes; for example
 * `ToRotationMatrixXYZ()` equals Rx(roll)·Ry(pitch)·Rz(yaw). Applied to column vectors,
 * this is an intrinsic rotation about A, then the rotated B, then the rotated C, which
 * is the same as an extrinsic rotation about the fixed axes C, B and A in that order.
 * The default order ZYX is the intrinsic yaw, pitch, roll sequence. `ToQuaternionABC()`
 * represents the same rotation, and `ToEulerAngleABC()` of `RotationMatrix` and
 * `Quaternion` recovers the angles under the same convention.
 *
 * @tparam Scalar 数据类型，如 float 或 double / Data type, such as float or double.
 */
template <typename Scalar>
class EulerAngle
{
 public:
  Scalar data_[3];  ///< 存储欧拉角的数组 / Array storing Euler angles.

  /// @brief 默认构造函数，初始化所有角度为零。
  /// Default constructor initializing all angles to zero.
  EulerAngle() : data_{0, 0, 0} {}

  /**
   * @brief 使用指定角度构造欧拉角对象。
   * Constructs an Euler angle object with given angles.
   * @param roll 绕 X 轴的角度 / Angle about the X-axis.
   * @param pitch 绕 Y 轴的角度 / Angle about the Y-axis.
   * @param yaw 绕 Z 轴的角度 / Angle about the Z-axis.
   */
  EulerAngle(Scalar roll, Scalar pitch, Scalar yaw) : data_{roll, pitch, yaw} {}

  /**
   * @brief 通过 Eigen 3D 向量构造欧拉角对象。
   * Constructs an Euler angle object using an Eigen 3D vector.
   * @param p 含有 (roll, pitch, yaw) 的向量 / Vector containing (roll, pitch, yaw).
   */
  EulerAngle(const Eigen::Matrix<Scalar, 3, 1>& p) : data_{p.x(), p.y(), p.z()} {}

  /**
   * @brief 拷贝构造函数 / Copy constructor.
   * @param p 另一个 EulerAngle 对象 / Another EulerAngle object.
   */
  EulerAngle(const EulerAngle& p) : data_{p.data_[0], p.data_[1], p.data_[2]} {}

  Scalar& Roll() noexcept { return data_[0]; }
  const Scalar& Roll() const noexcept { return data_[0]; }
  Scalar& Pitch() noexcept { return data_[1]; }
  const Scalar& Pitch() const noexcept { return data_[1]; }
  Scalar& Yaw() noexcept { return data_[2]; }
  const Scalar& Yaw() const noexcept { return data_[2]; }

  /**
   * @brief 通过 3 元素数组构造欧拉角对象。
   * Constructs an Euler angle object using a 3-element array.
   * @tparam T 数据类型，支持 Scalar、float 或 double。
   * Data type, supporting Scalar, float, or double.
   * @param data 存储 (roll, pitch, yaw) 的数组 / Array storing (roll, pitch, yaw).
   */
  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  EulerAngle(const T (&data)[3]) : data_{data[0], data[1], data[2]}
  {
  }

  /// @brief 转换为 Eigen 3D 向量 / Converts to an Eigen 3D vector.
  operator Eigen::Matrix<Scalar, 3, 1>() const
  {
    return Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>>(data_);
  }

  /// @brief 获取欧拉角的某个分量 / Retrieves a specific Euler angle component.
  Scalar operator()(int i) const { return data_[i]; }

  /// @brief 赋值运算符，从 Eigen 3D 向量赋值。
  /// Assignment operator from an Eigen 3D vector.
  EulerAngle& operator=(const Eigen::Matrix<Scalar, 3, 1>& p)
  {
    data_[0] = p(0);
    data_[1] = p(1);
    data_[2] = p(2);
    return *this;
  }

  /// @brief 赋值运算符，从另一个 EulerAngle 赋值。
  /// Assignment operator from another EulerAngle.
  EulerAngle& operator=(const EulerAngle& p)
  {
    if (this != &p)
    {
      data_[0] = p.data_[0];
      data_[1] = p.data_[1];
      data_[2] = p.data_[2];
    }
    return *this;
  }

  /// @brief 转换为旋转矩阵，默认使用 ZYX 顺序。
  /// Converts to a rotation matrix using the ZYX order by default.
  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrix() const { return ToRotationMatrixZYX(); }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixZYX() const
  {
    Scalar ca = std::cos(Yaw()), cb = std::cos(Pitch()), cc = std::cos(Roll());
    Scalar sa = std::sin(Yaw()), sb = std::sin(Pitch()), sc = std::sin(Roll());

    return (Eigen::Matrix<Scalar, 3, 3>() << ca * cb, ca * sb * sc - cc * sa,
            sa * sc + ca * cc * sb, cb * sa, ca * cc + sa * sb * sc,
            cc * sa * sb - ca * sc, -sb, cb * sc, cb * cc)
        .finished();
  }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixZXY() const
  {
    Scalar ca = std::cos(Yaw()), cb = std::cos(Roll()), cc = std::cos(Pitch());
    Scalar sa = std::sin(Yaw()), sb = std::sin(Roll()), sc = std::sin(Pitch());

    return (Eigen::Matrix<Scalar, 3, 3>() << ca * cc - sa * sb * sc, -cb * sa,
            ca * sc + cc * sa * sb, cc * sa + ca * sb * sc, ca * cb,
            sa * sc - ca * cc * sb, -cb * sc, sb, cb * cc)
        .finished();
  }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixYXZ() const
  {
    Scalar ca = std::cos(Pitch()), cb = std::cos(Roll()), cc = std::cos(Yaw());
    Scalar sa = std::sin(Pitch()), sb = std::sin(Roll()), sc = std::sin(Yaw());

    return (Eigen::Matrix<Scalar, 3, 3>() << ca * cc + sa * sb * sc,
            cc * sa * sb - ca * sc, cb * sa, cb * sc, cb * cc, -sb,
            ca * sb * sc - cc * sa, sa * sc + ca * cc * sb, ca * cb)
        .finished();
  }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixYZX() const
  {
    Scalar ca = std::cos(Pitch()), cb = std::cos(Yaw()), cc = std::cos(Roll());
    Scalar sa = std::sin(Pitch()), sb = std::sin(Yaw()), sc = std::sin(Roll());

    return (Eigen::Matrix<Scalar, 3, 3>() << ca * cb, sa * sc - ca * cc * sb,
            cc * sa + ca * sb * sc, sb, cb * cc, -cb * sc, -cb * sa,
            ca * sc + cc * sa * sb, ca * cc - sa * sb * sc)
        .finished();
  }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixXYZ() const
  {
    Scalar ca = std::cos(Roll()), cb = std::cos(Pitch()), cc = std::cos(Yaw());
    Scalar sa = std::sin(Roll()), sb = std::sin(Pitch()), sc = std::sin(Yaw());

    return (Eigen::Matrix<Scalar, 3, 3>() << cb * cc, -cb * sc, sb,
            ca * sc + cc * sa * sb, ca * cc - sa * sb * sc, -cb * sa,
            sa * sc - ca * cc * sb, cc * sa + ca * sb * sc, ca * cb)
        .finished();
  }

  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrixXZY() const
  {
    Scalar ca = std::cos(Roll()), cb = std::cos(Yaw()), cc = std::cos(Pitch());
    Scalar sa = std::sin(Roll()), sb = std::sin(Yaw()), sc = std::sin(Pitch());

    return (Eigen::Matrix<Scalar, 3, 3>() << cb * cc, -sb, cb * sc,
            sa * sc + ca * cc * sb, ca * cb, ca * sb * sc - cc * sa,
            cc * sa * sb - ca * sc, cb * sa, ca * cc + sa * sb * sc)
        .finished();
  }

  /// @brief 转换为四元数，默认使用 ZYX 顺序。
  /// Converts to a quaternion using the ZYX order by default.
  Eigen::Quaternion<Scalar> ToQuaternion() const { return ToQuaternionZYX(); }

#if 0
  Eigen::Quaternion<Scalar> ToQuaternionXYZ() const {
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    return rollAngle * pitchAngle * yawAngle;
  }

  Eigen::Quaternion<Scalar> ToQuaternionXZY() const {
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    return rollAngle * yawAngle * pitchAngle;
  }

  Eigen::Quaternion<Scalar> ToQuaternionYXZ() const {
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    return pitchAngle * rollAngle * yawAngle;
  }

  Eigen::Quaternion<Scalar> ToQuaternionYZX() const {
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    return pitchAngle * yawAngle * rollAngle;
  }

  Eigen::Quaternion<Scalar> ToQuaternionZXY() const {
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    return yawAngle * rollAngle * pitchAngle;
  }

  Eigen::Quaternion<Scalar> ToQuaternionZYX() const {
    Eigen::AngleAxisd yawAngle(Yaw(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(Pitch(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(Roll(), Eigen::Vector3d::UnitX());
    return yawAngle * pitchAngle * rollAngle;
  }
#endif

  /// @brief 按 XYZ 顺序转换为四元数。
  /// Converts to a quaternion using the XYZ order.
  Eigen::Quaternion<Scalar> ToQuaternionXYZ() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixXYZ());
  }

  Eigen::Quaternion<Scalar> ToQuaternionXZY() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixXZY());
  }

  Eigen::Quaternion<Scalar> ToQuaternionYXZ() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixYXZ());
  }

  Eigen::Quaternion<Scalar> ToQuaternionYZX() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixYZX());
  }

  Eigen::Quaternion<Scalar> ToQuaternionZXY() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixZXY());
  }

  Eigen::Quaternion<Scalar> ToQuaternionZYX() const
  {
    return Eigen::Quaternion<Scalar>(ToRotationMatrixZYX());
  }
};

/**
 * @brief 旋转矩阵类，继承自 Eigen::Matrix<Scalar, 3, 3>。
 *        Rotation matrix class, inheriting from Eigen::Matrix<Scalar, 3, 3>.
 *
 * 该类提供 3x3 旋转矩阵的构造、赋值、矩阵运算以及欧拉角转换等功能，
 * 并支持从四元数构造旋转矩阵。
 * This class provides functionalities for constructing, assigning,
 * operating on rotation matrices, and converting to Euler angles.
 * It also supports constructing rotation matrices from quaternions.
 *
 * @tparam Scalar 旋转矩阵元素的数据类型，如 `float` 或 `double`。
 *                The data type of the rotation matrix elements, such as `float` or
 * `double`.
 */
template <typename Scalar>
class RotationMatrix : public Eigen::Matrix<Scalar, 3, 3>
{
 public:
  /**
   * @brief 默认构造函数，初始化单位旋转矩阵。
   *        Default constructor initializing an identity rotation matrix.
   */
  RotationMatrix() : Eigen::Matrix<Scalar, 3, 3>()
  {
    (*this) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  }

  /**
   * @brief 通过 9 个矩阵元素的值构造旋转矩阵。
   *        Constructs a rotation matrix using 9 matrix elements.
   *
   * @param r00 第 0 行第 0 列元素 / Element at row 0, column 0
   * @param r01 第 0 行第 1 列元素 / Element at row 0, column 1
   * @param r02 第 0 行第 2 列元素 / Element at row 0, column 2
   * @param r10 第 1 行第 0 列元素 / Element at row 1, column 0
   * @param r11 第 1 行第 1 列元素 / Element at row 1, column 1
   * @param r12 第 1 行第 2 列元素 / Element at row 1, column 2
   * @param r20 第 2 行第 0 列元素 / Element at row 2, column 0
   * @param r21 第 2 行第 1 列元素 / Element at row 2, column 1
   * @param r22 第 2 行第 2 列元素 / Element at row 2, column 2
   */
  RotationMatrix(Scalar r00, Scalar r01, Scalar r02, Scalar r10, Scalar r11, Scalar r12,
                 Scalar r20, Scalar r21, Scalar r22)
      : Eigen::Matrix<Scalar, 3, 3>()
  {
    (*this) << r00, r01, r02, r10, r11, r12, r20, r21, r22;
  }

  /**
   * @brief 通过 Eigen 3x3 矩阵构造旋转矩阵。
   *        Constructs a rotation matrix from an Eigen 3x3 matrix.
   *
   * @param R 3x3 旋转矩阵 / The 3x3 rotation matrix.
   */
  RotationMatrix(const Eigen::Matrix<Scalar, 3, 3>& R) : Eigen::Matrix<Scalar, 3, 3>{R} {}

  /**
   * @brief 通过 Eigen 四元数构造旋转矩阵。
   *        Constructs a rotation matrix from an Eigen quaternion.
   *
   * @param q Eigen 四元数 / The Eigen quaternion.
   */
  RotationMatrix(const Eigen::Quaternion<Scalar>& q)
      : Eigen::Matrix<Scalar, 3, 3>{q.toRotationMatrix()}
  {
  }

  /**
   * @brief 通过 `Quaternion` 四元数构造旋转矩阵。
   *        Constructs a rotation matrix from a `Quaternion` object.
   *
   * @param q `Quaternion` 四元数 / The `Quaternion` object.
   */
  RotationMatrix(const Quaternion<Scalar>& q) { *this = q.ToRotationMatrix(); }

  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  RotationMatrix(const T (&data)[9]) : Eigen::Matrix<Scalar, 3, 3>()
  {
    (*this) << data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
        data[8];
  }

  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  RotationMatrix(const T (&data)[3][3]) : Eigen::Matrix<Scalar, 3, 3>()
  {
    (*this) << data[0][0], data[0][1], data[0][2], data[1][0], data[1][1], data[1][2],
        data[2][0], data[2][1], data[2][2];
  }

  /**
   * @brief 计算旋转矩阵的转置（逆矩阵）。
   *        Computes the transpose (inverse) of the rotation matrix.
   *
   * @return 旋转矩阵的转置 / The transposed rotation matrix.
   */
  Eigen::Matrix<Scalar, 3, 3> operator-() const { return this->transpose(); }

  /**
   * @brief 赋值运算符，将 `RotationMatrix` 赋值给当前对象。
   *        Overloaded assignment operator to assign a `RotationMatrix` to the current
   * object.
   *
   * @param R 需要赋值的旋转矩阵 / The rotation matrix to be assigned.
   * @return 返回赋值后的 `RotationMatrix` 对象引用。
   *         Returns a reference to the assigned `RotationMatrix` object.
   */
  RotationMatrix& operator=(const RotationMatrix& R)
  {
    if (this != &R)
    {
      this->data()[0] = R.data()[0];
      this->data()[1] = R.data()[1];
      this->data()[2] = R.data()[2];
      this->data()[3] = R.data()[3];
      this->data()[4] = R.data()[4];
      this->data()[5] = R.data()[5];
      this->data()[6] = R.data()[6];
      this->data()[7] = R.data()[7];
      this->data()[8] = R.data()[8];
    }
    return *this;
  }

  RotationMatrix& operator=(const Eigen::Matrix<Scalar, 3, 3>& R)
  {
    this->data()[0] = R.data()[0];
    this->data()[1] = R.data()[1];
    this->data()[2] = R.data()[2];
    this->data()[3] = R.data()[3];
    this->data()[4] = R.data()[4];
    this->data()[5] = R.data()[5];
    this->data()[6] = R.data()[6];
    this->data()[7] = R.data()[7];
    this->data()[8] = R.data()[8];
    return *this;
  }

  RotationMatrix& operator=(const Eigen::Quaternion<Scalar>& q)
  {
    *this = q.toRotationMatrix();
    return *this;
  }

  RotationMatrix& operator=(const Quaternion<Scalar>& q)
  {
    *this = q.ToRotationMatrix();
    return *this;
  }

  Position<Scalar> operator*(const Position<Scalar>& p) const
  {
    return Position<Scalar>((*this) * Eigen::Matrix<Scalar, 3, 1>(p));
  }

  /**
   * @brief 计算旋转矩阵与三维向量的乘积。
   *        Computes the product of the rotation matrix and a 3D vector.
   *
   * @param p 输入的三维位置向量 / The input 3D position vector.
   * @return 旋转后的三维向量 / The rotated 3D vector.
   */
  Eigen::Matrix<Scalar, 3, 1> operator*(const Eigen::Matrix<Scalar, 3, 1>& p) const
  {
    return static_cast<Eigen::Matrix<Scalar, 3, 3>>(*this) * p;
  }

  /**
   * @brief 计算两个旋转矩阵的乘积。
   *        Computes the product of two rotation matrices.
   *
   * @param rhs 另一个旋转矩阵 / The second rotation matrix.
   * @return 两个旋转矩阵的乘积 / The product of the two rotation matrices.
   */
  RotationMatrix operator*(const RotationMatrix& rhs) const
  {
    const Eigen::Matrix<Scalar, 3, 3>& a =
        static_cast<const Eigen::Matrix<Scalar, 3, 3>&>(*this);
    const Eigen::Matrix<Scalar, 3, 3>& b =
        static_cast<const Eigen::Matrix<Scalar, 3, 3>&>(rhs);
    return RotationMatrix(a * b);
  }

  /**
   * @brief 将旋转矩阵转换为欧拉角（默认使用 ZYX 顺序）。
   *        Converts the rotation matrix to Euler angles (default ZYX order).
   *
   * 该方法调用 `ToEulerAngleZYX()` 方法，使用 ZYX（航向-俯仰-横滚）顺序转换欧拉角。
   * This method calls `ToEulerAngleZYX()`, converting to Euler angles
   * in the ZYX (yaw-pitch-roll) order.
   *
   * @return 三个欧拉角（roll, pitch, yaw） / The three Euler angles (roll, pitch, yaw).
   */
  Eigen::Matrix<Scalar, 3, 1> ToEulerAngle() const { return ToEulerAngleZYX(); }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleZYX() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar roll = std::atan2(r(2, 1), r(2, 2));
    Scalar yaw = std::atan2(r(1, 0), r(0, 0));
    Scalar pitch = std::asin(-r(2, 0));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleXZY() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar roll = std::atan2(r(2, 1), r(1, 1));
    Scalar yaw = std::asin(-r(0, 1));
    Scalar pitch = std::atan2(r(0, 2), r(0, 0));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleYZX() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar pitch = std::atan2(-r(2, 0), r(0, 0));
    Scalar yaw = std::asin(r(1, 0));
    Scalar roll = std::atan2(-r(1, 2), r(1, 1));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleYXZ() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar pitch = std::atan2(r(0, 2), r(2, 2));
    Scalar roll = std::asin(-r(1, 2));
    Scalar yaw = std::atan2(r(1, 0), r(1, 1));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleZXY() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar roll = std::asin(r(2, 1));
    Scalar yaw = std::atan2(-r(0, 1), r(1, 1));
    Scalar pitch = std::atan2(-r(2, 0), r(2, 2));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleXYZ() const
  {
    const Eigen::Matrix<Scalar, 3, 3>& r = (*this);

    Scalar yaw = std::atan2(-r(0, 1), r(0, 0));
    Scalar pitch = std::asin(r(0, 2));
    Scalar roll = std::atan2(-r(1, 2), r(2, 2));

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }
};

/**
 * @class Quaternion
 * @brief 四元数表示与运算，继承自 Eigen::Quaternion / Quaternion representation and
 * operations, inheriting from Eigen::Quaternion
 *
 * 该类提供了对四元数的基本运算，如加法、减法、乘法、除法，以及与旋转矩阵和欧拉角的转换等。
 * This class provides fundamental quaternion operations, such as addition, subtraction,
 * multiplication, and division, along with conversions to rotation matrices and Euler
 * angles.
 *
 * @tparam Scalar 数值类型（默认使用 DefaultScalar） / Numeric type (default:
 * DefaultScalar)
 */
template <typename Scalar>
class Quaternion : public Eigen::Quaternion<Scalar>
{
 public:
  /**
   * @brief 默认构造函数，初始化为单位四元数 / Default constructor initializing to an
   * identity quaternion
   */
  Quaternion() : Eigen::Quaternion<Scalar>(1, 0, 0, 0) {}

  /**
   * @brief 通过四个分量初始化四元数 / Construct a quaternion using four components
   * @param w 实部 / Real part
   * @param x i 分量 / i component
   * @param y j 分量 / j component
   * @param z k 分量 / k component
   */
  Quaternion(Scalar w, Scalar x, Scalar y, Scalar z)
      : Eigen::Quaternion<Scalar>(w, x, y, z)
  {
  }

  /**
   * @brief 复制构造函数 / Copy constructor
   * @param q 另一个 Eigen::Quaternion 四元数 / Another Eigen::Quaternion
   */
  Quaternion(const Eigen::Quaternion<Scalar>& q) : Eigen::Quaternion<Scalar>(q) {}

  /**
   * @brief 通过旋转矩阵构造四元数 / Construct a quaternion from a rotation matrix
   * @param R 旋转矩阵 / Rotation matrix
   */
  Quaternion(const RotationMatrix<Scalar>& R)
      : Eigen::Quaternion<Scalar>(
            Eigen::Quaternion<Scalar>(static_cast<Eigen::Matrix<Scalar, 3, 3>>(R)))
  {
  }

  /**
   * @brief 通过 3x3 旋转矩阵构造四元数 / Construct a quaternion from a 3x3 rotation
   * matrix
   * @param R 旋转矩阵 / Rotation matrix
   */
  Quaternion(const Eigen::Matrix<Scalar, 3, 3> R)
      : Eigen::Quaternion<Scalar>(
            Eigen::Quaternion<Scalar>(static_cast<Eigen::Matrix<Scalar, 3, 3>>(R)))
  {
  }

  /**
   * @brief 通过四维数组初始化四元数 / Construct a quaternion from a 4-element array
   * @param data 4 维数组，表示 (w, x, y, z) / A 4-element array representing (w, x, y, z)
   */
  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  Quaternion(const T (&data)[4])
      : Eigen::Quaternion<Scalar>(
            static_cast<Scalar>(data[0]), static_cast<Scalar>(data[1]),
            static_cast<Scalar>(data[2]), static_cast<Scalar>(data[3]))
  {
  }

  /**
   * @brief 获取四元数的分量 / Retrieve a specific component of the quaternion
   * @param i 分量索引 (0: x, 1: y, 2: z, 3: w) / Component index (0: x, 1: y, 2: z, 3: w)
   * @return 该分量的值 / Value of the component
   */
  Scalar operator()(int i) const
  {
    switch (i)
    {
      case 3:
        return this->w();
      case 0:
        return this->x();
      case 1:
        return this->y();
      case 2:
        return this->z();
      default:
        return 0;
    }
  }

  /**
   * @brief 赋值运算符 / Assignment operator
   * @param q 另一个四元数 / Another quaternion
   * @return 赋值后的 Quaternion / Assigned Quaternion
   */
  Quaternion& operator=(const Eigen::Quaternion<Scalar>& q)
  {
    *this = Quaternion(q);
    return *this;
  }

  /**
   * @brief 取共轭四元数 / Get the conjugate quaternion
   * @return 共轭四元数 / Conjugated quaternion
   */
  Quaternion operator-() const { return Quaternion((*this).conjugate()); }

  /**
   * @brief 四元数加法 / Quaternion addition
   * @param q 另一个四元数 / Another quaternion
   * @return 计算后的四元数 / Computed quaternion
   */
  Quaternion operator+(const Quaternion& q) const
  {
    return Quaternion(this->w() + q.w(), this->x() + q.x(), this->y() + q.y(),
                      this->z() + q.z());
  }

  Quaternion operator+(const Eigen::Quaternion<Scalar>& q) const
  {
    return Quaternion(this->w() + q.w(), this->x() + q.x(), this->y() + q.y(),
                      this->z() + q.z());
  }

  Quaternion operator-(const Quaternion& q) const
  {
    return Quaternion(this->w() - q.w(), this->x() - q.x(), this->y() - q.y(),
                      this->z() - q.z());
  }

  Quaternion operator-(const Eigen::Quaternion<Scalar>& q) const
  {
    return Quaternion(this->w() - q.w(), this->x() - q.x(), this->y() - q.y(),
                      this->z() - q.z());
  }

  /**
   * @brief 四元数乘法 / Quaternion multiplication
   * @param q 另一个四元数 / Another quaternion
   * @return 计算后的四元数 / Computed quaternion
   */
  Quaternion operator*(const Quaternion& q) const
  {
    return Eigen::Quaternion<Scalar>(*this) * Eigen::Quaternion<Scalar>(q);
  }

  Quaternion operator*(const Eigen::Quaternion<Scalar>& q) const
  {
    return Eigen::Quaternion<Scalar>(*this) * q;
  }

  /**
   * @brief 四元数除法（即左乘其共轭） / Quaternion division (multiplication by its
   * conjugate)
   * @param q 另一个四元数 / Another quaternion
   * @return 计算后的四元数 / Computed quaternion
   */
  Quaternion operator/(const Quaternion& q) const { return (*this) * (-q); }

  Quaternion operator/(const Eigen::Quaternion<Scalar>& q) const
  {
    return Eigen::Quaternion<Scalar>(*this) * q.conjugate();
  }

  template <typename Q,
            std::enable_if_t<std::is_same<Q, Quaternion>::value ||
                                 std::is_same<Q, Eigen::Quaternion<Scalar>>::value,
                             int> = 0>
  const Quaternion& operator+=(const Q& q)
  {
    *this = *this + q;
    return *this;
  }

  template <typename Q,
            std::enable_if_t<std::is_same<Q, Quaternion>::value ||
                                 std::is_same<Q, Eigen::Quaternion<Scalar>>::value,
                             int> = 0>
  const Quaternion& operator-=(const Q& q)
  {
    *this = *this - q;
    return *this;
  }

  /**
   * @brief 旋转三维向量 / Rotate a 3D vector
   * @param p 需要旋转的 Position 向量 / Position vector to be rotated
   * @return 旋转后的 Position 向量 / Rotated Position vector
   */
  Position<Scalar> operator*(const Position<Scalar>& p) const
  {
    return Eigen::Quaternion<Scalar>(*this) * Eigen::Matrix<Scalar, 3, 1>(p);
  }

  /**
   * @brief 获取四元数的欧拉角表示（默认使用 ZYX 旋转顺序）
   *        Get the Euler angles representation of the quaternion (default ZYX order)
   *
   * 该方法将四元数转换为欧拉角，表示旋转顺序为 ZYX（即先绕 Z 轴旋转，再绕 Y 轴，最后绕 X
   * 轴）。
   * This method converts the quaternion into Euler angles using the ZYX rotation order
   * (first rotate around the Z-axis, then Y-axis, and finally the X-axis).
   *
   * @return 计算得到的欧拉角向量 (roll, pitch, yaw) / Computed Euler angles vector (roll,
   * pitch, yaw)
   */
  Eigen::Matrix<Scalar, 3, 1> ToEulerAngle() const { return ToEulerAngleZYX(); }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleYZX() const
  {
    Scalar roll = std::atan2(2 * this->w() * this->x() - 2 * this->y() * this->z(),
                             1 - 2 * (this->x() * this->x() + this->z() * this->z()));
    Scalar pitch = std::atan2(2 * this->w() * this->y() - 2 * this->x() * this->z(),
                              1 - 2 * (this->y() * this->y() + this->z() * this->z()));
    Scalar yaw = std::asin(2 * (this->w() * this->z() + this->x() * this->y()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleZYX() const
  {
    Scalar yaw = std::atan2(2 * (this->w() * this->z() + this->x() * this->y()),
                            1 - 2 * (this->z() * this->z() + this->y() * this->y()));
    Scalar pitch = std::asin(2 * (this->w() * this->y() - this->x() * this->z()));
    Scalar roll = std::atan2(2 * (this->w() * this->x() + this->y() * this->z()),
                             1 - 2 * (this->y() * this->y() + this->x() * this->x()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleYXZ() const
  {
    Scalar roll = std::asin(2 * (this->w() * this->x() - this->y() * this->z()));
    Scalar yaw = std::atan2(2 * (this->w() * this->z() + this->x() * this->y()),
                            1 - 2 * (this->z() * this->z() + this->x() * this->x()));
    Scalar pitch = std::atan2(2 * (this->x() * this->z() + this->w() * this->y()),
                              1 - 2 * (this->y() * this->y() + this->x() * this->x()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleZXY() const
  {
    Scalar pitch = std::atan2(-2 * (this->x() * this->z() - this->w() * this->y()),
                              1 - 2 * (this->y() * this->y() + this->x() * this->x()));
    Scalar roll = std::asin(2 * (this->w() * this->x() + this->y() * this->z()));
    Scalar yaw = std::atan2(-2 * (this->x() * this->y() - this->w() * this->z()),
                            1 - 2 * (this->z() * this->z() + this->x() * this->x()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleXZY() const
  {
    Scalar pitch = std::atan2(2 * (this->w() * this->y() + this->x() * this->z()),
                              1 - 2 * (this->z() * this->z() + this->y() * this->y()));
    Scalar yaw = std::asin(2 * (this->w() * this->z() - this->x() * this->y()));
    Scalar roll = std::atan2(2 * (this->w() * this->x() + this->y() * this->z()),
                             1 - 2 * (this->z() * this->z() + this->x() * this->x()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  Eigen::Matrix<Scalar, 3, 1> ToEulerAngleXYZ() const
  {
    Scalar yaw = std::atan2(-2 * (this->x() * this->y() - this->w() * this->z()),
                            1 - 2 * (this->z() * this->z() + this->y() * this->y()));
    Scalar pitch = std::asin(2 * (this->w() * this->y() + this->x() * this->z()));
    Scalar roll = std::atan2(-2 * (this->y() * this->z() - this->w() * this->x()),
                             1 - 2 * (this->y() * this->y() + this->x() * this->x()));
    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
  }

  /**
   * @brief 将四元数转换为 3x3 旋转矩阵 / Convert the quaternion to a 3x3 rotation matrix
   *
   * 该方法使用四元数生成等效的 3x3 旋转矩阵，矩阵可用于坐标变换、姿态计算等。
   * This method generates an equivalent 3x3 rotation matrix from the quaternion.
   * The resulting matrix can be used for coordinate transformations, attitude
   * calculations, etc.
   *
   * @return 计算得到的 3x3 旋转矩阵 / Computed 3x3 rotation matrix
   */
  Eigen::Matrix<Scalar, 3, 3> ToRotationMatrix() const
  {
    return Eigen::Matrix<Scalar, 3, 3>(*this);
  }
};

/**
 * @class Transform
 * @brief 表示三维空间中的刚体变换，包括旋转和位移。
 * Represents rigid body transformations in 3D space, including rotation and translation.
 * @tparam Scalar 数据类型，默认为 DefaultScalar / Data type, defaulting to DefaultScalar.
 */
template <typename Scalar = DefaultScalar>
class Transform
{
 public:
  Quaternion<Scalar> rotation;   ///< 旋转部分，使用四元数表示 / Rotation component
                                 ///< represented by a quaternion.
  Position<Scalar> translation;  ///< 平移部分，使用三维向量表示 / Translation component
                                 ///< represented by a 3D vector.

  /// @brief 默认构造函数，创建单位变换。
  /// Default constructor creating an identity transformation.
  Transform() = default;

  /**
   * @brief 使用给定的旋转和位移构造变换。
   * Constructs a transformation with the given rotation and translation.
   * @param rotation 旋转四元数 / Rotation quaternion.
   * @param translation 平移向量 / Translation vector.
   */
  Transform(const Quaternion<Scalar>& rotation, const Position<Scalar>& translation)
      : rotation(rotation), translation(translation)
  {
  }

  /**
   * @brief 赋值运算符，将旋转部分设置为给定的四元数。
   * Assignment operator setting the rotation component to the given quaternion.
   * @param q 旋转四元数 / Rotation quaternion.
   * @return 当前 Transform 对象的引用 / Reference to the current Transform object.
   */
  Transform& operator=(const Quaternion<Scalar>& q)
  {
    rotation = q;
    return *this;
  }

  /**
   * @brief 赋值运算符，将旋转部分设置为给定的旋转轴-角度表示。
   * Assignment operator setting the rotation component using an axis-angle
   * representation.
   * @param a 旋转轴-角度对象 / Axis-angle representation.
   * @return 当前 Transform 对象的引用 / Reference to the current Transform object.
   */
  Transform& operator=(const Eigen::AngleAxis<Scalar>& a)
  {
    rotation = Eigen::Quaternion<Scalar>(a);
    return *this;
  }

  /**
   * @brief 赋值运算符，将平移部分设置为给定的位移。
   * Assignment operator setting the translation component to the given position.
   * @param p 平移向量 / Translation vector.
   * @return 当前 Transform 对象的引用 / Reference to the current Transform object.
   */
  Transform& operator=(const Position<Scalar>& p)
  {
    translation = p;
    return *this;
  }

  /**
   * @brief 计算当前变换与另一个变换的组合。
   * Computes the composition of the current transformation with another transformation.
   * @param t 另一个变换 / Another transformation.
   * @return 组合后的变换 / Resulting transformation.
   */
  Transform operator+(const Transform& t) const
  {
    return Transform(Eigen::Quaternion<Scalar>(rotation * t.rotation),
                     Eigen::Matrix<Scalar, 3, 1>(translation + rotation * t.translation));
  }

  /**
   * @brief 计算当前变换与另一个变换的相对变换。
   * Computes the relative transformation between the current transformation and another
   * transformation.
   * @param t 另一个变换 / Another transformation.
   * @return 计算得到的相对变换 / Resulting relative transformation.
   */
  Transform operator-(const Transform& t) const
  {
    return Transform(rotation / t.rotation,
                     Eigen::Matrix<Scalar, 3, 1>(translation - t.translation));
  }
};

}  // namespace LibXR

#endif

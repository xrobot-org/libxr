#pragma once

#ifndef LIBXR_NO_EIGEN

#include <Eigen/Core>
#include <Eigen/Dense>
#include <type_traits>

#include "libxr_mem.hpp"
#include "transform.hpp"

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

/**
 * @class Inertia
 * @brief 表示刚体的惯性张量和质量信息的类。
 * Provides a class to represent the inertia tensor and mass of a rigid body.
 * @tparam Scalar 数据类型，默认为 DefaultScalar。
 * The data type, defaulting to DefaultScalar.
 */
template <typename Scalar = DefaultScalar>
class Inertia
{
 public:
  Scalar data[9];  ///< 惯性张量数据，3x3 矩阵存储为一维数组 / Inertia tensor data stored
                   ///< as a 3x3 matrix in a 1D array.
  Scalar mass;     ///< 质量值 / Mass value.

  /**
   * @brief 使用质量和 3x3 惯性张量数组构造惯性对象。
   * Constructs an inertia object using mass and a 3x3 inertia tensor array.
   * @tparam T 数据类型，支持 Scalar、float 或 double。
   * Data type, supporting Scalar, float, or double.
   * @param m 质量值 / Mass value.
   * @param values 3x3 惯性张量数组 / 3x3 inertia tensor array.
   */
  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  explicit Inertia(Scalar m, const T (&values)[9]) : mass(m)
  {
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        data[i * 3 + j] = static_cast<Scalar>(values[i * 3 + j]);
      }
    }
  }

  /**
   * @brief 使用质量和二维 3x3 数组构造惯性对象。
   * Constructs an inertia object using mass and a 3x3 matrix.
   * @tparam T 数据类型，支持 Scalar、float 或 double。
   * Data type, supporting Scalar, float, or double.
   * @param m 质量值 / Mass value.
   * @param matrix 3x3 惯性张量矩阵 / 3x3 inertia tensor matrix.
   */
  template <typename T, std::enable_if_t<std::is_same<T, Scalar>::value ||
                                             std::is_same<T, float>::value ||
                                             std::is_same<T, double>::value,
                                         int> = 0>
  explicit Inertia(Scalar m, const T (&matrix)[3][3]) : mass(m)
  {
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        data[i * 3 + j] = static_cast<Scalar>(matrix[i][j]);
      }
    }
  }

  /**
   * @brief 默认构造函数，初始化质量为0，惯性张量为零矩阵。
   * Default constructor initializing mass to 0 and inertia tensor to zero.
   */
  Inertia() : mass(0) { Memory::FastSet(data, 0, sizeof(data)); }

  /**
   * @brief 使用质量和 6 维数组（对称惯性矩阵）构造惯性对象。
   * Constructs an inertia object using mass and a 6-element symmetric inertia matrix.
   * @tparam T 数据类型，支持 float 或 double / Data type, supporting float or double.
   * @param m 质量值 / Mass value.
   * @param arr 6 维惯性张量数据 / 6-element inertia tensor data.
   */
  template <typename T,
            std::enable_if_t<
                std::is_same<T, float>::value || std::is_same<T, double>::value, int> = 0>
  explicit Inertia(Scalar m, const T (&arr)[6])
      : data{arr[0],  -arr[3], -arr[5], -arr[3], arr[1],
             -arr[4], -arr[5], -arr[4], arr[2]},
        mass(m)
  {
  }

  /**
   * @brief 直接指定质量和惯性张量分量构造惯性对象。
   * Constructs an inertia object with mass and specified inertia tensor elements.
   * @param m 质量值 / Mass value.
   * @param xx, yy, zz 主惯性矩 / Principal moments of inertia.
   * @param xy, yz, xz 交叉惯性矩 / Cross moments of inertia.
   */
  Inertia(Scalar m, Scalar xx, Scalar yy, Scalar zz, Scalar xy, Scalar yz, Scalar xz)
      : data{xx, -xy, -xz, -xy, yy, -yz, -xz, -yz, zz}, mass(m)
  {
  }

  /**
   * @brief 使用 Eigen 3x3 矩阵构造惯性对象。
   * Constructs an inertia object using an Eigen 3x3 matrix.
   * @param m 质量值 / Mass value.
   * @param R 3x3 惯性张量矩阵 / 3x3 inertia tensor matrix.
   */
  Inertia(Scalar m, const Eigen::Matrix<Scalar, 3, 3>& R) : mass(m)
  {
    data[0] = R.data()[0];
    data[1] = R.data()[1];
    data[2] = R.data()[2];
    data[3] = R.data()[3];
    data[4] = R.data()[4];
    data[5] = R.data()[5];
    data[6] = R.data()[6];
    data[7] = R.data()[7];
    data[8] = R.data()[8];
  }

  /// @brief 转换为 Eigen 3x3 矩阵 / Converts to an Eigen 3x3 matrix.
  operator Eigen::Matrix<Scalar, 3, 3>() const
  {
    return Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(data);
  }

  /// @brief 获取惯性张量中的特定元素。
  /// Retrieves a specific element from the inertia tensor.
  Scalar operator()(int i, int j) const { return data[i + j * 3]; }

  /// @brief 将惯性张量与另一个 3x3 矩阵相加。
  /// Adds the inertia tensor with another 3x3 matrix.

  Eigen::Matrix<Scalar, 3, 3> operator+(const Eigen::Matrix<Scalar, 3, 3>& R) const
  {
    return Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(data) + R;
  }

  /**
   * @brief 平移惯性对象 / Translates the inertia object.
   * @param p 平移向量 / Translation vector.
   * @return 平移后的惯性对象 / Translated inertia object.
   */
  Inertia Translate(const Eigen::Matrix<Scalar, 3, 1>& p) const
  {
    Scalar dx = p(0), dy = p(1), dz = p(2);
    Eigen::Matrix<Scalar, 3, 3> translation_matrix;
    translation_matrix << dy * dy + dz * dz, -dx * dy, -dx * dz, -dx * dy,
        dx * dx + dz * dz, -dy * dz, -dx * dz, -dy * dz, dx * dx + dy * dy;

    return Inertia(mass, Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(data) +
                             mass * translation_matrix);
  }

  /**
   * @brief 旋转惯性张量 / Rotates the inertia tensor.
   * @param R 旋转矩阵 / Rotation matrix.
   * @return 旋转后的惯性张量 / Rotated inertia tensor.
   */
  Inertia Rotate(const Eigen::Matrix<Scalar, 3, 3>& R) const
  {
    return Inertia(
        mass, R * Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(data) * R.transpose());
  }

  /**
   * @brief 使用 RotationMatrix 旋转惯性张量。
   * Rotates the inertia tensor using a RotationMatrix.
   * @param R 旋转矩阵对象 / Rotation matrix object.
   * @return 旋转后的惯性张量 / Rotated inertia tensor.
   */
  Inertia Rotate(const RotationMatrix<Scalar>& R) const
  {
    return Rotate(Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(R.data()));
  }

  /**
   * @brief 使用四元数旋转惯性张量 / Rotates the inertia tensor using a quaternion.
   * @param q 四元数 / Quaternion.
   * @return 旋转后的惯性张量 / Rotated inertia tensor.
   */
  Inertia Rotate(const Eigen::Quaternion<Scalar>& q) const
  {
    return Inertia(
        mass, q * Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>(data) * q.conjugate());
  }

  /**
   * @brief 使用四元数旋转 3x3 矩阵 / Rotates a 3x3 matrix using a quaternion.
   * @param R 3x3 矩阵 / 3x3 matrix.
   * @param q 四元数 / Quaternion.
   * @return 旋转后的矩阵 / Rotated matrix.
   */
  static Eigen::Matrix<Scalar, 3, 3> Rotate(const Eigen::Matrix<Scalar, 3, 3>& R,
                                            const Eigen::Quaternion<Scalar>& q)
  {
    return q * R * q.conjugate();
  }

  /**
   * @brief 使用自定义 Quaternion 旋转惯性张量。
   * Rotates the inertia tensor using a custom Quaternion.
   * @param q 自定义四元数对象 / Custom quaternion object.
   * @return 旋转后的惯性张量 / Rotated inertia tensor.
   */
  Inertia Rotate(const Quaternion<Scalar>& q) const
  {
    return Rotate(Eigen::Quaternion<Scalar>(q));
  }
};

/**
 * @class CenterOfMass
 * @brief 质心信息表示类 / Represents the center of mass information.
 * @tparam Scalar 数据类型，默认为 DefaultScalar / Data type, defaulting to DefaultScalar.
 */
template <typename Scalar = DefaultScalar>
class CenterOfMass
{
 public:
  Eigen::Matrix<Scalar, 3, 1> position;  ///< 质心位置 / Center of mass position.
  Scalar mass;                           ///< 质量值 / Mass value.

  /// @brief 默认构造函数，初始化质心位置为 (0,0,0)，质量为 0。
  /// Default constructor initializing position to (0,0,0) and mass to 0.
  CenterOfMass() : position(0., 0., 0.), mass(0.) {}

  /**
   * @brief 使用质量和位置构造质心对象。
   * Constructs a center of mass object using mass and position.
   * @param m 质量值 / Mass value.
   * @param p 质心位置 / Center of mass position.
   */
  CenterOfMass(Scalar m, const LibXR::Position<Scalar>& p) : position(p), mass(m) {}

  /**
   * @brief 使用质量和 Eigen 3D 向量构造质心对象。
   * Constructs a center of mass object using mass and Eigen 3D vector.
   * @param m 质量值 / Mass value.
   * @param p 3D 位置向量 / 3D position vector.
   */
  CenterOfMass(Scalar m, const Eigen::Matrix<Scalar, 3, 1>& p) : position(p), mass(m) {}

  /**
   * @brief 从惯性对象和变换构造质心对象。
   * Constructs a center of mass object from inertia and transformation.
   * @param m 惯性对象 / Inertia object.
   * @param p 变换对象 / Transformation object.
   */
  CenterOfMass(const Inertia<Scalar>& m, const Transform<Scalar>& p)
      : position(p.translation), mass(m.mass)
  {
  }

  /**
   * @brief 计算两个质心的合成 / Computes the combined center of mass.
   * @param m 另一个质心对象 / Another center of mass object.
   * @return 合并后的质心对象 / Combined center of mass object.
   */
  CenterOfMass operator+(const CenterOfMass& m) const
  {
    Scalar new_mass = mass + m.mass;
    return CenterOfMass(
        new_mass,
        Position<Scalar>((position(0) * mass + m.position(0) * m.mass) / new_mass,
                         (position(1) * mass + m.position(1) * m.mass) / new_mass,
                         (position(2) * mass + m.position(2) * m.mass) / new_mass));
  }

  /**
   * @brief 质心累加运算符 / Accumulation operator for center of mass.
   * @param m 另一个质心对象 / Another center of mass object.
   * @return 当前对象的引用 / Reference to the current object.
   */
  CenterOfMass& operator+=(const CenterOfMass& m)
  {
    *this = *this + m;
    return *this;
  }
};

}  // namespace LibXR

#endif

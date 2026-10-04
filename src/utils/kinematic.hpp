#pragma once

#ifndef LIBXR_NO_EIGEN

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "inertia.hpp"
#include "list.hpp"
#include "transform.hpp"

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

namespace Kinematic
{
/**
 * @brief 关节（Joint）类，表示机器人连杆间的旋转关节。
 *        Joint class representing a rotational joint between robot links.
 *
 * 该类包含关节的运动学信息，如旋转轴、前向/逆向运动学参数，并提供状态与目标设定方法。
 * This class contains kinematic information of the joint, such as rotation axis,
 * forward/inverse kinematics parameters, and provides methods to set states and targets.
 *
 * @tparam Scalar 角度和位置的存储类型。
 *                The storage type for angles and positions.
 */
template <typename Scalar = DefaultScalar>
class Joint;
template <typename Scalar = DefaultScalar>
class Object;
template <typename Scalar = DefaultScalar>
class EndPoint;
template <typename Scalar = DefaultScalar>
class StartPoint;

template <typename Scalar>
class Joint
{
 public:
  /**
   * @brief 关节参数结构体。
   *        Structure containing joint parameters.
   */
  typedef struct Param
  {
    Transform<Scalar> parent2this;  ///< 父坐标系到当前关节的变换 / Transform from the
                                    ///< parent frame to the current joint.
    Transform<Scalar> this2child;   ///< 当前关节到子坐标系的变换 / Transform from the
                                    ///< current joint to the child frame.
    Axis<Scalar> axis;              ///< 关节旋转轴 / Rotation axis of the joint.
    Scalar
        ik_mult;  ///< 逆向运动学步长系数 / Step size coefficient for inverse kinematics.
  } Param;

  /**
   * @brief 关节运行时状态结构体。
   *        Structure containing runtime state of the joint.
   */
  typedef struct Runtime
  {
    Eigen::AngleAxis<Scalar>
        state_angle;  ///< 当前关节角度状态 / Current joint angle state.
    Eigen::AngleAxis<Scalar>
        target_angle;  ///< 目标关节角度状态 / Target joint angle state.

    Eigen::Matrix<Scalar, 3, 3>
        inertia;  ///< 关节的惯性矩阵 / Inertia matrix of the joint.

    Axis<Scalar> state_axis;   ///< 当前关节轴向 / Current axis orientation.
    Axis<Scalar> target_axis;  ///< 目标关节轴向 / Target axis orientation.

    Transform<Scalar>
        state;  ///< 当前状态的变换矩阵 / Transformation matrix of the current state.
    Transform<Scalar>
        target;  ///< 目标状态的变换矩阵 / Transformation matrix of the target state.
  } Runtime;

  Runtime runtime_;  ///< 关节的运行时数据 / Runtime data of the joint.

  Object<Scalar>* parent = nullptr;  ///< 指向父物体的指针 / Pointer to the parent object.
  Object<Scalar>* child = nullptr;   ///< 指向子物体的指针 / Pointer to the child object.
  Param param_;                      ///< 关节的参数配置 / Parameters of the joint.

  /**
   * @brief 构造 `Joint` 关节对象。
   *        Constructs a `Joint` object.
   *
   * @param axis 关节旋转轴 / Rotation axis of the joint.
   * @param parent 关节的父对象 / Parent object of the joint.
   * @param parent2this 父坐标系到该关节的变换。
   * Transform from the parent frame to this joint.
   * @param child 关节的子对象 / Child object of the joint.
   * @param this2child 该关节到子坐标系的变换。
   * Transform from this joint to the child frame.
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   */
  Joint(Axis<Scalar> axis, Object<Scalar>* parent, Transform<Scalar>& parent2this,
        Object<Scalar>* child, Transform<Scalar>& this2child)
      : parent(parent), child(child), param_({parent2this, this2child, axis, 1.0})
  {
    runtime_.inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();
    auto link = new typename Object<Scalar>::Link(this);
    parent->joints.Add(*link);
    child->parent = this;
  }

  /**
   * @brief 设置关节当前状态角度。
   *        Sets the current state angle of the joint.
   *
   * 该函数确保角度值在 `[-π, π]` 之间。
   * This function ensures the angle value remains within `[-π, π]`.
   *
   * @param state 需要设置的角度值（弧度） / The angle value to be set (in radians).
   */
  void SetState(Scalar state)
  {
    if (state > LibXR::PI)
    {
      state -= 2 * LibXR::PI;
    }
    if (state < -LibXR::PI)
    {
      state += 2 * LibXR::PI;
    }
    runtime_.state_angle.angle() = state;
    runtime_.state_angle.axis() = param_.axis;
  }

  /**
   * @brief 设置关节的目标角度。
   *        Sets the target angle of the joint.
   *
   * 该函数确保目标角度值在 `[-π, π]` 之间。
   * This function ensures the target angle value remains within `[-π, π]`.
   *
   * @param target 目标角度值（弧度） / The target angle value (in radians).
   */
  void SetTarget(Scalar target)
  {
    if (target > LibXR::PI)
    {
      target -= 2 * LibXR::PI;
    }
    if (target < -LibXR::PI)
    {
      target += 2 * LibXR::PI;
    }
    runtime_.target_angle.angle() = target;
    runtime_.target_angle.axis() = param_.axis;
  }

  /**
   * @brief 设置逆向运动学计算的步长系数。
   *        Sets the step size coefficient for inverse kinematics calculations.
   *
   * 该参数用于调整关节在逆向运动学计算中的变化速率。
   * This parameter adjusts the rate of change of the joint during inverse kinematics
   * calculations.
   *
   * @param mult 逆向运动学步长系数 / The step size coefficient.
   */
  void SetBackwardMult(Scalar mult) { param_.ik_mult = mult; }
};

/**
 * @brief 机器人中的物体（Object）类。
 *        Object class representing an entity in the robot.
 *
 * 该类表示机器人中的刚性体，如连杆、基座等，并提供运动学计算方法。
 * This class represents rigid bodies in the robot, such as links and bases,
 * and provides kinematic computation methods.
 *
 * @tparam Scalar 角度和位置的存储类型。
 *                The storage type for angles and positions.
 */
template <typename Scalar>
class Object
{
 public:
  typedef List::Node<Joint<Scalar>*> Link;  ///< 关节链接类型 / Type for linking joints.

  /**
   * @brief 物体参数结构体，存储物体的惯性参数。
   *        Structure storing inertia parameters of the object.
   */
  typedef struct
  {
    Inertia<Scalar> inertia;
  } Param;

  /**
   * @brief 物体运行时状态结构体，存储物体的变换信息。
   *        Structure storing runtime transformation data of the object.
   */
  typedef struct
  {
    Transform<Scalar>
        state;  ///< 物体的当前状态变换矩阵 / Transformation matrix of the current state.
    Transform<Scalar>
        target;  ///< 物体的目标状态变换矩阵 / Transformation matrix of the target state.
  } Runtime;

  List joints;  ///< 物体的关节列表 / List of joints associated with the object.

  Joint<Scalar>* parent = nullptr;  ///< 指向父关节的指针 / Pointer to the parent joint.

  Param param_;      ///< 物体参数 / Object parameters.
  Runtime runtime_;  ///< 物体运行时状态 / Object runtime data.

  /**
   * @brief 使用惯性参数构造 `Object` 对象。
   *        Constructs an `Object` using inertia parameters.
   *
   * @param inertia 物体的惯性参数 / The inertia parameters of the object.
   */
  Object(Inertia<Scalar>& inertia) : param_({inertia}) {}

  /**
   * @brief 设置物体的位置信息。
   *        Sets the position of the object.
   *
   * 该函数将 `pos` 赋值给 `runtime_.state`，用于更新物体的当前位置。
   * This function assigns `pos` to `runtime_.state`, updating the current position of the
   * object.
   *
   * @param pos 物体的新位置。
   *            The new position of the object.
   */
  void SetPosition(const Position<Scalar>& pos) { runtime_.state = pos; }

  /**
   * @brief 设置物体的旋转信息（四元数表示）。
   *        Sets the rotation of the object using a quaternion.
   *
   * 该函数将 `quat` 赋值给 `runtime_.state`，用于更新物体的当前旋转状态。
   * This function assigns `quat` to `runtime_.state`, updating the current rotation state
   * of the object.
   *
   * @param quat 物体的新旋转四元数。
   *             The new quaternion representing the object's rotation.
   */
  void SetQuaternion(const Quaternion<Scalar>& quat) { runtime_.state = quat; }

  virtual void CalcBackward() {}
};

/**
 * @brief 机器人末端点（EndPoint）类，继承自 Object。
 *        EndPoint class representing the end effector of a robotic system, inheriting
 * from Object.
 *
 * 该类用于处理机器人末端的目标位置与方向，
 * 并计算逆运动学（IK）以调整关节角度，使末端达到期望位置。
 * This class handles the target position and orientation of the robot's end effector
 * and computes inverse kinematics (IK) to adjust joint angles to reach the desired
 * position.
 *
 * @tparam Scalar 角度和位置的存储类型。
 *                The storage type for angles and positions.
 */
template <typename Scalar>
class EndPoint : public Object<Scalar>
{
 private:
  Eigen::Matrix<Scalar, 6, Eigen::Dynamic>* jacobian_matrix_ =
      nullptr;  ///< 雅可比矩阵，用于逆运动学计算 / Jacobian matrix for inverse kinematics
                ///< calculations.
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>* delta_theta_ =
      nullptr;  ///< 关节角度调整量 / Joint angle adjustments.
  Eigen::Matrix<Scalar, 6, 1> err_weight_ =
      Eigen::Matrix<Scalar, 6, 1>::Constant(1);  ///< 误差权重矩阵 / Error weight matrix.
  int joint_num_ = 0;  ///< 机器人末端至基座的关节数量 / Number of joints from the end
                       ///< effector to the base.

  Quaternion<Scalar> target_quat_;  ///< 目标四元数方向 / Target quaternion orientation.
  Position<Scalar> target_pos_;     ///< 目标位置 / Target position.
  Scalar max_angular_velocity_ =
      -1.0;  ///< 最大角速度（若小于 0 则无约束） / Maximum angular velocity (negative
             ///< value means no limit).
  Scalar max_line_velocity_ = -1.0;  ///< 最大线速度（若小于 0 则无约束） / Maximum linear
                                     ///< velocity (negative value means no limit).

 public:
  /**
   * @brief 构造 `EndPoint` 末端点对象。
   *        Constructs an `EndPoint` object.
   *
   * @param inertia 物体的惯性参数 / The inertia parameters of the object.
   */
  EndPoint(Inertia<Scalar>& inertia) : Object<Scalar>(inertia) {}

  /**
   * @brief 设置目标四元数方向。
   *        Sets the target quaternion orientation.
   *
   * @param quat 目标四元数 / The target quaternion.
   */
  void SetTargetQuaternion(const Quaternion<Scalar>& quat) { target_quat_ = quat; }

  /**
   * @brief 设置目标位置。
   *        Sets the target position.
   *
   * @param pos 目标位置 / The target position.
   */
  void SetTargetPosition(const Position<Scalar>& pos) { target_pos_ = pos; }

  /**
   * @brief 设置误差权重矩阵。
   *        Sets the error weight matrix.
   *
   * 该矩阵用于调整不同方向上的误差贡献权重。
   * This matrix is used to adjust the contribution weights of errors in different
   * directions.
   *
   * @param weight 误差权重矩阵 / The error weight matrix.
   */
  void SetErrorWeight(const Eigen::Matrix<Scalar, 6, 1>& weight) { err_weight_ = weight; }

  /**
   * @brief 设置最大角速度。
   *        Sets the maximum angular velocity.
   *
   * 该函数用于限制末端点旋转变化的速率。
   * This function limits the rate of rotational changes at the end effector.
   *
   * @param velocity 角速度上限（负值表示无限制）。
   * The maximum angular velocity (negative value means no limit).
   */
  void SetMaxAngularVelocity(Scalar velocity) { max_angular_velocity_ = velocity; }

  /**
   * @brief 获取末端点位置误差。
   *        Gets the position error of the end effector.
   *
   * 该误差表示当前状态与目标位置之间的差值。
   * This error represents the difference between the current state and the target
   * position.
   *
   * @return 位置误差向量 / The position error vector.
   */
  Eigen::Matrix<Scalar, 3, 1> GetPositionError()
  {
    return target_pos_ - Object<Scalar>::runtime_.target.translation;
  }

  /**
   * @brief 获取末端点方向误差。
   *        Gets the orientation error of the end effector.
   *
   * 计算当前方向和目标方向的四元数误差。
   * Computes the quaternion error between the current and target orientations.
   *
   * @return 方向误差的四元数 / The quaternion representing the orientation error.
   */
  Eigen::Quaternion<Scalar> GetQuaternionError()
  {
    return Object<Scalar>::runtime_.target.rotation / target_quat_;
  }

  /**
   * @brief 设置最大线速度。
   *        Sets the maximum linear velocity.
   *
   * 该函数用于限制末端点线性移动的速率。
   * This function limits the rate of linear movement at the end effector.
   *
   * @param velocity 线速度上限（负值表示无限制）。
   * The maximum linear velocity (negative value means no limit).
   */
  void SetMaxLineVelocity(Scalar velocity) { max_line_velocity_ = velocity; }

  /**
   * @brief 计算逆运动学（IK），调整关节角度以达到目标位置和方向。
   *        Computes inverse kinematics (IK) to adjust joint angles for reaching the
   * target position and orientation.
   *
   * 该函数采用雅可比矩阵求解最优关节角度调整量，并进行迭代优化。
   * This function uses the Jacobian matrix to compute optimal joint angle adjustments and
   * performs iterative optimization.
   *
   * @param dt 时间步长 / Time step.
   * @param max_step 最大迭代步数 / Maximum number of iterations.
   * @param max_err 允许的最大误差 / Maximum allowable error.
   * @param step_size 逆运动学步长 / Step size for inverse kinematics.
   * @return 计算后的误差向量 / The computed error vector.
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   */
  Eigen::Matrix<Scalar, 6, 1> CalcBackward(Scalar dt, int max_step = 10,
                                           Scalar max_err = 1e-3, Scalar step_size = 1.0)
  {
    Eigen::Matrix<Scalar, 6, 1> error;

    /* Initialize */
    if (jacobian_matrix_ == nullptr)
    {
      Object<Scalar>* tmp = this;
      for (int i = 0;; i++)
      {
        if (tmp->parent == nullptr)
        {
          joint_num_ = i;
          break;
        }
        tmp = tmp->parent->parent;
      }

      jacobian_matrix_ = new Eigen::Matrix<Scalar, 6, Eigen::Dynamic>(6, joint_num_);

      delta_theta_ = new Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(joint_num_);
    }

    /* Apply limits */
    Position<Scalar> target_pos = target_pos_;
    Quaternion<Scalar> target_quat = target_quat_;
    if (max_line_velocity_ > 0 && max_angular_velocity_ > 0)
    {
      Scalar max_pos_delta = max_angular_velocity_ * dt;
      Scalar max_angle_delta = max_line_velocity_ * dt;

      Position<Scalar> pos_err =
          Position<Scalar>(target_pos_ - this->runtime_.target.translation);
      Eigen::AngleAxis<Scalar> angle_err(
          Quaternion<Scalar>(target_quat_ / this->runtime_.target.rotation));

      Scalar pos_err_norm = pos_err.norm();

      if (pos_err_norm > max_pos_delta)
      {
        pos_err = (max_pos_delta / pos_err_norm) * pos_err;
        target_pos = this->runtime_.target.translation + pos_err;
      }
      else
      {
        target_pos = target_pos_;
      }

      if (angle_err.angle() > max_angle_delta)
      {
        angle_err.angle() = max_angle_delta;
        target_quat = this->runtime_.target.rotation * angle_err;
      }
      else
      {
        target_quat = target_quat_;
      }
    }

    for (int step = 0; step < max_step; ++step)
    {
      /* Calculate Error */
      error.template head<3>() = target_pos - this->runtime_.target.translation;
      error.template tail<3>() =
          (Eigen::Quaternion<Scalar>(this->runtime_.target.rotation).conjugate() *
           target_quat)
              .vec();

      error = err_weight_.array() * error.array();

      auto err_norm = error.norm();

      if (err_norm < max_err)
      {
        break;
      }

      /* Calculate Jacobian */
      do
      {
        Joint<Scalar>* joint = this->parent;
        for (int joint_index = 0; joint_index < joint_num_; joint_index++)
        {
          /* J = [translation[3], rotation[3]] */
          Eigen::Matrix<Scalar, 6, 1> d_transform;
          d_transform.template head<3>() = joint->runtime_.target_axis.cross(
              this->runtime_.target.translation - joint->runtime_.target.translation);

          d_transform.template tail<3>() = joint->runtime_.target_axis;

          jacobian_matrix_->col(joint_index) = d_transform;

          joint = joint->parent->parent;
        }
      } while (0);

      /* Calculate delta_theta: delta_theta = J^+ * error */
      *delta_theta_ =
          jacobian_matrix_->completeOrthogonalDecomposition().pseudoInverse() * error *
          step_size / std::sqrt(err_norm);

      /* Update Joint Angle */
      do
      {
        Joint<Scalar>* joint = this->parent;

        for (int joint_index = 0; joint_index < joint_num_; joint_index++)
        {
          Eigen::AngleAxis<Scalar> target_angle_axis_delta((*delta_theta_)(joint_index),
                                                           joint->runtime_.target_axis);

          target_angle_axis_delta =
              Quaternion<Scalar>(Eigen::Quaternion<Scalar>(target_angle_axis_delta)) /
              joint->runtime_.target.rotation;

          joint->SetTarget(joint->runtime_.target_angle.angle() +
                           (*delta_theta_)(joint_index)*joint->param_.ik_mult);

          joint = joint->parent->parent;
        }
      } while (0);

      /* Recalculate Forward Kinematics */
      do
      {
        Joint<Scalar>* joint = this->parent;

        for (int joint_index = 0; joint_index < joint_num_; joint_index++)
        {
          if (joint_index == joint_num_ - 1)
          {
            auto start_point = reinterpret_cast<StartPoint<Scalar>*>(joint->parent);
            start_point->CalcTargetForward();
            break;
          }

          joint = joint->parent->parent;
        }
      } while (0);
    }

    return error;
  }
};

/**
 * @brief 机器人起始点（StartPoint）类，继承自 Object。
 *        StartPoint class representing the base or root of a robotic kinematic chain,
 * inheriting from Object.
 *
 * 该类作为运动学链的起始点，负责计算前向运动学、逆向运动学、
 * 质心计算以及惯性计算等关键功能。
 * This class serves as the starting point of a kinematic chain and is responsible for
 * computing forward kinematics, inverse kinematics, center of mass calculation,
 * and inertia computation.
 *
 * @tparam Scalar 角度和位置的存储类型。
 *                The storage type for angles and positions.
 */
template <typename Scalar>
class StartPoint : public Object<Scalar>
{
 public:
  CenterOfMass<Scalar> cog;  ///< 机器人质心 / The center of mass of the robot.

  /**
   * @brief 构造 `StartPoint` 物体对象。
   *        Constructs a `StartPoint` object.
   *
   * @param inertia 物体的惯性参数 / The inertia parameters of the object.
   */
  StartPoint(Inertia<Scalar>& inertia) : Object<Scalar>(inertia) {}

  /**
   * @brief 计算当前状态的前向运动学（FK）。
   *        Computes forward kinematics (FK) for the current state.
   *
   * 该函数遍历所有关节，根据当前状态计算其空间变换矩阵。
   * This function traverses all joints and calculates their spatial transformations based
   * on the current state.
   */
  void CalcForward()
  {
    this->runtime_.target = this->runtime_.state;
    auto fun = [&](Joint<Scalar>* joint) { return ForwardForeachFunLoop(joint, *this); };
    this->joints.template Foreach<Joint<Scalar>*>(fun);
  }

  /**
   * @brief 计算目标状态的前向运动学（FK）。
   *        Computes forward kinematics (FK) for the target state.
   *
   * 该函数用于目标位置调整，确保所有关节正确响应目标状态的变化。
   * This function is used for target position adjustments, ensuring all joints
   * correctly respond to changes in the target state.
   */
  void CalcTargetForward()
  {
    this->runtime_.target = this->runtime_.state;
    auto fun = [&](Joint<Scalar>* joint)
    { return TargetForwardForeachFunLoop(joint, *this); };
    this->joints.template Foreach<Joint<Scalar>*>(fun);
  }

  /**
   * @brief 计算机器人系统的惯性分布。
   *        Computes the inertia distribution of the robotic system.
   *
   * 该函数遍历所有关节，并根据质心和旋转惯性矩计算整体惯性分布。
   * This function traverses all joints and calculates the overall inertia distribution
   * based on the center of mass and rotational inertia matrix.
   */
  void CalcInertia()
  {
    Joint<Scalar>* res = nullptr;
    auto fun = [&](Joint<Scalar>* joint)
    { return InertiaForeachFunLoopStart(joint, res); };
    this->joints.template Foreach<Joint<Scalar>*>(fun);
  }

  /**
   * @brief 计算机器人系统的质心。
   *        Computes the center of mass (CoM) of the robotic system.
   *
   * 该函数遍历所有关节，并利用惯性计算系统整体质心位置。
   * This function traverses all joints and uses inertia calculations
   * to determine the overall center of mass.
   */
  void CalcCenterOfMass()
  {
    this->cog = CenterOfMass<Scalar>(this->param_.inertia, this->runtime_.state);
    this->joints.Foreach(CenterOfMassForeachFunLoop, *this);
  }

  /**
   * @brief 计算起始点惯性（首次遍历）。
   *        Computes the inertia of the start point (first traversal).
   */
  static ErrorCode InertiaForeachFunLoopStart(Joint<Scalar>* joint, Joint<Scalar>* parent)
  {
    UNUSED(parent);
    joint->runtime_.inertia = joint->child->param_.inertia
                                  .Translate(joint->runtime_.state.translation -
                                             joint->child->runtime_.state.translation)
                                  .Rotate(joint->child->runtime_.state.rotation /
                                          joint->runtime_.state.rotation);

    joint->runtime_.inertia = Inertia<Scalar>::Rotate(
        joint->runtime_.inertia, Eigen::Quaternion<Scalar>(joint->runtime_.state_angle));

    auto fun_loop = [&](Joint<Scalar>* child_joint)
    { return InertiaForeachFunLoop(child_joint, joint); };

    auto fun_start = [&](Joint<Scalar>* child_joint)
    { return InertiaForeachFunLoopStart(child_joint, joint); };

    joint->child->joints.template Foreach<Joint<Scalar>*>(fun_loop);
    joint->child->joints.template Foreach<Joint<Scalar>*>(fun_start);

    return ErrorCode::OK;
  }

  /**
   * @brief 计算机器人系统的惯性（后续遍历）。
   *        Computes the inertia of the robotic system (subsequent traversal).
   */
  static ErrorCode InertiaForeachFunLoop(Joint<Scalar>* joint, Joint<Scalar>* parent)
  {
    auto new_inertia =
        joint->child->param_.inertia
            .Translate(parent->runtime_.state.translation -
                       joint->child->runtime_.state.translation)
            .Rotate(parent->runtime_.state.rotation / joint->runtime_.state.rotation);

    parent->runtime_.inertia = new_inertia + parent->runtime_.inertia;

    auto fun_loop = [&](Joint<Scalar>* child_joint)
    { return InertiaForeachFunLoop(child_joint, joint); };

    joint->child->joints.template Foreach<Joint<Scalar>*>(fun_loop);

    return ErrorCode::OK;
  }

  /**
   * @brief 计算当前状态的前向运动学（FK）。
   *        Computes forward kinematics (FK) for the current state.
   */
  static ErrorCode ForwardForeachFunLoop(Joint<Scalar>* joint, StartPoint<Scalar>& start)
  {
    Transform<Scalar> t_joint(joint->parent->runtime_.state + joint->param_.parent2this);

    joint->runtime_.state = t_joint;

    Transform<Scalar> t_child(
        joint->runtime_.state.rotation * joint->runtime_.state_angle,
        joint->runtime_.state.translation);

    t_child = t_child + joint->param_.this2child;

    joint->child->runtime_.state = t_child;
    joint->runtime_.state_axis = joint->runtime_.state.rotation * joint->param_.axis;
    joint->runtime_.target_axis = joint->runtime_.state_axis;

    joint->runtime_.target = joint->runtime_.state;
    joint->runtime_.target_angle = joint->runtime_.state_angle;
    joint->child->runtime_.target = joint->child->runtime_.state;

    auto fun = [&](Joint<Scalar>* child_joint)
    { return ForwardForeachFunLoop(child_joint, start); };

    joint->child->joints.template Foreach<Joint<Scalar>*>(fun);

    return ErrorCode::OK;
  }

  /**
   * @brief 计算目标状态的前向运动学（FK）。
   *        Computes forward kinematics (FK) for the target state.
   */
  static ErrorCode TargetForwardForeachFunLoop(Joint<Scalar>* joint,
                                               StartPoint<Scalar>& start)
  {
    Transform<Scalar> t_joint(joint->parent->runtime_.target + joint->param_.parent2this);

    joint->runtime_.target = t_joint;

    Transform<Scalar> t_child(
        joint->runtime_.target.rotation * joint->runtime_.target_angle,
        joint->runtime_.target.translation);

    t_child = t_child + joint->param_.this2child;
    joint->child->runtime_.target = t_child;

    joint->runtime_.target_axis = joint->runtime_.target.rotation * joint->param_.axis;

    auto fun = [&](Joint<Scalar>* child_joint)
    { return TargetForwardForeachFunLoop(child_joint, start); };

    joint->child->joints.template Foreach<Joint<Scalar>*>(fun);

    return ErrorCode::OK;
  }

  /**
   * @brief 计算系统的质心。
   *        Computes the center of mass of the system.
   */
  static ErrorCode CenterOfMassForeachFunLoop(Joint<Scalar>* joint,
                                              StartPoint<Scalar>& start)
  {
    CenterOfMass<Scalar> child_cog(joint->child->param_.inertia,
                                   joint->child->runtime_.state);

    start.cog += child_cog;

    joint->child->joints.Foreach(TargetForwardForeachFunLoop, start);

    return ErrorCode::OK;
  }
};

}  // namespace Kinematic

}  // namespace LibXR

#endif

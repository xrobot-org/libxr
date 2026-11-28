/**
 * @file test_kinematic.cpp
 * @brief 双关节运动学求解测试 / Two-joint kinematics test.
 *
 * 建立双关节模型并求解目标位姿，检查末端的位置误差和四元数误差。
 * Build a two-joint model, solve for a target pose and check endpoint position and
 * quaternion errors.
 */

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_kinematic()
{
  LibXR::Inertia inertia_endpoint(1.0, 0.1, 0.1, 0.1, 0., 0., 0.);
  LibXR::Inertia inertia_midpoint(1.0, 0.1, 0.1, 0.1, 0., 0., 0.);
  LibXR::Inertia inertia_startpoint(1000., 100., 100., 100., 0., 0., 0.);

  LibXR::Position pos_startpoint(0., 0., 1.);
  LibXR::Quaternion quat_startpoint = LibXR::EulerAngle(0., 0., 0.).ToQuaternion();

  LibXR::Position pos_startpoint2joint(0., 0.0, 0.5);
  LibXR::Position pos_joint2midpoint(1.0, 0.0, 0.0);
  LibXR::Position pos_midpoint2joint(1.0, 0.0, 0.0);
  LibXR::Position pos_joint2endpoint(0.5, 0.0, 0.0);

  LibXR::Quaternion quat_startpoint2joint(1., 0., 0., 0.);
  LibXR::Quaternion quat_joint2midpoint(1., 0., 0., 0.);
  LibXR::Quaternion quat_midpoint2joint(1., 0., 0., 0.);
  LibXR::Quaternion quat_joint2endpoint(1., 0., 0., 0.);

  LibXR::Transform t_startpoint2joint(quat_startpoint2joint, pos_startpoint2joint);
  LibXR::Transform t_joint2midpoint(quat_joint2midpoint, pos_joint2midpoint);
  LibXR::Transform t_midpoint2joint(quat_midpoint2joint, pos_midpoint2joint);
  LibXR::Transform t_joint2endpoint(quat_joint2endpoint, pos_joint2endpoint);

  LibXR::Kinematic::EndPoint object_endpoint(inertia_endpoint);
  LibXR::Kinematic::Object object_midpoint(inertia_midpoint);
  LibXR::Kinematic::StartPoint object_startpoint(inertia_startpoint);

  object_startpoint.SetPosition(pos_startpoint);
  object_startpoint.SetQuaternion(quat_startpoint);

  LibXR::Kinematic::Joint joint_midpoint(LibXR::Axis<>::Y(), &object_startpoint,
                                         t_startpoint2joint, &object_midpoint,
                                         t_joint2midpoint);

  LibXR::Kinematic::Joint joint_endpoint(LibXR::Axis<>::Y(), &object_midpoint,
                                         t_midpoint2joint, &object_endpoint,
                                         t_joint2endpoint);

  joint_endpoint.SetState(0.);
  joint_midpoint.SetState(LibXR::PI / 2);

  LibXR::Quaternion target_quat(0.7071068, 0., -0.7071068, 0.);
  LibXR::Position target_pos(0., 0., 4.);

  object_endpoint.SetTargetQuaternion(target_quat);
  object_endpoint.SetTargetPosition(target_pos);

  object_startpoint.CalcForward();
  object_startpoint.CalcInertia();

  object_endpoint.CalcBackward(0, 1000, 0.01, 0.1);

  auto error_pos = object_endpoint.GetPositionError();
  auto error_quat = object_endpoint.GetQuaternionError();

  TEST_ASSERT(error_pos.norm() < 1e-3);
  TEST_ASSERT(std::abs(error_quat.x()) < 1e-2 && std::abs(error_quat.y()) < 1e-2 &&
              std::abs(error_quat.z()) < 1e-2 && std::abs(error_quat.w() - 1.0) < 1e-2);

  // 二级串联关节 + 兄弟分支质心聚合回归测试 /
  // Two-level chain + sibling branch center-of-mass regression test.
  {
    LibXR::Inertia com_start_inertia(2.0, 0.1, 0.1, 0.1, 0., 0., 0.);
    LibXR::Inertia com_mid_inertia(3.0, 0.1, 0.1, 0.1, 0., 0., 0.);
    LibXR::Inertia com_end_inertia(5.0, 0.1, 0.1, 0.1, 0., 0., 0.);
    LibXR::Inertia com_branch_inertia(2.0, 0.1, 0.1, 0.1, 0., 0., 0.);

    LibXR::Kinematic::StartPoint com_start(com_start_inertia);
    LibXR::Kinematic::Object com_mid(com_mid_inertia);
    LibXR::Kinematic::EndPoint com_end(com_end_inertia);
    LibXR::Kinematic::EndPoint com_branch(com_branch_inertia);

    com_start.SetPosition(LibXR::Position(0., 0., 1.));
    com_start.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

    LibXR::Transform t_s2j1(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(0., 0., 0.5));
    LibXR::Transform t_j12m(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(1., 0., 0.));
    LibXR::Transform t_m2j2(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(1., 0., 0.));
    LibXR::Transform t_j22e(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(0.5, 0., 0.));
    LibXR::Transform t_s2jb(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(0., 2., 0.));
    LibXR::Transform t_jb2b(LibXR::Quaternion(1., 0., 0., 0.),
                            LibXR::Position(0., 1., 0.));

    LibXR::Kinematic::Joint j1(LibXR::Axis<>::Y(), &com_start, t_s2j1, &com_mid, t_j12m);
    LibXR::Kinematic::Joint j2(LibXR::Axis<>::Y(), &com_mid, t_m2j2, &com_end, t_j22e);
    LibXR::Kinematic::Joint jb(LibXR::Axis<>::Y(), &com_start, t_s2jb, &com_branch,
                               t_jb2b);
    j1.SetState(0.);
    j2.SetState(0.);
    jb.SetState(0.);

    com_start.CalcForward();
    com_start.CalcCenterOfMass();

    TEST_ASSERT(equal(com_start.cog.mass, 12.0));
    TEST_ASSERT(equal(com_start.cog.position(0), 15.5 / 12.0));
    TEST_ASSERT(equal(com_start.cog.position(1), 6.0 / 12.0));
    TEST_ASSERT(equal(com_start.cog.position(2), 16.0 / 12.0));
  }

  // 兄弟关节、祖先后代链赋值及物体赋值保留图边（无成环、无需手工恢复指针）回归测试 /
  // Sibling, ancestor-descendant Joint, and Object copy/move assignments preserve graph
  // edges without manual pointer restoration and never form traversal cycles.
  {
    LibXR::Inertia root_inertia(1.0, 0.1, 0.1, 0.1, 0., 0., 0.);
    LibXR::Inertia a_inertia(2.0, 0.1, 0.1, 0.1, 0., 0., 0.);
    LibXR::Inertia b_inertia(3.0, 0.1, 0.1, 0.1, 0., 0., 0.);

    LibXR::Kinematic::StartPoint root(root_inertia);
    LibXR::Kinematic::EndPoint child_a(a_inertia);
    LibXR::Kinematic::EndPoint child_b(b_inertia);

    root.SetPosition(LibXR::Position(0., 0., 0.));
    root.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

    LibXR::Transform ta(LibXR::Quaternion(1., 0., 0., 0.), LibXR::Position(1., 0., 0.));
    LibXR::Transform tb(LibXR::Quaternion(1., 0., 0., 0.), LibXR::Position(0., 2., 0.));
    LibXR::Transform tzero(LibXR::Quaternion(1., 0., 0., 0.),
                           LibXR::Position(0., 0., 0.));

    LibXR::Kinematic::Joint joint_a(LibXR::Axis<>::Z(), &root, ta, &child_a, tzero);
    LibXR::Kinematic::Joint joint_b(LibXR::Axis<>::Z(), &root, tb, &child_b, tzero);
    joint_a.SetState(0.);
    joint_b.SetState(0.);
    root.CalcForward();

    // 兄弟赋值后不手工恢复任何父子指针：必须保留原图边且复制参数/运行值。
    // Sibling copy assignment without manual pointer restoration preserves edges.
    joint_a = joint_b;
    TEST_ASSERT(joint_a.parent == &root);
    TEST_ASSERT(joint_a.child == &child_a);

    size_t visited = 0U;
    root.ForeachJoint(
        [&](LibXR::Kinematic::Joint<LibXR::DefaultScalar>*)
        {
          ++visited;
          TEST_ASSERT(visited <= 2U);
          return LibXR::ErrorCode::OK;
        });
    TEST_ASSERT(visited == 2U);

    root.CalcForward();
    root.CalcCenterOfMass();
    TEST_ASSERT(equal(root.cog.mass, 6.0));
    TEST_ASSERT(equal(root.cog.position(0), 0.0));
    TEST_ASSERT(equal(root.cog.position(1), 10.0 / 6.0));
    TEST_ASSERT(equal(root.cog.position(2), 0.0));

    // 祖先后代链 root -> j1 -> mid -> j2 -> leaf：直接执行 j2 = j1、移动赋值和
    // Object 赋值，均不得改写父子边或制造 mid -> j2 -> mid 递归环。
    // Ancestor-descendant chain assignment (copy, move, and Object assign) must not
    // overwrite graph edges or create a mid -> j2 -> mid recursion cycle.
    LibXR::Kinematic::StartPoint c_root(root_inertia);
    LibXR::Kinematic::Object c_mid(a_inertia);
    LibXR::Kinematic::EndPoint c_leaf(b_inertia);
    c_root.SetPosition(LibXR::Position(0., 0., 0.));
    c_root.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

    LibXR::Kinematic::Joint j1(LibXR::Axis<>::Z(), &c_root, ta, &c_mid, tzero);
    LibXR::Kinematic::Joint j2(LibXR::Axis<>::Z(), &c_mid, tb, &c_leaf, tzero);
    j1.SetState(0.);
    j2.SetState(0.);
    c_root.CalcForward();

    j2 = j1;
    TEST_ASSERT(j2.parent == &c_mid);
    TEST_ASSERT(j2.child == &c_leaf);
    c_root.CalcForward();
    c_root.CalcCenterOfMass();
    // j2 继承 j1 的 parent2this=(1,0,0)：c_mid 在 (1,0,0)，c_leaf 在 (2,0,0)
    TEST_ASSERT(equal(c_root.cog.mass, 6.0));
    TEST_ASSERT(equal(c_root.cog.position(0), (2.0 * 1.0 + 3.0 * 2.0) / 6.0));
    TEST_ASSERT(equal(c_root.cog.position(1), 0.0));

    // 移动赋值与 Object 赋值同样保留图边。
    // Move assignment and Object assignment preserve all graph edges too.
    LibXR::Kinematic::Joint detached_j(j1);
    TEST_ASSERT(detached_j.parent == nullptr && detached_j.child == nullptr);
    j2 = std::move(detached_j);
    TEST_ASSERT(j2.parent == &c_mid && j2.child == &c_leaf);

    LibXR::Kinematic::Object detached_o(c_mid);
    TEST_ASSERT(detached_o.parent == nullptr);
    size_t detached_children = 0U;
    detached_o.ForeachJoint(
        [&](LibXR::Kinematic::Joint<LibXR::DefaultScalar>*)
        {
          ++detached_children;
          return LibXR::ErrorCode::OK;
        });
    TEST_ASSERT(detached_children == 0U);

    c_mid = c_root;
    TEST_ASSERT(c_mid.parent == &j1);
    c_root.CalcForward();
    c_root.CalcCenterOfMass();
    TEST_ASSERT(std::isfinite(c_root.cog.position(0)));
    TEST_ASSERT(equal(c_root.cog.mass, 5.0));

    // EndPoint 复制/移动赋值不得复制与源拓扑绑定的雅可比矩阵和关节计数缓存：
    // 对两关节末端执行 CalcBackward() 后赋给单关节末端，单关节末端再求解 IK
    // 不得越界或解引用空指针。 EndPoint copy/move assignment must preserve each
    // endpoint's topology-bound IK cache.
    LibXR::Kinematic::StartPoint ik_root2(root_inertia);
    LibXR::Kinematic::Object ik_mid2(a_inertia);
    LibXR::Kinematic::EndPoint ik_ep2(b_inertia);
    ik_root2.SetPosition(LibXR::Position(0., 0., 0.));
    ik_root2.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));
    LibXR::Kinematic::Joint ik_j2a(LibXR::Axis<>::Z(), &ik_root2, ta, &ik_mid2, tzero);
    LibXR::Kinematic::Joint ik_j2b(LibXR::Axis<>::Z(), &ik_mid2, ta, &ik_ep2, tzero);
    ik_j2a.SetState(0.);
    ik_j2b.SetState(0.);
    ik_root2.CalcForward();
    ik_ep2.SetTargetPosition(LibXR::Position(1.5, 0.5, 0.));
    ik_ep2.SetTargetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));
    (void)ik_ep2.CalcBackward(0.1, 3, 1e-4, 0.5);

    LibXR::Kinematic::StartPoint ik_root1(root_inertia);
    LibXR::Kinematic::EndPoint ik_ep1(b_inertia);
    ik_root1.SetPosition(LibXR::Position(0., 0., 0.));
    ik_root1.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));
    LibXR::Kinematic::Joint ik_j1a(LibXR::Axis<>::Z(), &ik_root1, ta, &ik_ep1, tzero);
    ik_j1a.SetState(0.);
    ik_root1.CalcForward();

    // 将两关节末端赋给单关节末端后执行逆运动学，验证不沿用 joint_num_=2 或源雅可比缓存。
    ik_ep1 = ik_ep2;
    TEST_ASSERT(ik_ep1.parent == &ik_j1a);
    ik_ep1.SetTargetPosition(LibXR::Position(0.5, 0.8, 0.));
    auto ik_err1 = ik_ep1.CalcBackward(0.1, 3, 1e-4, 0.5);
    TEST_ASSERT(std::isfinite(ik_err1.norm()));

    // 反向赋值：单关节末端赋给两关节末端，两关节末端须保留自身的 2 关节缓存并可求解 IK。
    ik_ep2 = ik_ep1;
    TEST_ASSERT(ik_ep2.parent == &ik_j2b);
    ik_ep2.SetTargetPosition(LibXR::Position(1.2, 0.9, 0.));
    auto ik_err_rev = ik_ep2.CalcBackward(0.1, 3, 1e-4, 0.5);
    TEST_ASSERT(std::isfinite(ik_err_rev.norm()));

    // 拷贝构造脱离拓扑的 EndPoint，并直接挂接到新的单关节链执行 CalcBackward()：
    // 若拷贝构造误浅拷贝了 ik_ep2 的 joint_num_=2 缓存，此处必在第二层解引用空指针。
    LibXR::Kinematic::EndPoint ik_copy(ik_ep2);
    TEST_ASSERT(ik_copy.parent == nullptr);
    LibXR::Kinematic::StartPoint copy_root(root_inertia);
    copy_root.SetPosition(LibXR::Position(0., 0., 0.));
    copy_root.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));
    LibXR::Kinematic::Joint copy_j(LibXR::Axis<>::Z(), &copy_root, ta, &ik_copy, tzero);
    copy_j.SetState(0.);
    copy_root.CalcForward();
    ik_copy.SetTargetPosition(LibXR::Position(0.6, 0.7, 0.));
    auto ik_err_copy = ik_copy.CalcBackward(0.1, 3, 1e-4, 0.5);
    TEST_ASSERT(std::isfinite(ik_err_copy.norm()));

    // 移动构造脱离拓扑的 EndPoint，并直接挂接到新的单关节链执行 CalcBackward()，
    // 再将其移动赋值回 ik_ep1 验证保留目标既有缓存。
    LibXR::Kinematic::EndPoint ik_moved(std::move(ik_ep2));
    TEST_ASSERT(ik_moved.parent == nullptr);
    LibXR::Kinematic::StartPoint moved_root(root_inertia);
    moved_root.SetPosition(LibXR::Position(0., 0., 0.));
    moved_root.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));
    LibXR::Kinematic::Joint moved_j(LibXR::Axis<>::Z(), &moved_root, ta, &ik_moved,
                                    tzero);
    moved_j.SetState(0.);
    moved_root.CalcForward();
    ik_moved.SetTargetPosition(LibXR::Position(0.8, 0.4, 0.));
    auto ik_err_moved = ik_moved.CalcBackward(0.1, 3, 1e-4, 0.5);
    TEST_ASSERT(std::isfinite(ik_err_moved.norm()));

    ik_ep1 = std::move(ik_moved);
    TEST_ASSERT(ik_ep1.parent == &ik_j1a);
    ik_ep1.SetTargetPosition(LibXR::Position(0.7, 0.7, 0.));
    auto ik_err2 = ik_ep1.CalcBackward(0.1, 3, 1e-4, 0.5);
    TEST_ASSERT(std::isfinite(ik_err2.norm()));
  }

  // 零质量根 + 零质量中间连杆 + 正质量末端，以及全零质量树质心回归测试 /
  // Zero-mass root/intermediate + positive leaf CoM, and all-zero-mass tree CoM.
  {
    LibXR::Inertia zero_inertia;
    LibXR::Inertia leaf_inertia(4.0, 0.1, 0.1, 0.1, 0., 0., 0.);

    LibXR::Kinematic::StartPoint z_root(zero_inertia);
    LibXR::Kinematic::Object z_mid(zero_inertia);
    LibXR::Kinematic::EndPoint z_leaf(leaf_inertia);

    z_root.SetPosition(LibXR::Position(0., 0., 1.));
    z_root.SetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

    LibXR::Transform t1(LibXR::Quaternion(1., 0., 0., 0.), LibXR::Position(1., 0., 0.));
    LibXR::Transform t2(LibXR::Quaternion(1., 0., 0., 0.), LibXR::Position(0., 2., 0.));
    LibXR::Transform t0(LibXR::Quaternion(1., 0., 0., 0.), LibXR::Position(0., 0., 0.));

    LibXR::Kinematic::Joint j1(LibXR::Axis<>::Z(), &z_root, t1, &z_mid, t0);
    LibXR::Kinematic::Joint j2(LibXR::Axis<>::Z(), &z_mid, t2, &z_leaf, t0);
    j1.SetState(0.);
    j2.SetState(0.);

    z_root.CalcForward();
    z_root.CalcCenterOfMass();
    TEST_ASSERT(std::isfinite(z_root.cog.position(0)) &&
                std::isfinite(z_root.cog.position(1)) &&
                std::isfinite(z_root.cog.position(2)));
    TEST_ASSERT(equal(z_root.cog.mass, 4.0));
    TEST_ASSERT(equal(z_root.cog.position(0), 1.0));
    TEST_ASSERT(equal(z_root.cog.position(1), 2.0));
    TEST_ASSERT(equal(z_root.cog.position(2), 1.0));

    // 全零质量树返回零质量且坐标有限（非 NaN）。
    // All-zero-mass tree returns zero mass with finite root position.
    z_leaf.param_.inertia = zero_inertia;
    z_root.CalcCenterOfMass();
    TEST_ASSERT(std::isfinite(z_root.cog.position(0)) &&
                std::isfinite(z_root.cog.position(1)) &&
                std::isfinite(z_root.cog.position(2)));
    TEST_ASSERT(equal(z_root.cog.mass, 0.0));
    TEST_ASSERT(equal(z_root.cog.position(2), 1.0));
  }
}

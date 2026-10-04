/**
 * @file test_transform_euler_orders.cpp
 * @brief 六种欧拉角顺序转换测试 / Conversion tests for all six Euler angle orders.
 *
 * 对每种顺序检查已知旋转矩阵，并核对欧拉角、矩阵与四元数之间的往返转换。
 * Check a known rotation matrix for each order and round trips among angles, matrices and
 * quaternions.
 */

#include "test_assert.hpp"
#include "transform_test_common.hpp"

void RunTransformEulerOrderTests()
{
  TransformTestState state{};
  auto& eulr = state.eulr;
  auto& eulr_new = state.eulr_new;
  auto& rot = state.rot;
  auto& rot_new = state.rot_new;
  auto& quat = state.quat;
  auto& quat_new = state.quat_new;

  /* ZYX Order */
  rot = eulr.ToRotationMatrixZYX();
  TEST_ASSERT(equal(rot(0, 0), 0.6123725) && equal(rot(0, 1), -0.5915064) &&
              equal(rot(0, 2), 0.5245190) && equal(rot(1, 0), 0.6123725) &&
              equal(rot(1, 1), 0.7745190) && equal(rot(1, 2), 0.1584937) &&
              equal(rot(2, 0), -0.5000000) && equal(rot(2, 1), 0.2241439) &&
              equal(rot(2, 2), 0.8365163));

  eulr_new = rot.ToEulerAngleZYX();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionZYX();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleZYX();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  /* ZXY Order */
  rot = eulr.ToRotationMatrixZXY();
  TEST_ASSERT(equal(rot(0, 0), 0.5208661) && equal(rot(0, 1), -0.6830127) &&
              equal(rot(0, 2), 0.5120471) && equal(rot(1, 0), 0.7038788) &&
              equal(rot(1, 1), 0.6830127) && equal(rot(1, 2), 0.1950597) &&
              equal(rot(2, 0), -0.4829629) && equal(rot(2, 1), 0.2588190) &&
              equal(rot(2, 2), 0.8365163));

  eulr_new = rot.ToEulerAngleZXY();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionZXY();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleZXY();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  /* YXZ Order */
  rot = eulr.ToRotationMatrixYXZ();
  TEST_ASSERT(equal(rot(0, 0), 0.7038788) && equal(rot(0, 1), -0.5208661) &&
              equal(rot(0, 2), 0.4829629) && equal(rot(1, 0), 0.6830127) &&
              equal(rot(1, 1), 0.6830127) && equal(rot(1, 2), -0.2588190) &&
              equal(rot(2, 0), -0.1950597) && equal(rot(2, 1), 0.5120471) &&
              equal(rot(2, 2), 0.8365163));

  eulr_new = rot.ToEulerAngleYXZ();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionYXZ();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleYXZ();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  /* XYZ Order */
  rot = eulr.ToRotationMatrixXYZ();
  TEST_ASSERT(equal(rot(0, 0), 0.6123725) && equal(rot(0, 1), -0.6123725) &&
              equal(rot(0, 2), 0.5000000) && equal(rot(1, 0), 0.7745190) &&
              equal(rot(1, 1), 0.5915064) && equal(rot(1, 2), -0.2241439) &&
              equal(rot(2, 0), -0.1584937) && equal(rot(2, 1), 0.5245190) &&
              equal(rot(2, 2), 0.8365163));

  eulr_new = rot.ToEulerAngleXYZ();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionXYZ();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleXYZ();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  /* XZY Order */
  rot = eulr.ToRotationMatrixXZY();
  TEST_ASSERT(equal(rot(0, 0), 0.6123725) && equal(rot(0, 1), -0.7071068) &&
              equal(rot(0, 2), 0.3535534) && equal(rot(1, 0), 0.7209159) &&
              equal(rot(1, 1), 0.6830127) && equal(rot(1, 2), 0.1173625) &&
              equal(rot(2, 0), -0.3244693) && equal(rot(2, 1), 0.1830127) &&
              equal(rot(2, 2), 0.9280227));

  eulr_new = rot.ToEulerAngleXZY();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionXZY();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleXZY();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  /* YZX Order */
  rot = eulr.ToRotationMatrixYZX();
  TEST_ASSERT(equal(rot(0, 0), 0.6123725) && equal(rot(0, 1), -0.4620968) &&
              equal(rot(0, 2), 0.6414565) && equal(rot(1, 0), 0.7071068) &&
              equal(rot(1, 1), 0.6830127) && equal(rot(1, 2), -0.1830127) &&
              equal(rot(2, 0), -0.3535534) && equal(rot(2, 1), 0.5656502) &&
              equal(rot(2, 2), 0.7450100));

  eulr_new = rot.ToEulerAngleYZX();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  quat = LibXR::Quaternion(rot);
  quat_new = eulr.ToQuaternionYZX();
  TEST_ASSERT(equal(quat_new.w(), quat.w()) && equal(quat_new.x(), quat.x()) &&
              equal(quat_new.y(), quat.y()) && equal(quat_new.z(), quat.z()));

  rot_new = quat.ToRotationMatrix();
  TEST_ASSERT(equal(rot_new(0, 0), rot(0, 0)) && equal(rot_new(0, 1), rot(0, 1)) &&
              equal(rot_new(0, 2), rot(0, 2)) && equal(rot_new(1, 0), rot(1, 0)) &&
              equal(rot_new(1, 1), rot(1, 1)) && equal(rot_new(1, 2), rot(1, 2)) &&
              equal(rot_new(2, 0), rot(2, 0)) && equal(rot_new(2, 1), rot(2, 1)) &&
              equal(rot_new(2, 2), rot(2, 2)));

  eulr_new = quat.ToEulerAngleYZX();
  TEST_ASSERT(equal(eulr_new(0), eulr(0)) && equal(eulr_new(1), eulr(1)) &&
              equal(eulr_new(2), eulr(2)));

  // 再用三组互不相等的角度检查六种顺序的往返；上面的 yaw 为 π/4，atan2 两个参数互换时
  // 结果也相同，测不出参数顺序。
  // Round trips of all six orders with three sets of distinct angles; the yaw above is
  // π/4, where swapped atan2 arguments give the same result, so it cannot catch them.
  const LibXR::EulerAngle<> angle_sets[] = {
      {0.3, -0.5, 1.1}, {-0.7, 0.2, -1.3}, {1.2, 0.9, -0.4}};
  for (const auto& angles : angle_sets)
  {
    auto same = [&](const LibXR::EulerAngle<>& got)
    {
      return equal(got(0), angles(0)) && equal(got(1), angles(1)) &&
             equal(got(2), angles(2));
    };

    rot = angles.ToRotationMatrixZYX();
    quat = angles.ToQuaternionZYX();
    TEST_ASSERT(same(rot.ToEulerAngleZYX()) && same(quat.ToEulerAngleZYX()));

    rot = angles.ToRotationMatrixZXY();
    quat = angles.ToQuaternionZXY();
    TEST_ASSERT(same(rot.ToEulerAngleZXY()) && same(quat.ToEulerAngleZXY()));

    rot = angles.ToRotationMatrixYXZ();
    quat = angles.ToQuaternionYXZ();
    TEST_ASSERT(same(rot.ToEulerAngleYXZ()) && same(quat.ToEulerAngleYXZ()));

    rot = angles.ToRotationMatrixXYZ();
    quat = angles.ToQuaternionXYZ();
    TEST_ASSERT(same(rot.ToEulerAngleXYZ()) && same(quat.ToEulerAngleXYZ()));

    rot = angles.ToRotationMatrixXZY();
    quat = angles.ToQuaternionXZY();
    TEST_ASSERT(same(rot.ToEulerAngleXZY()) && same(quat.ToEulerAngleXZY()));

    rot = angles.ToRotationMatrixYZX();
    quat = angles.ToQuaternionYZX();
    TEST_ASSERT(same(rot.ToEulerAngleYZX()) && same(quat.ToEulerAngleYZX()));
  }
}

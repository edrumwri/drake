#include <DR/tools/differential_inverse_kinematics.h>

#include <gtest/gtest.h>

using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::Vector2;
using drake::Vector3;
using drake::VectorX;
using drake::MatrixX;

namespace DR {
namespace {

// This tester is based on a model of a double pendulum of lengths l0() and l1()
// of its first and second links, respectively. The first joint of the pendulum
// is located at (0,0,0), and the pendulum hangs along the negative y-axis when
// in its zero configuration (q=[0 0]). Both joints rotate counter clockwise
// about the positive z-axis. The first element of q gives the angle of the
// first link with respect to the world frame; the second gives the angle of
// the second link with respect to the first link.
class InverseKinematicsTester : public ::testing::Test {
 public:
  // Computes the forward kinematics of the double pendulum.
  RigidTransform<double> ForwardKinematics(const VectorX<double>& q) const {
    DRAKE_DEMAND(q.size() == 2);

    // Set the transformation from joint 0 to the world.
    RigidTransform<double> X_W0j;
    X_W0j.set_rotation(RotationMatrix<double>::MakeZRotation(q[0]));

    // Set the transformation from link 0 to joint 0.
    RigidTransform<double> X_0j0l;
    X_0j0l.set_translation(Vector3<double>(0, -l0_, 0));

    // Set the transformation from joint 1 to link 0.
    RigidTransform<double> X_0l1j;
    X_0l1j.set_rotation(RotationMatrix<double>::MakeZRotation(q[1]));

    // Set the transformation from link 1 to joint 1.
    RigidTransform<double> X_1j1l;
    X_1j1l.set_translation(Vector3<double>(0, -l1_, 0));

    return X_W0j * X_0j0l * X_0l1j * X_1j1l;
  }

  // Computes the generalized position differential that yields the given operational space differential.
  VectorX<double> delta_q_differential(
      const VectorX<double>& q, const VectorX<double>& operational_space_differential) const {
    const MatrixX<double> J = JacobianEE(q);
    const VectorX<double> delta_q = J.colPivHouseholderQr().solve(operational_space_differential);
    return delta_q;
  }

  double l0() const { return l0_; }
  double l1() const { return l1_; }

  // Computes the Jacobian matrix with orientation components on bottom, translation components on top.
  MatrixX<double> JacobianEE(const VectorX<double> q) const {
    DRAKE_DEMAND(q.size() == 2);
    MatrixX<double> J(6, q.size());
    const Vector3<double> ee_location = ForwardKinematics(q).translation();
    const Vector3<double> z(0, 0, 1);

    // Formulas taken from Sciavicco and Siciliano, 2000.
    J.col(0).head<3>() = z.cross(ee_location);
    J.col(0).tail<3>() = z;
    J.col(1).head<3>() = z.cross(ee_location - joint1_location(q));
    J.col(1).tail<3>() = z;
    return J;
  }

 private:
  // Computes the location of the second joint in the world frame.
  Vector3<double> joint1_location(const VectorX<double>& q) const {
    // The second joint is located at the first link endpoint.
    const Vector3<double> link0_ep_0(0, -l0_, 0);
    const RotationMatrix<double> R_W0 = RotationMatrix<double>::MakeZRotation(q[0]);
    return R_W0 * link0_ep_0;
  }

  const double l0_{2.0};  // The length of the first link.
  const double l1_{1.0};  // The length of the second link.
};

// Test three simple double pendulum configurations.
TEST_F(InverseKinematicsTester, PendulumForwardKinematics) {
  // A tight tolerance under which the following tests pass.
  const double tol = 10 * std::numeric_limits<double>::epsilon();

  // TODO(edrumwri) Test the orientation components below. We have some idea
  // that this works properly because IK is working properly. Still, it would
  // be a good idea to test these orientations too.

  Vector2<double> q1;
  q1 << 0, 0;
  const RigidTransform<double> X1 = ForwardKinematics(q1);
  EXPECT_LT((X1.translation() -
      Vector3<double>(0, -(l0() + l1()), 0)).norm(), tol);

  Vector2<double> q2;
  q2 << M_PI_2, M_PI_2;
  const RigidTransform<double> X2 = ForwardKinematics(q2);
  EXPECT_LT((X2.translation() - Vector3<double>(l0(), l1(), 0)).norm(), tol);

  Vector2<double> q3;
  q3 << M_PI_2, 0;
  const RigidTransform<double> X3 = ForwardKinematics(q3);
  EXPECT_LT((X3.translation() -
      Vector3<double>(l0() + l1(), 0, 0)).norm(), tol);
}

// Tests the Jacobian computation against a finite differenced version.
TEST_F(InverseKinematicsTester, Jacobian) {
  // Compute the Jacobian.
  Vector2<double> q;
  q << M_PI_4, M_PI_4;
  const MatrixX<double> J = JacobianEE(q);
  const int nv = J.rows();    // Number of velocity variables.
  const int ndof = J.cols();  // Number of degrees-of-freedom.

  // Compute the Jacobian using forward differencing.
  const double h = 1e-6;      // Forward differencing constant.
  MatrixX<double> J_ndiff(J.rows(), J.cols());
  for (int i = 0; i < ndof; ++i) {
    Vector2<double> qprime = q;
    qprime(i) += h;
    J_ndiff.col(i) = DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(
        ForwardKinematics(q), ForwardKinematics(qprime), false /* angular components on bottom */) / h;
  }

  // Check the result.
  for (int i = 0; i < nv; ++i) {
    for (int j = 0; j < ndof; ++j)
      EXPECT_NEAR(J(i, j), J_ndiff(i, j), h * 10);
  }
}

// Tests the positional differential against a manually computed result.
TEST_F(InverseKinematicsTester, PosDiff) {
  const Vector3<double> v1(1.0, 2.0, 3.0);
  const Vector3<double> v2(5.0, 7.0, 11.0);
  EXPECT_EQ(
      DifferentialInverseKinematics<double>::CalcPositionDifferential(v1, v2),
      Vector3<double>(4.0, 5.0, 8.0));
}

// Tests the orientational differential against a manually computed result.
TEST_F(InverseKinematicsTester, OriDiff) {
  // Note: the accuracy of this test empirically looks to be about O(theta^3).
  const double theta = 1e-2;

  // Approximately the tightest accuracy with which this test will pass for the
  // current value of theta.
  const double tol = 2e-7;

  const auto R1 = RotationMatrix<double>::Identity();
  const auto R2 = RotationMatrix<double>::MakeYRotation(theta);
  const Vector3<double> omega_expected(0, theta, 0);
  EXPECT_LT((DifferentialInverseKinematics<double>::CalcOrientationDifferential(R1, R2) - omega_expected).norm(), tol);
}

// Tests the ability to remove components aligned with a direction of rotation from an angular velocity vector.
TEST_F(InverseKinematicsTester, RemoveAngularVelComponents) {
  const Vector3<double> direction(0, 1.0, 0);
  const Vector3<double> omega(1.0, 2.0, 3.0);
  EXPECT_EQ(DifferentialInverseKinematics<double>::RemoveAngularVelocityComponentsAlongDirection(omega, direction),
      Vector3<double>(1.0, 0.0, 3.0));
}

// Tests the IK solver.
TEST_F(InverseKinematicsTester, Solve) {
  // Set the desired configuration.
  Vector2<double> q_des;
  q_des << M_PI_2, M_PI_2;

  // Set the desired tolerance.
  double tol = 1e-8;

  // Set the starting seed.
  Vector2<double> q_seed;
  q_seed << M_PI_4, M_PI_4;

  // Set the target configuration.
  const RigidTransform<double> X_WT = ForwardKinematics(q_des);

  // Wraps the real operational space function.
  auto opspace_differential = [](
      const VectorX<double>&, /* not dependent upon generalized configuration */
      const RigidTransform<double>& P1,
      const RigidTransform<double>& P2) -> VectorX<double> {
    return DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(
        P1, P2, false /* angular components on bottom */);
  };

  // Wraps the delta q function.
  auto dq = [this](const VectorX<double>& q, const VectorX<double>& operational_space_differential) -> VectorX<double> {
    return delta_q_differential(q, operational_space_differential);
  };

  // Wraps the forward kinematics function.
  auto fkin = [this](const VectorX<double>& q) -> RigidTransform<double> {
    return ForwardKinematics(q);
  };

  // Compute the solution.
  DifferentialInverseKinematics<double> ik;
  const int max_iterations = 100;
  VectorX<double> q_found;
  ASSERT_NO_THROW(q_found = ik.SolveInverseKinematics(
      X_WT, fkin, opspace_differential, dq, q_seed, max_iterations, tol));

  // Verify the solution.
  EXPECT_LT((q_found - q_des).norm(), tol);
}

}  // namespace
}  // namespace DR


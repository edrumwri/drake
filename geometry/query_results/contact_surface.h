#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/mesh_field.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {


/** The %ContactSurface characterizes the intersection of two penetrating
  geometries M and N as a contact surface with a scalar field and a vector
  field.

  <h3> Mathematical Definitions </h3>

  We describe the _contact_ _surface_ Sₘₙ between two compliant bodies M and
  N.  A contact surface is a surface of equilibrium eₘ = eₙ, where eₘ and
  eₙ are the given scalar fields on bodies M and N:

               eₘ : M → ℝ, eₙ : N → ℝ,

               Sₘₙ = { q ∈ M ∩ N : eₘ(r_MQ) = eₙ(r_NQ) },

  where r_MQ and r_NQ are the position vectors of a point q expressed in M's
  frame and B's frame respectively.

  Conceptually we can define the function hₘₙ = eₘ - eₙ as a scalar field on
  M ∩ N:

               hₘₙ : M ∩ N → ℝ,
               hₘₙ(q) = eₘ(r_MQ) - eₙ(r_NQ).

  The function hₘₙ measures the difference between the two scalar fields
  eₘ and eₙ.

  A point q is on the contact surface Sₘₙ if and only if hₘₙ(q) = 0.

  Our data structure for the contact surface does not store hₘₙ, but it stores
  the vector field ∇hₘₙ on the contact surface Sₘₙ:

               ∇hₘₙ : Sₘₙ → ℝ³,
               ∇hₘₙ(q) = ∇eₘ(r_MQ) - ∇eₙ(r_NQ),

  where ∇eₘ : M → ℝ³ is the gradient vector field of eₘ on M, and similarly
  ∇eₙ : N → ℝ³ is the gradient vector field of eₙ on N.

  Mathematically the vector ∇hₘₙ(q) of q ∈ Sₘₙ is orthogonal to the contact
  surface Sₘₙ at q. The vector ∇hₘₙ(q) points in the direction of increasing
  eₘ and decreasing eₙ. It measures how fast eₘ and eₙ deviates from the other.

  <h3> Computational Representation </h3>

  Computationally our data structure represents the contact surface Sₘₙ as a
  triangulated _surface_ _mesh_, which is the division of the contact surface
  into triangular faces that share vertices. See SurfaceMesh for more details.

  We represent the scalar field e() and the vector field grad_h_M() on the
  surface mesh using the class SurfaceMeshField that can evaluate
  the field value at any point on any triangle of the SurfaceMesh.

  At each vertex v, we store the scalar value:

               e(v) = eₘ(r_MV) = eₙ(r_NV)

  and the vector value:

               grad_h_M(v) = ∇hₘₙ(r_MV), expressed in M's frame.

  On each triangle, we provide evaluation of the scalar `e` and the vector
  `grad_h_M` at any point in the triangle.

  Due to discretization, the vector `grad_h_M` at a vertex v is not
  strictly orthogonal to any triangle sharing v.  Orthogonality does improve
  with finer discretization.

  <h3> Local Coordinates </h3>

  We identify a point p in a triangle with vertices v₀, v₁, v₂ using
  _barycentric_ _coordinates_ (b0, b1, b2) by:

               p = b0 * v₀ + b1 * v₁ + b2 * v₂,
               b0 + b1 + b2 = 1, bᵢ ∈ [0,1].

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ContactSurface {
 public:
  /** @name Does not allow copy; implements MoveConstructible, MoveAssignable
   */
  //@{
  ContactSurface(const ContactSurface&) = delete;
  ContactSurface& operator=(const ContactSurface&) = delete;
  ContactSurface(ContactSurface&&) = default;
  ContactSurface& operator=(ContactSurface&&) = default;
  //@}

  /** Constructs a ContactSurface from SurfaceMesh and SurfaceMeshField's. */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<SurfaceMesh<T>> mesh,
                 SurfaceMeshField<T, T>&& e,
                 SurfaceMeshField<Vector3<T>, T>&& grad_h_M)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_(std::move(mesh)),
        e_(std::move(e)),
        grad_h_M_(std::move(grad_h_M)) {}

  /** Returns the geometry id of the body M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of the body N. */
  GeometryId id_N() const { return id_N_; }

  /** Evaluates the scalar field e() at a point P in a triangle.
    The point P is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of P on the triangle.
   */
  T EvaluateE(SurfaceFaceIndex face,
              const typename SurfaceMesh<T>::Barycentric& barycentric) {
    return e_.Evaluate(face, barycentric);
  }

  /** Evaluates the vector field grad_h_M() at a point P on a triangle.
    The point P is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of P on the triangle.
    @retval  The vector ∇h is expressed in M's frame.
   */
  Vector3<T> EvaluateGrad_h_M(
      SurfaceFaceIndex face,
      const typename SurfaceMesh<T>::Barycentric& barycentric) {
    return grad_h_M_.Evaluate(face, barycentric);
  }

  /** Returns the number of triangular faces.
   */
  int num_faces() const { return mesh_->num_faces(); }

  /** Returns the number of vertices.
   */
  int num_vertices() const { return mesh_->num_vertices(); }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_M_;
  /// The id of the second geometry in the contact.
  GeometryId id_N_;

  std::unique_ptr<SurfaceMesh<T>> mesh_;
  // Scalar field `e`.
  SurfaceMeshField<T, T> e_;
  // Vector field ∇hₘₙ, expressed in M's frame.
  SurfaceMeshField<Vector3<T>, T> grad_h_M_;
};

using ContactSurfaced = ContactSurface<double>;

}  // namespace geometry
}  // namespace drake



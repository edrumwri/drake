#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {

/**
  %MeshField represents a field variable defined on a referenced mesh.

  It can evaluate the field value at any position on any element
  of the mesh.

  We store one field value per one vertex of the mesh. Currently we
  support two kinds of values: scalar and 3-d vector.

  @tparam FieldType  a valid Eigen scalar or vector for the field value.
  @tparam MeshType   the kind of the meshes: surface mesh or volume mesh.
*/
template <class FieldType, class MeshType>
class MeshField {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshField)

  /** Constructs a MeshField that references a mesh.
    @param name    The name of the field variable.
    @param values  The field values at each vertex of the mesh.
    @param mesh    The mesh to which this MeshField refers.
    @pre   The number of entries in values is the same as the number of
           vertices of the mesh.
   */
  MeshField(const std::string& name,
            std::vector<FieldType>&& values,
            MeshType* mesh)
  : name_(name), values_(std::move(values)), mesh_(mesh)
  {}

  /** Evaluates the field value at a position on an element.
    @param e The index of the element.
    @param b The barycentric coordinates.
   */
  FieldType Evaluate(const typename MeshType::ElementIndex e,
                     const typename MeshType::Barycentric& b) const {
    const auto& element = mesh_->element(e);
    FieldType value = b[0] * values_[element.vertex(0)];
    for (int i = 1; i < MeshType::kDim + 1; ++i) {
      value += b[i] * values_[element.vertex(i)];
    }
    return value;
  }

 private:
  std::string name_;
  std::vector<FieldType> values_;
  MeshType* mesh_;
};

template <typename FieldValue, typename CoordType>
using SurfaceMeshField = MeshField<FieldValue, SurfaceMesh<CoordType>>;

// TODO(DamrongGuoy): Define VolumeMeshField when we have VolumeMesh by:
//  template <typename FieldValue, typename CoordType>
//  using VolumeMeshField = MeshField<FieldValue, VolumeMesh<CoordType>>;

}  // namespace geometry
}  // namespace drake

//------------------------------------------------------------------------------
/* (The following document is hidden from Doxygen.)

  <h1> Supplemental Document for Future Development </h1>

  This document will be useful when we compute the contact surface as
  the pressure-equilibrium iso-surface between two pressure fields defined on
  two volume meshes.

  <h2> Terminology from Finite Element Method </h2>

  We borrow terminology from Finite Element Method:

      O.C. Zienkiewicz, R.L. Taylor & J.Z. Zhu.
      The Finite Element Method: Its Basis and Fundamentals.
      Chapter 3. Weak Forms and Finite Element Approximation.
      Chapter 6. Shape Functions, Derivatives, and Integration.

  We divide the domain into small regular shaped regions (e.g. triangles or
  tetrahedrons), each of which define a _finite_ _element_ domain, and
  the finite set of points shared between the finite elements define the
  _nodes_.  The division of the domain into elements and nodes is
  called a _finite_ _element_ _mesh_.

  Each of the two compliant bodies M and N is discretized into
  a 3D _volume_ _mesh_ consisting of tetrahedral elements.

  The contact surface is discretized into a 3D _surface_ _mesh_
  consisting of triangular elements. See SurfaceMesh.

  On each finite element E, we have one _shape_ _function_ Nᵢ for each
  node nᵢ of the element,

               Nᵢ : E → ℝ

  and the _finite_ _element_ _approximation_ uᵉ of an arbitrary field
  variable u on the element E is:

               uᵉ(x,y,z) = ∑ Nᵢ(x,y,z) * uᵢ

  where uᵢ is the value of u at the node nᵢ.

  <h3>Example 1. _Linear_ _Triangular_ _Element_</h3>

  A _linear_ _triangular_ _element_ E with three vertices v₀, v₁, v₂
  has its three nodes n₀, n₁, n₂ coincide with the vertices.  Its finite
  element approximation is:

               uᵉ = N₀ * u₀ + N₁ * u₁ + N₂ * u₂

  For a triangular element, it is beneficial to use a map from
  _parent_ _coordinate_ _system_ (L₀, L₁, L₂) (also known as
  _barycentric_ or _area_ _coordinates_) to a position p
  on the triangle:

               p(L₀, L₁, L₂) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂.

  The parent coordinates are constrained by:

               L₀ + L₁ + L₂ = 1, Lᵢ ∈ [0,1].

  Geometrically we can define Lᵢ(p) as the ratio between the area of the
  triangle p, vᵢ₊₁, vᵢ₊₂ and the area of the triangular element E.

  For a linear triangular element, the shape function is the same as the
  parent coordinate function:

               Nᵢ = Lᵢ, i = 0,1,2

  Linear tetrahedral elements are similar.


  <h3>Example 2. _Quadratic_ _Tetrahedral_ _Element_</h3>

  Currently we only use linear elements for contact surfaces; however, we will
  also discuss _quadratic_ _elements_, which might be useful for setting up
  a pressure field in body M that is discretized into a 3D volume mesh
  consisting of tetrahedral elements.

  Similar to a triangular element, a tetrahedral element with vertices
  v₀, v₁, v₂, v₃  has a map from _parent_ _coordinate_ _system_
  (L₀, L₁, L₂, L₃) to a position p in the tetrahedron:

               p(L₀, L₁, L₂, L₃) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂ +  L₃ * v₃,
               L₀ + L₁ + L₂ + L₃ = 1, Lᵢ ∈ [0,1].

  A _quadratic_ _tetrahedral_ _element_ E has 10 nodes. It has one node at
  each of the four vertices and one node at the mid-side of each of the
  six edges:

               nᵢ = vᵢ, i = 0,1,2,3
               nᵢⱼ = (vᵢ + vⱼ)/2, 0 ≤ i < j ≤ 3

  The 10 _shape_ _functions_ of a quadratic tetrahedral element are
  constructed from the parent coordinate function like this:

               Nᵢ = (2 * Lᵢ - 1) * Lᵢ, i = 0,1,2,3
               Nᵢⱼ = 4 Lᵢ * Lⱼ, 0 ≤ i < j ≤ 3

  The _finite_ _element_ _approximation_ uᵉ of a field variable u on the
  above quadratic tetrahedral element E is:

               uᵉ = ∑(Nᵢ * uᵢ) + ∑(Nᵢⱼ * uᵢⱼ), 0 ≤ i ≤ 3; 0 ≤ i < j ≤ 3

  where uᵢ is the value of u at the node nᵢ, and uᵢⱼ is the value of u at the
  mid-side node nᵢⱼ.

  Quadratic triangular elements are similar.
 */



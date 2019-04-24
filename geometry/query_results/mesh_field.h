#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {

/** %MeshField is an abstract class that represents a field variable defined
  on a mesh, inspired by Finite Element Method.

  It can evaluate the field value at any position on any element of the mesh
  using the concepts of shape functions from Finite Element Method.

  <h2> Shape Functions in Finite Element Method </h2>

  We borrow terminology from:

      O.C. Zienkiewicz, R.L. Taylor & J.Z. Zhu.
      The Finite Element Method: Its Basis and Fundamentals.
      Chapter 3. Weak Forms and Finite Element Approximation.
      Chapter 6. Shape Functions, Derivatives, and Integration.

  We divide the domain into small regular shaped regions (e.g. triangles or
  tetrahedrons), each of which define a _finite element_ domain, and
  a finite set of points shared between the finite elements define the
  _nodes_.  The division of the domain into elements and nodes is
  called a _finite element mesh_.

  For hydroelastic-pressure-field contact model, each of the two bodies M and
  N is discretized into a 3-d VolumeMesh consisting of tetrahedral elements,
  and their ContactSurface--a free surface bounding no volume--is discretized
  into a 3-d SurfaceMesh consisting of triangular elements. The term _element_
  refers to a tetrahedron in VolumeMesh and a triangle in SurfaceMesh.

  On each finite element E, we have one _shape function_ Nᵢ for each
  node nᵢ of the element,

               Nᵢ : E → ℝ

  and use Nᵢ to define the _finite element approximation uᵉ_ of a field
  variable u on the element E as:

               uᵉ(x,y,z) = ∑ Nᵢ(x,y,z) * uᵢ

  where uᵢ is the value of u at the node nᵢ.  A specific definition of a
  shape function Nᵢ depends on the shape and order of approximation of a finite
  element E. For example, E could be a triangle, a quadrilateral, a
  tetrahedron, a hexahedron (a brick-like element), etc., with first-order
  (linear) approximation, second-order (quadratic) approximation, etc.

  We give examples of the shape functions below.

  <h3>Example 1. Shape Function of a Linear Triangular Element</h3>

  A _linear triangular element_ E with three vertices v₀, v₁, v₂ has its
  three nodes n₀, n₁, n₂ coincide with the vertices. For brevity, here we
  write vᵢ for both the label of the vertex and also the Cartesian coordinates
  of its position.  Its finite element approximation is:

               uᵉ = N₀ * u₀ + N₁ * u₁ + N₂ * u₂

  For a triangular element, it is beneficial to use a map from the
  _parent coordinate system_ (L₀, L₁, L₂) (also known as
  _barycentric_ or _area coordinates_) to the Cartesian coordinates of a
  position p on the triangle:

               p(L₀, L₁, L₂) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂.

  The parent coordinates are constrained by:

               L₀ + L₁ + L₂ = 1, Lᵢ ∈ [0,1].

  Geometrically we can define Lᵢ(p) as the ratio between the area of the
  triangle p, vᵢ₊₁, vᵢ₊₂ (index modulo 3) and the area of the triangular
  element E.

  For a linear triangular element, the shape function is the same as the
  parent coordinate function:

               Nᵢ = Lᵢ, i = 0,1,2

  Linear tetrahedral elements are similar.


  <h3>Example 2. Shape function of a Quadratic Tetrahedral Element</h3>

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

  <h3> Class hierarchy </h3>
  MeshField is an abstract class with two concrete subclasses:
  MeshFieldLinear and MeshFieldQuadratic, each of which can be defined
  on either a SurfaceMesh or a VolumeMesh.

  @tparam FieldType  a valid Eigen scalar or vector for the field value.
  @tparam MeshType   the kind of the meshes: surface mesh or volume mesh.
*/
template <class FieldType, class MeshType>
class MeshField {
 public:
  /** Evaluates the field value at a position on an element.
    @param e The index of the element.
    @param b The barycentric coordinates.
   */
  virtual FieldType Evaluate(const typename MeshType::ElementIndex e,
                             const typename MeshType::Barycentric& b) const = 0;

 protected:
  MeshField() = default;
  virtual ~MeshField() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshField)
};

/**
 %MeshFieldLinear represents a field variable defined on a finite-element
 simplicial (triangular or tetrahedral) mesh using first-order (linear)
 approximation.

 We store one field value per one vertex of the mesh, and each element
 (triangle or tetrahedron) has (d+1) nodes, where d is the dimension of the
 element.

 @tparam FieldType  a valid Eigen scalar or vector for the field value.
 @tparam MeshType   the kind of the meshes: surface mesh or volume mesh.
 */
template <class FieldType, class MeshType>
class MeshFieldLinear final : public MeshField<FieldType, MeshType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshFieldLinear)

  /** Constructs a MeshFieldLinear.
    @param name    The name of the field variable.
    @param values  The field value at each vertex of the mesh.
    @param mesh    The mesh to which this MeshField refers.
    @pre   The number of entries in values is the same as the number of
           vertices of the mesh.
   */
  MeshFieldLinear(const std::string name, std::vector<FieldType>&& values,
                  MeshType* mesh)
      : name_(name), values_(std::move(values)), mesh_(mesh) {}

  FieldType Evaluate(const typename MeshType::ElementIndex e,
                     const typename MeshType::Barycentric& b) const override {
    const auto& element = mesh_->element(e);
    FieldType value = b[0] * values_[element.vertex(0)];
    for (int i = 1; i < MeshType::kDim + 1; ++i) {
      value += b[i] * values_[element.vertex(i)];
    }
    return value;
  }

 private:
  std::string name_;
  // The field values are indexed in the same way as vertices, i.e.,
  // values_[i] is the field value for the mesh vertices_[i].
  std::vector<FieldType> values_;
  MeshType* mesh_;
};

// TODO(DamrongGuoy): Set up unit tests for MeshFieldQuadratic.

/**
 %MeshFieldQuadratic represents a field variable defined on a finite-element
 simplicial (triangular or tetrahedral) mesh using second-order (quadratic)
 approximation.

 We store one field value per one vertex and one field value per edge of the
 mesh. Each element (triangle or tetrahedron) has (d+1) nodes at its (d+1)
 vertices plus d*(d+1)/2 nodes at the midpoints of its d*(d+1)/2 edges.
 For example, a tetrahedral element as 4 nodes at its 4 vertices plus 6 nodes
 at the midpoints of its 6 edges.

 @tparam FieldType  a valid Eigen scalar or vector for the field value.
 @tparam MeshType   the kind of the meshes: surface mesh or volume mesh.
 */
template <class FieldType, class MeshType>
class MeshFieldQuadratic final : public MeshField<FieldType, MeshType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshFieldQuadratic)

  /** Constructs a MeshFieldQuadratic.
    @param name           The name of the field variable.
    @param vertex_values  The field value at each vertex of the mesh.
    @param edge_values    The field value at the midpoint of each edge of the
                          mesh.
    @param mesh           The mesh to which this MeshField refers.
    @pre   The number of entries in vertex_values equals the number of
           vertices of the mesh.  The number of entries in edge_values equals
           the number of edges of the mesh.
   */
  MeshFieldQuadratic(const std::string name,
                     std::vector<FieldType>&& vertex_values,
                     std::map<SortedPair<int>, FieldType>&& edge_values,
                     MeshType* mesh);

  FieldType Evaluate(const typename MeshType::ElementIndex e,
                     const typename MeshType::Barycentric& b) const override {
    const auto& element = mesh_->element(e);
    FieldType value(0);
    using C = typename MeshType::CoordType;
    for (int i = 0; i < MeshType::kDim + 1; ++i) {
      value += (C(2.0) * b[i] - C(1.0)) * b[i] * vertex_values_[i];
    }
    for (int i = 0; i < MeshType::kDim + 1; ++i) {
      for (int j = i + 1; j < MeshType::kDim + 1; ++j) {
        value += C(4.0) * b[i] * b[j] * edge_values_[SortedPair<int>(i, j)];
      }
    }
    return FieldType(0);
  }

 private:
  std::string name_;
  // The field values are indexed in the same way as vertices, i.e.,
  // vertex_values_[i] is the field value for the mesh vertices_[i].
  std::vector<FieldType> vertex_values_;
  // The field values are indexed in the same way as edges, i.e.,
  // edge_Values_[i] is the field value for the mesh edge between
  // vertices_[i] and vertices_[j].
  std::map<SortedPair<int>, FieldType> edge_values_;
  MeshType* mesh_;
};

// TODO(DamrongGuoy): Define VolumeMeshFieldLinear and/or
//  VolumeMeshFieldQuadratic when we have VolumeMesh like this:
//  template <typename FieldValue, typename T>
//  using VolumeMeshFieldQuadratic =
//      MeshFieldQuadratic<FieldValue, VolumeMesh<T>>;

template <typename FieldType, typename T>
using SurfaceMeshFieldLinear =
    MeshFieldLinear<FieldType, SurfaceMesh<T>>;

}  // namespace geometry
}  // namespace drake


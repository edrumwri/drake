#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/**
 Index used to identify a vertex in a surface mesh.
 */
using SurfaceVertexIndex = TypeSafeIndex<class SurfaceVertexTag>;

/**
 Index for identifying a triangular face in a surface mesh.
 */
using SurfaceFaceIndex = TypeSafeIndex<class SurfaceFaceTag>;

/** %SurfaceVertex represents a vertex in SurfaceMesh of a contact surface
 between bodies M and N.
 @tparam CoordType the underlying scalar type. Must be a valid Eigen scalar.
*/
template <class CoordType>
class SurfaceVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceVertex)

  /** Constructs SurfaceVertex.
   @param r_MV  displacement vector from the origin of M's frame to this
   vertex, expressed in M's frame.
   */
  explicit SurfaceVertex(const Vector3<CoordType>& r_MV)
      : r_MV_(r_MV) {}

  /** Returns the displacement vector from the origin of M's frame to this
   vertex, expressed in M's frame.
   */
  const Vector3<CoordType>& r_MV() const { return r_MV_; }

 private:
  // Displacement vector from the origin of M's frame to this vertex,
  // expressed in M's frame.
  Vector3<CoordType> r_MV_;
};

/** %SurfaceFace represents a triangular face in a SurfaceMesh of a contact
 surface between bodies M and N.
 */
class SurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceFace)

  /** Constructs ContactSurfaceFace.
   @param v0 Index of the first vertex in SurfaceMesh.
   @param v1 Index of the second vertex in SurfaceMesh.
   @param v2 Index of the last vertex in SurfaceMesh.
   @note   The order of the three vertices gives the counterclockwise normal
          direction towards increasing eₘ the scalar field on body M. See
          ContactSurface.
   */
  SurfaceFace(SurfaceVertexIndex v0,
              SurfaceVertexIndex v1,
              SurfaceVertexIndex v2)
      : vertex_({v0, v1, v2}) {}

  /** Constructs ContactSurfaceFace.
   @param v  array of three integer indices of the vertices of the face in
             SurfaceMesh.
   @note   The order of the three vertices gives the counterclockwise normal
          direction towards increasing eₘ the scalar field on body M.
   */
  explicit SurfaceFace(const int v[3])
      : vertex_({SurfaceVertexIndex(v[0]),
                 SurfaceVertexIndex(v[1]),
                 SurfaceVertexIndex(v[2])}) {}

  /** Returns the vertex index in SurfaceMesh of the i-th vertex of this face.
   @param i  The local index of the vertex in this face.
   @pre 0 <= i < 3
   */
  SurfaceVertexIndex vertex(int i) const {
    DRAKE_DEMAND(0 <= i && i < 3);
    return vertex_[i];
  }

 private:
  // The vertices of this face.
  std::array<SurfaceVertexIndex, 3> vertex_;
};

/** %SurfaceMesh represents a triangulated surface of a contact surface.
  A field variable can be defined on SurfaceMesh using SurfaceMeshField.
 @tparam CoordType The underlying scalar type for coordinates, e.g., double
                   or AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class CoordType>
class SurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMesh)

  /**
   @name Interface to MeshField

   The following definitions are needed by a MeshField defined on this
   SurfaceMesh.

   MeshField uses the term _elements_ (inspired by Finite Element Method)
   for _faces_, i.e., triangles, in a triangulated surface mesh. (For a
   tetrahedral volume mesh, the term elements would be for tetrahedrons.)
  */
  //@{

  /**
   A surface mesh has the intrinsic dimension 2.  It is embedded in 3-d space.
   */
  static constexpr int kDim = 2;

  /**
    Index for identifying a vertex.
   */
  using VertexIndex = SurfaceVertexIndex;

  /**
    Index for identifying a triangular element.
   */
  using ElementIndex = SurfaceFaceIndex;

  /**
    Type of barycentric coordinates on a triangular element.
   */
  using Barycentric = Vector<CoordType, kDim + 1>;

  /** Returns the triangular element identified by a given index.
    @param e   The index of the triangular element.
    @pre e ∈ [0, faces_.size()).
   */
  const SurfaceFace& element(ElementIndex e) const {
    DRAKE_DEMAND(0 <= e && e < faces_.size());
    return faces_[e];
  }

  /** Returns the vertex identified by a given index.
    @param v  The index of the vertex.
    @pre v ∈ [0, vertices.size()).
   */
  const SurfaceVertex<CoordType>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < vertices_.size());
    return vertices_[v];
  }

  //@}

  /** Constructs a SurfaceMesh from faces and vertices.
    @param faces     The triangular faces.
    @param vertices  The vertices.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<SurfaceVertex<CoordType>>&& vertices)
  : faces_(std::move(faces)), vertices_(std::move(vertices))
  {}

  /** Returns the number of triangular elements.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns the number of vertices.
   */
  int num_vertices() const { return vertices_.size(); }

 private:
  // The triangles that comprise the surface.
  std::vector<SurfaceFace> faces_;
  // The vertices that are shared between the triangles.
  std::vector<SurfaceVertex<CoordType>> vertices_;
};


}  // namespace geometry
}  // namespace drake


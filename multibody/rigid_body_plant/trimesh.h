#pragma once

#include "drake/multibody/rigid_body_plant/triangle.h"

namespace drake {
namespace multibody {

template <class T>
class Trimesh {
 public:
  Trimesh() {}

  /// Creates the triangle mesh using vertices and indices.
  Trimesh(const std::vector<Vector3<T>>& vertices,
          const std::vector<Eigen::Vector3i>& face_indices) { 
    vertices_ = vertices;
    face_indices_ = face_indices;
    ConstructTriangles(); 
  }

  Trimesh(const Trimesh<T>& t) {
    vertices_ = t.vertices_;
    face_indices_ = t.face_indices_;
    ConstructTriangles(); 
  }

  Trimesh& operator=(const Trimesh<T>& t) {
    vertices_ = t.vertices_;
    face_indices_ = t.face_indices_;
    ConstructTriangles();
    return *this; 
  }

  /// Gets the requisite triangle. Aborts on index out of bound error.
  const Triangle3<T>& triangle(int i) const {
    DRAKE_DEMAND(i < face_indices_.size() && i >= 0); 
    return tris_[i];
  } 

  /// Gets the number of triangles.
  int num_triangles() const { return static_cast<int>(face_indices_.size()); }
 
 private:
  void ConstructTriangles() {
    tris_.clear();
    for (int i = 0; i < static_cast<int>(face_indices_.size()); ++i) {
      const Eigen::Vector3i& f = face_indices_[i];
      DRAKE_DEMAND(f[0] >= 0 && f[0] < vertices_.size());
      DRAKE_DEMAND(f[1] >= 0 && f[1] < vertices_.size());
      DRAKE_DEMAND(f[2] >= 0 && f[2] < vertices_.size());
      const Vector3<T>* a = &vertices_[f[0]];
      const Vector3<T>* b = &vertices_[f[1]];
      const Vector3<T>* c = &vertices_[f[2]];
      tris_.push_back(Triangle3<T>(a, b, c));
    }
  }

  // Vector of triangles.
  std::vector<Triangle3<T>> tris_;

  // Vector of vertices.
  std::vector<Vector3<T>> vertices_;

  // Vector of indices.
  std::vector<Eigen::Vector3i> face_indices_;
};

}  // namespace multibody
}  // namespace drake


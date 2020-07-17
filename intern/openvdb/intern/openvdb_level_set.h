/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 Blender Foundation.
 * All rights reserved.
 */

#ifndef __OPENVDB_LEVEL_SET_H__
#define __OPENVDB_LEVEL_SET_H__

#include "openvdb_capi.h"
#include <openvdb/math/FiniteDifference.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/LevelSetFilter.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

using BoolTreeType = openvdb::FloatGrid::TreeType::template ValueConverter<bool>::Type;

struct CSGOperand {
  unsigned int vert_start;
  unsigned int vert_count;

  unsigned int face_start;
  unsigned int face_count;

  OpenVDBLevelSet_CSGOperation operation;
};

struct OpenVDBLevelSet {
 private:
  openvdb::FloatGrid::Ptr grid;

  // save input data as reference geometry for sharpenfeatures
  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec3s> out_points;
  std::vector<openvdb::Vec4I> out_quads;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<uint32_t> vert_tri;

  // for sharpen features
  bool sharpen_features;
  float edge_tolerance;
  BoolTreeType::Ptr maskTree;
  openvdb::tools::MeshToVoxelEdgeData edgeData;
  openvdb::FloatGrid::Ptr refGrid;

  void sharpenFeaturesPre(float edge_tolerance);

 public:
  OpenVDBLevelSet();
  ~OpenVDBLevelSet();

  std::vector<CSGOperand> csg_operands;

  const openvdb::FloatGrid::Ptr &get_grid();
  const std::vector<openvdb::Vec3s> &get_points();
  const std::vector<openvdb::Vec3s> &get_out_points();
  const std::vector<openvdb::Vec4I> &get_out_quads();
  const std::vector<openvdb::Vec3I> &get_triangles();
  const std::vector<uint32_t> &get_vert_tri();

  void set_grid(const openvdb::FloatGrid::Ptr &grid);
  void set_points(const std::vector<openvdb::Vec3s> &points);
  void set_out_points(const std::vector<openvdb::Vec3s> &out_points);
  void set_out_quads(const std::vector<openvdb::Vec4I> &out_quads);
  void set_triangles(const std::vector<openvdb::Vec3I> &triangles);
  void set_vert_tri(const std::vector<uint32_t> &vert_tri);
  openvdb::Vec3s face_normal(uint32_t faceOffset);

  void mesh_to_level_set(const float *vertices,
                         const unsigned int *faces,
                         const unsigned int totvertices,
                         const unsigned int totfaces,
                         const openvdb::math::Transform::Ptr &transform,
                         bool do_convert,
                         bool do_add,
                         OpenVDBLevelSet_CSGOperation op);

  void volume_to_mesh(struct OpenVDBVolumeToMeshData *mesh,
                      const double isovalue,
                      const double adaptivity,
                      const bool relax_disoriented_triangles,
                      struct OpenVDBLevelSet *mask);

  void filter(OpenVDBLevelSet_FilterType filter_type,
              int width,
              int iterations,
              float sigma,
              float distance,
              OpenVDBLevelSet_FilterBias filter_bias,
              bool sharpen_features,
              float edge_tolerance);

  openvdb::FloatGrid::Ptr CSG_operation_apply(const openvdb::FloatGrid::Ptr &gridA,
                                              const openvdb::FloatGrid::Ptr &gridB,
                                              OpenVDBLevelSet_CSGOperation operation);

  void particles_to_level_set(ParticleList part_list,
                              float min_radius,
                              bool trail,
                              float trail_size);
};

#endif /* __OPENVDB_LEVEL_SET_H__ */

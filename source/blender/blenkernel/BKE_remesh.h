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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 */
#ifndef __BKE_REMESH_H__
#define __BKE_REMESH_H__

/** \file
 * \ingroup bke
 */

#include "DNA_mesh_types.h"

#include "openvdb_capi.h"

/* OpenVDB Voxel Remesher */
void BKE_remesh_voxel_ovdb_mesh_to_level_set(struct OpenVDBLevelSet *level_set,
                                             Mesh *mesh,
                                             struct OpenVDBTransform *transform,
                                             bool do_convert,
                                             bool do_add,
                                             Object *ob_mod,
                                             Object *ob_op,
                                             VoxelMesherModifierOp op);

void BKE_remesh_voxel_ovdb_particles_to_level_set(struct OpenVDBLevelSet *level_set,
                                                  ParticleSystem *psys,
                                                  struct Scene *scene,
                                                  Object *ob,
                                                  struct Depsgraph *depsgraph,
                                                  float scale_factor,
                                                  float vel_factor,
                                                  float min_radius,
                                                  bool trail,
                                                  float trail_size);

Mesh *BKE_remesh_voxel_ovdb_volume_to_mesh_nomain(struct OpenVDBLevelSet *level_set,
                                                  double isovalue,
                                                  double adaptivity,
                                                  bool relax_disoriented_triangles,
                                                  struct OpenVDBLevelSet *mask);

#endif /* __BKE_REMESH_H__ */

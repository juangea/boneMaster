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

/** \file
 * \ingroup bke
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <float.h>
#include <ctype.h>

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "DNA_object_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_particle_types.h"

#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_library.h"
#include "BKE_customdata.h"
#include "BKE_bvhutils.h"
#include "BKE_remesh.h"
#include "BKE_particle.h"
#include "BKE_lattice.h"

#include "DEG_depsgraph_query.h"

#ifdef WITH_OPENVDB
#  include "openvdb_capi.h"
#endif

struct ParticleList;

void BKE_remesh_voxel_ovdb_mesh_to_level_set(struct OpenVDBLevelSet *level_set,
                                             Mesh *mesh,
                                             struct OpenVDBTransform *transform)
{
  BKE_mesh_runtime_looptri_recalc(mesh);
  const MLoopTri *looptri = BKE_mesh_runtime_looptri_ensure(mesh);
  MVertTri *verttri = MEM_callocN(sizeof(*verttri) * BKE_mesh_runtime_looptri_len(mesh),
                                  "remesh_looptri");
  BKE_mesh_runtime_verttri_from_looptri(
      verttri, mesh->mloop, looptri, BKE_mesh_runtime_looptri_len(mesh));

  int totfaces = BKE_mesh_runtime_looptri_len(mesh);
  unsigned int totverts = mesh->totvert;
  float *verts = (float *)MEM_calloc_arrayN(totverts * 3, sizeof(float), "remesh_input_verts");
  unsigned int *faces = (unsigned int *)MEM_calloc_arrayN(
      totfaces * 3, sizeof(unsigned int), "remesh_intput_faces");

  for (int i = 0; i < totverts; i++) {
    MVert mvert = mesh->mvert[i];
    verts[i * 3] = mvert.co[0];
    verts[i * 3 + 1] = mvert.co[1];
    verts[i * 3 + 2] = mvert.co[2];
  }

  for (int i = 0; i < totfaces; i++) {
    MVertTri vt = verttri[i];
    faces[i * 3] = vt.tri[0];
    faces[i * 3 + 1] = vt.tri[1];
    faces[i * 3 + 2] = vt.tri[2];
  }

  OpenVDBLevelSet_mesh_to_level_set(level_set, verts, faces, totverts, totfaces, transform);

  MEM_freeN(verts);
  MEM_freeN(faces);
  MEM_freeN(verttri);
}

static void populate_particle_list(ParticleSystem* psys,
                                   struct ParticleList* part_list,
                                   struct Scene *scene,
                                   Object *ob,
                                   struct Depsgraph *depsgraph)
{
    ParticleSimulationData sim = {0};
    ParticleKey state;
    int p;
    float ob_iquat[4];

    sim.depsgraph = depsgraph;
    sim.scene = scene;
    sim.ob = ob;
    sim.psys = psys;
    mat4_to_quat(ob_iquat, ob->imat);

    psys->lattice_deform_data = psys_create_lattice_deform_data(&sim);

    for (p = 0; p < psys->totpart; p++) {
      float pos[3], vel[3];

      if (psys->particles[p].flag & (PARS_NO_DISP | PARS_UNEXIST)) {
        continue;
      }

      state.time = DEG_get_ctime(depsgraph);

      if (psys_get_particle_state(&sim, p, &state, 0) == 0) {
        continue;
      }

      /* location */
      mul_v3_m4v3(pos, ob->imat, state.co);

      /* velocity */
      copy_v3_v3(vel, state.vel);
      mul_qt_v3(ob_iquat, vel);
      mul_v3_fl(vel, psys->part->normfac);

      OpenVDB_add_particle(part_list, pos, psys->particles[p].size, vel);
    }

    if (psys->lattice_deform_data) {
      end_latt_deform(psys->lattice_deform_data);
      psys->lattice_deform_data = NULL;
    }
}

void BKE_remesh_voxel_ovdb_particles_to_level_set(struct OpenVDBLevelSet *level_set,
                                                  ParticleSystem *psys,
                                                  struct Scene* scene,
                                                  Object* ob,
                                                  struct Depsgraph *depsgraph,
                                                  float scale_factor,
                                                  float vel_factor,
                                                  float min_radius,
                                                  bool trail,
                                                  float trail_size)
{
  /* Generate a particle list and a level set from it */
  struct ParticleList* part_list;

  part_list = OpenVDB_create_part_list(psys->totpart, scale_factor, vel_factor);
  populate_particle_list(psys, part_list, scene, ob, depsgraph);

  OpenVDBLevelSet_particles_to_level_set(level_set, part_list, min_radius, trail, trail_size);
}

Mesh *BKE_remesh_voxel_ovdb_volume_to_mesh_nomain(struct OpenVDBLevelSet *level_set,
                                                  double isovalue,
                                                  double adaptivity,
                                                  bool relax_disoriented_triangles)
{
  struct OpenVDBVolumeToMeshData output_mesh;
  OpenVDBLevelSet_volume_to_mesh(
      level_set, &output_mesh, isovalue, adaptivity, relax_disoriented_triangles);

  Mesh *mesh = BKE_mesh_new_nomain(
      output_mesh.totvertices, 0, output_mesh.totquads + output_mesh.tottriangles, 0, 0);
  int q = output_mesh.totquads;

  for (int i = 0; i < output_mesh.totvertices; i++) {
    float vco[3] = {output_mesh.vertices[i * 3],
                    output_mesh.vertices[i * 3 + 1],
                    output_mesh.vertices[i * 3 + 2]};
    copy_v3_v3(mesh->mvert[i].co, vco);
  }

  for (int i = 0; i < output_mesh.totquads; i++) {
    mesh->mface[i].v4 = output_mesh.quads[i * 4];
    mesh->mface[i].v3 = output_mesh.quads[i * 4 + 1];
    mesh->mface[i].v2 = output_mesh.quads[i * 4 + 2];
    mesh->mface[i].v1 = output_mesh.quads[i * 4 + 3];
  }

  for (int i = 0; i < output_mesh.tottriangles; i++) {
    mesh->mface[i + q].v4 = 0;
    mesh->mface[i + q].v3 = output_mesh.triangles[i * 3];
    mesh->mface[i + q].v2 = output_mesh.triangles[i * 3 + 1];
    mesh->mface[i + q].v1 = output_mesh.triangles[i * 3 + 2];
  }

  BKE_mesh_calc_edges_tessface(mesh);
  BKE_mesh_convert_mfaces_to_mpolys(mesh);
  BKE_mesh_calc_normals(mesh);

  MEM_freeN(output_mesh.quads);
  MEM_freeN(output_mesh.vertices);

  if (output_mesh.tottriangles > 0) {
    MEM_freeN(output_mesh.triangles);
  }

  return mesh;
}

/*caller needs to free returned data */
MLoopCol *BKE_remesh_remap_loop_vertex_color_layer(Mesh *mesh)
{
  MLoopCol *source_color = CustomData_get_layer(&mesh->ldata, CD_MLOOPCOL);
  MLoopCol *remap = NULL;
  if (source_color) {
    int i = 0;
    remap = MEM_calloc_arrayN(mesh->totvert, sizeof(MLoopCol), "oldcolor");
    /*map loopbased storage onto vertices*/
    for (i = 0; i < mesh->totloop; i++) {
      MLoopCol c = source_color[i];
      // printf("COLOR %d %d %d %d %d \n", mesh->mloop[i].v, c.r, c.g, c.b, c.a);
      remap[mesh->mloop[i].v].r = c.r;
      remap[mesh->mloop[i].v].g = c.g;
      remap[mesh->mloop[i].v].b = c.b;
      remap[mesh->mloop[i].v].a = c.a;
    }
  }

  return remap;
}

void BKE_remesh_voxel_reproject_remapped_vertex_paint(Mesh *target, Mesh *source, MLoopCol *remap)
{
  BVHTreeFromMesh bvhtree = {NULL};

  if (remap) {
    MVert *target_verts = target->mvert;
    MLoop *target_loops = target->mloop;
    MLoopCol *target_color = CustomData_add_layer(
        &target->ldata, CD_MLOOPCOL, CD_CALLOC, NULL, target->totloop);
    BKE_bvhtree_from_mesh_get(&bvhtree, source, BVHTREE_FROM_VERTS, 2);

    for (int i = 0; i < target->totloop; i++) {
      float from_co[3];
      BVHTreeNearest nearest;
      nearest.index = -1;
      nearest.dist_sq = FLT_MAX;
      copy_v3_v3(from_co, target_verts[target_loops[i].v].co);
      BLI_bvhtree_find_nearest(
          bvhtree.tree, from_co, &nearest, bvhtree.nearest_callback, &bvhtree);

      if (nearest.index != -1) {
        MLoopCol c = remap[nearest.index];
        // printf("MAPPED %d %d %d %d %d %d \n", i, nearest.index, c.r, c.g, c.b, c.a);
        target_color[i].r = c.r;
        target_color[i].g = c.g;
        target_color[i].b = c.b;
        target_color[i].a = c.a;
      }
      else {
        target_color[i].r = 255;
        target_color[i].g = 255;
        target_color[i].b = 255;
        target_color[i].a = 255;
      }
    }
  }
}

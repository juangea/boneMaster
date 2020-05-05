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
 * The Original Code is Copyright (C) 2011 by Nicholas Bishop.
 */

/** \file
 * \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "BLI_kdtree.h"
#include "BLI_listbase.h"
#include "BLI_math_base.h"
#include "BLI_memarena.h"
#include "BLI_utildefines.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_particle_types.h"
#include "DNA_scene_types.h"

#include "MOD_modifiertypes.h"

#include "BKE_bvhutils.h"
#include "BKE_data_transfer.h"
#include "BKE_deform.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_library.h"
#include "BKE_material.h"
#include "BKE_mball_tessellate.h"
#include "BKE_mesh.h"
#include "BKE_mesh_mapping.h"
#include "BKE_mesh_remap.h"
#include "BKE_mesh_remesh_voxel.h"
#include "BKE_mesh_runtime.h"
#include "BKE_object.h"
#include "BKE_remesh.h"
#include "BKE_report.h"

#include "DEG_depsgraph_query.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef WITH_MOD_REMESH
#  include "BLI_math_vector.h"
#endif

#ifdef WITH_OPENVDB
#  include "openvdb_capi.h"
#endif

static void initData(ModifierData *md)
{
  VoxelMesherModifierData *vmd = (VoxelMesherModifierData *)md;

  vmd->mode = MOD_VOXELMESHER_VOXEL;
  vmd->voxel_size = 0.1f;
  vmd->isovalue = 0.0f;
  vmd->adaptivity = 0.0f;
  vmd->filter_width = 1;
  vmd->filter_bias = OPENVDB_LEVELSET_FIRST_BIAS;
  vmd->filter_type = OPENVDB_LEVELSET_FILTER_NONE;
  vmd->filter_distance = 0.1f;
  vmd->flag |= MOD_VOXELMESHER_LIVE_REMESH;

  vmd->basesize[0] = vmd->basesize[1] = vmd->basesize[2] = 1.0f;
  vmd->thresh = 0.6f;
  vmd->wiresize = 0.4f;
  vmd->rendersize = 0.2f;

  vmd->filter_iterations = 1;
  vmd->filter_sigma = 1.0f;
  vmd->edge_tolerance = 0.1f;
  vmd->psys = 1;

    /* since initData has no Object parameter (sigh) we have to check the initial condition of the
     self object being present in the operand list in each applyModifier call */
  if (vmd->csg_operands.first == NULL) {
    // self object is first in list
    CSGVolume_Object *vcob;
    vmd->active_index = 0;

    vcob = MEM_callocN(sizeof(CSGVolume_Object), "vcob");
    vcob->voxel_size = vmd->voxel_size;
    vcob->flag |= MOD_VOXELMESHER_CSG_OBJECT_ENABLED;
    vcob->flag |= MOD_VOXELMESHER_CSG_SYNC_VOXEL_SIZE;
    vcob->sampler = eVoxelMesherModifierSampler_None;
    vcob->voxel_percentage = 100.0f;
    vcob->object = NULL;
    vcob->operation = eVoxelMesherModifierOp_Union;

    BLI_addtail(&vmd->csg_operands, vcob);

    vcob->input = MOD_VOXELMESHER_VERTICES;
    vcob->pflag = 1;
    vcob->psys = 1;

    vcob->part_trail_size = 1.0f;
    vcob->part_min_radius = 0.1f;
    vcob->part_scale_factor = 2.0f;
    vcob->part_vel_factor = 0.25f;
  }
}

#ifdef WITH_MOD_REMESH

static int get_particle_data(VoxelMesherModifierData *vmd,
                             ParticleSystem *psys,
                             Object *ob,
                             float (**pos)[3],
                             float **size,
                             float (**vel)[3],
                             float (**rot)[4],
                             int **index,
                             MemArena *pardata)
{
  // take alive, for now
  ParticleData *pa;
  int i = 0, j = 0;
  float imat[4][4], quat[4];
  invert_m4_m4(imat, ob->obmat);
  mat4_to_quat(quat, imat);

  (*pos) = BLI_memarena_calloc(pardata, sizeof(float) * 3 * psys->totpart);
  (*size) = BLI_memarena_calloc(pardata, sizeof(float) * psys->totpart);
  (*vel) = BLI_memarena_calloc(pardata, sizeof(float) * 3 * psys->totpart);
  (*rot) = BLI_memarena_calloc(pardata, sizeof(float) * 4 * psys->totpart);
  (*index) = BLI_memarena_calloc(pardata, sizeof(int) * psys->totpart);

  for (i = 0; i < psys->totpart; i++) {
    float co[3], rt[4];
    pa = psys->particles + i;
    if (pa->alive == PARS_UNBORN && (vmd->pflag & eVoxelMesherFlag_Unborn) == 0)
      continue;
    if (pa->alive == PARS_ALIVE && (vmd->pflag & eVoxelMesherFlag_Alive) == 0)
      continue;
    if (pa->alive == PARS_DEAD && (vmd->pflag & eVoxelMesherFlag_Dead) == 0)
      continue;

    mul_v3_m4v3(co, imat, pa->state.co);
    copy_v3_v3((*pos)[j], co);
    (*size)[j] = pa->size;

    copy_v3_v3(co, pa->state.vel);
    // mul_m4_v3(imat, co);
    mul_qt_v3(quat, co);
    copy_v3_v3((*vel)[j], co);

    copy_qt_qt(rt, pa->state.rot);
    mul_qt_qtqt(rt, rt, quat);
    copy_qt_qt((*rot)[j], rt);

    (*index)[j] = i;
    j++;
  }

  return j;
}

static ParticleSystem *get_psys(VoxelMesherModifierData *vmd,
                                CSGVolume_Object *vcob,
                                Object* ob,
                                Scene *scene,
                                bool render)
{
  ParticleSystem *psys;
  ModifierData *ob_md = (ModifierData *)vmd;
  int psys_index = vmd->psys;
  Object *obj = ob;

  if (vmd->mode == MOD_VOXELMESHER_VOXEL && vcob) {
    psys_index = vcob->psys;

    if (vcob->object)
      obj = vcob->object;
  }

  psys = BLI_findlink(&obj->particlesystem, psys_index - 1);
  if (psys == NULL)
    return NULL;

  /* If the psys modifier is disabled we cannot use its data.
   * First look up the psys modifier from the object, then check if it is enabled.
   */

  for (ob_md = obj->modifiers.first; ob_md; ob_md = ob_md->next) {
    if (ob_md->type == eModifierType_ParticleSystem) {
      ParticleSystemModifierData *psmd = (ParticleSystemModifierData *)ob_md;
      if (psmd->psys == psys) {
        int required_mode;

        if (render)
          required_mode = eModifierMode_Render;
        else
          required_mode = eModifierMode_Realtime;

        if (modifier_isEnabled(scene, ob_md, required_mode)) {
          return psys;
        }

        return NULL;
      }
    }
  }

  return NULL;
}

static Mesh *repolygonize(
    VoxelMesherModifierData *vmd, Object *ob, Mesh *derived, ParticleSystem *psys, bool render)
{
  Mesh *dm = NULL, *result = NULL;
  MVert *mv = NULL, *mv2 = NULL;
  float(*pos)[3] = NULL, (*vel)[3] = NULL, (*rot)[4] = NULL;
  float *size = NULL, *psize = NULL;
  int i = 0, n = 0, *index, *orig_index;
  bool override_size = vmd->pflag & eVoxelMesherFlag_Size;
  bool verts_only = vmd->pflag & eVoxelMesherFlag_Verts;
  MDeformVert *dvert = NULL;
  int defgrp_size = -1;

  if (vmd->size_defgrp_name[0]) {
    defgrp_size = BKE_object_defgroup_name_index(ob, vmd->size_defgrp_name);
    dvert = CustomData_get_layer(&derived->vdata, CD_MDEFORMVERT);
  }

  if (((vmd->input & MOD_VOXELMESHER_VERTICES) == 0) && (vmd->input & MOD_VOXELMESHER_PARTICLES)) {
    // particles only
    MemArena *pardata = NULL;
    if (psys == NULL)
      return derived;

    pardata = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "pardata");

    n = get_particle_data(vmd, psys, ob, &pos, &size, &vel, &rot, &index, pardata);
    dm = BKE_mesh_new_nomain(n, 0, 0, 0, 0);
    mv = dm->mvert;
    psize = CustomData_add_layer_named(&dm->vdata, CD_PROP_FLT, CD_CALLOC, NULL, n, "psize");
    orig_index = CustomData_add_layer(&dm->vdata, CD_ORIGINDEX, CD_CALLOC, NULL, n);

    for (i = 0; i < n; i++) {
      copy_v3_v3(mv[i].co, pos[i]);
      psize[i] = size[i];
      orig_index[i] = index[i];
    }

    if (verts_only) {
      BLI_memarena_free(pardata);
      return dm;
    }
    else {
      BLI_memarena_free(pardata);
      result = BKE_repolygonize_dm(dm,
                                   vmd->thresh,
                                   vmd->basesize,
                                   vmd->wiresize,
                                   vmd->rendersize,
                                   render,
                                   override_size,
                                   defgrp_size);
      BKE_mesh_free(dm);
      MEM_freeN(dm);
      return result;
    }
  }
  else if ((vmd->input & MOD_VOXELMESHER_VERTICES) &&
           ((vmd->input & MOD_VOXELMESHER_PARTICLES) == 0)) {
    // verts only
    result = BKE_repolygonize_dm(derived,
                                 vmd->thresh,
                                 vmd->basesize,
                                 vmd->wiresize,
                                 vmd->rendersize,
                                 render,
                                 override_size,
                                 defgrp_size);
    return result;
  }
  else if ((vmd->input & MOD_VOXELMESHER_VERTICES) && (vmd->input & MOD_VOXELMESHER_PARTICLES)) {
    // both, for simplicity only use vert data here
    n = 0;
    MemArena *pardata = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "pardata");
    MDeformVert *dvert_new = NULL;

    if (psys)
      n = get_particle_data(vmd, psys, ob, &pos, &size, &vel, &rot, &index, pardata);

    dm = BKE_mesh_new_nomain(n + derived->totvert, 0, 0, 0, 0);
    psize = CustomData_add_layer_named(
        &dm->vdata, CD_PROP_FLT, CD_CALLOC, NULL, n + derived->totvert, "psize");

    orig_index = CustomData_add_layer(
        &dm->vdata, CD_ORIGINDEX, CD_CALLOC, NULL, n + derived->totvert);

    if (dvert && defgrp_size > -1) {
      dvert_new = CustomData_add_layer(
          &dm->vdata, CD_MDEFORMVERT, CD_CALLOC, NULL, n + derived->totvert);
    }

    mv = dm->mvert;
    mv2 = derived->mvert;

    for (i = 0; i < n; i++) {
      copy_v3_v3(mv[i].co, pos[i]);
      psize[i] = size[i];

      orig_index[i] = index[i];

      if (dvert_new && dvert && defgrp_size > -1) {
        BKE_defvert_add_index_notest(dvert_new + i, defgrp_size, 1.0f);
      }
    }

    for (i = n; i < n + derived->totvert; i++) {
      copy_v3_v3(mv[i].co, mv2[i - n].co);
      psize[i] = -1.0f;  // use mball sizep
      orig_index[i] = i;
      if (dvert_new && dvert && defgrp_size > -1) {
        int ind = i - n;
        MDeformWeight *dw = (dvert + ind)->dw;
        float w = 1.0f;
        if (dw) {
          w = dw[defgrp_size].weight;
        }

        BKE_defvert_add_index_notest(dvert_new + i, defgrp_size, w);
      }
    }

    if (verts_only) {
      BLI_memarena_free(pardata);
      return dm;
    }
    else {
      BLI_memarena_free(pardata);
      result = BKE_repolygonize_dm(dm,
                                   vmd->thresh,
                                   vmd->basesize,
                                   vmd->wiresize,
                                   vmd->rendersize,
                                   render,
                                   override_size,
                                   defgrp_size);
      BKE_mesh_free(dm);
      MEM_freeN(dm);
      return result;
    }
  }

  return NULL;
}

#  ifdef WITH_OPENVDB

static Mesh *voxel_remesh(VoxelMesherModifierData *vmd,
                          Mesh *UNUSED(mesh),
                          struct OpenVDBLevelSet *level_set)
{
  Mesh *target = NULL;

  OpenVDBLevelSet_filter(level_set,
                         vmd->filter_type,
                         vmd->filter_width,
                         vmd->filter_iterations,
                         vmd->filter_sigma,
                         vmd->filter_distance,
                         vmd->filter_bias,
                         vmd->flag & MOD_VOXELMESHER_SHARPEN_FEATURES,
                         vmd->edge_tolerance);

  target = BKE_remesh_voxel_ovdb_volume_to_mesh_nomain(level_set,
                                                       vmd->isovalue * vmd->voxel_size,
                                                       vmd->adaptivity,
                                                       vmd->flag & MOD_VOXELMESHER_RELAX_TRIANGLES,
                                                       vmd->levelset_cached);
  OpenVDBLevelSet_free(level_set);

  if (vmd->flag & MOD_VOXELMESHER_SMOOTH_NORMALS) {
    MPoly *mpoly = target->mpoly;
    int i, totpoly = target->totpoly;

    /* Apply smooth shading to output faces */
    for (i = 0; i < totpoly; i++) {
      mpoly[i].flag |= ME_SMOOTH;
    }
  }

  return target;
}

static struct OpenVDBLevelSet *csgOperation(struct OpenVDBLevelSet *level_set,
                                            CSGVolume_Object *vcob,
                                            Object *ob,
                                            VoxelMesherModifierData *vmd,
                                            ModifierEvalContext *ctx)
{
  short type = vcob->object->type;
  Mesh *me_orig = NULL;
  Mesh *me = NULL;
  float imat[4][4];
  float omat[4][4];
  float size;

  size = (vcob->flag & MOD_VOXELMESHER_CSG_VOXEL_PERCENTAGE) ?
             vmd->voxel_size * vcob->voxel_percentage / 100.0f :
             vcob->voxel_size;

  if (vcob->input & MOD_VOXELMESHER_VERTICES) {

    switch (type) {
      case OB_MESH:
        // me_orig = BKE_object_get_evaluated_mesh(depsgraph, vcob->object);
        me_orig = BKE_object_get_evaluated_mesh(vcob->object);
        if (me_orig) {
          me = BKE_mesh_new_nomain(me_orig->totvert,
                                   me_orig->totedge,
                                   me_orig->totface,
                                   me_orig->totloop,
                                   me_orig->totpoly);
          BKE_mesh_nomain_to_mesh(me_orig, me, vcob->object, &CD_MASK_MESH, false);

          BKE_remesh_voxel_ovdb_mesh_to_level_set(
              level_set, me, NULL, false, true, ob, vcob->object, vcob->operation);
        }
        else {
          return level_set;
        }
        break;
      case OB_FONT:
      case OB_CURVE:
      case OB_SURF:
        me = BKE_mesh_new_from_object(NULL, vcob->object, true);
        if (!me) {
          return level_set;
        }
        else {
          BKE_remesh_voxel_ovdb_mesh_to_level_set(
              level_set, me, NULL, false, true, ob, vcob->object, vcob->operation);
        }
        break;

      default:
        return level_set;
    }

    invert_m4_m4(imat, ob->obmat);
    mul_m4_m4m4(omat, imat, vcob->object->obmat);

    for (int i = 0; i < me->totvert; i++) {
      mul_m4_v3(omat, me->mvert[i].co);
    }
  }

  struct OpenVDBTransform *xform = OpenVDBTransform_create();
  OpenVDBTransform_create_linear_transform(xform, size);
  struct OpenVDBLevelSet *level_setB = NULL;

  if (vcob->input & MOD_VOXELMESHER_VERTICES) {
    level_setB = OpenVDBLevelSet_create(false, 0.0f, 0.0f);
    BKE_remesh_voxel_ovdb_mesh_to_level_set(level_setB, me, xform, true, false, NULL, NULL, 0);
  }

  if (vcob->input & MOD_VOXELMESHER_PARTICLES) {
    bool render = ctx->flag & MOD_APPLY_RENDER;
    Scene *scene = DEG_get_input_scene(ctx->depsgraph);
    ParticleSystem *psys = get_psys(vmd, vcob, vcob->object, scene, render);
    if (!level_setB) {
      level_setB = OpenVDBLevelSet_create(true, vcob->voxel_size, 3.0f);
    }

    if (psys) {

      BKE_remesh_voxel_ovdb_particles_to_level_set(level_setB,
                                                   psys,
                                                   scene,
                                                   ob,
                                                   ctx->depsgraph,
                                                   vcob->part_scale_factor,
                                                   vcob->part_vel_factor,
                                                   vcob->part_min_radius,
                                                   vcob->part_trail,
                                                   vcob->part_trail_size);
    }
  }

  if (vcob->sampler != OPENVDB_LEVELSET_GRIDSAMPLER_NONE) {
    level_set = OpenVDBLevelSet_transform_and_resample(
        level_set, level_setB, vcob->sampler, vmd->isovalue);
  }

  OpenVDBLevelSet_CSG_operation(
      level_set, level_set, level_setB, (OpenVDBLevelSet_CSGOperation)vcob->operation);

  OpenVDBLevelSet_free(level_setB);
  OpenVDBTransform_free(xform);
  if (me) {
    BKE_mesh_free(me);
    MEM_freeN(me);
  }

  return level_set;
}

static Mesh *copy_mesh(Mesh *me)
{
  Mesh *result = BKE_mesh_new_nomain(
      me->totvert, me->totedge, me->totface, me->totloop, me->totpoly);

  BKE_mesh_nomain_to_mesh(me, result, NULL, &CD_MASK_MESH, false);
  return result;
}
#  endif

static void remesh_copy_customdata(CustomData *src,
                                   CustomData *dst,
                                   CustomDataMask mask,
                                   int src_ofs,
                                   int dst_ofs,
                                   int copyelem,
                                   int totelem)
{
  CustomDataLayer *layer;
  int i;
  for (i = 0; i < src->totlayer; i++) {
    layer = src->layers + i;
    if (mask & CD_TYPE_AS_MASK(layer->type)) {
      if (!CustomData_get_layer_named(dst, layer->type, layer->name)) {
        CustomData_add_layer_named(dst, layer->type, CD_CALLOC, NULL, totelem, layer->name);
      }

      CustomData_copy_data_layer(src,
                                 dst,
                                 i,
                                 CustomData_get_named_layer_index(dst, layer->type, layer->name),
                                 src_ofs,
                                 dst_ofs,
                                 copyelem);
    }
  }
}

static void join_mesh_to_mesh(Mesh *mesh,
                              Mesh *me,
                              int vertstart,
                              int edgestart,
                              int loopstart,
                              int polystart,
                              int num_verts,
                              int num_edges,
                              int num_loops,
                              int num_polys,
                              float mat[4][4])
{
  MPoly *mp;
  MLoop *ml;
  MEdge *e;
  int i;

  memcpy(mesh->mvert + vertstart, me->mvert, me->totvert * sizeof(MVert));
  for (int v = 0; v < me->totvert; v++) {
    mul_m4_v3(mat, mesh->mvert[v + vertstart].co);
  }

  memcpy(mesh->mpoly + polystart, me->mpoly, me->totpoly * sizeof(MPoly));

  for (i = 0, mp = mesh->mpoly + polystart; i < me->totpoly; ++i, ++mp) {
    /* adjust loopstart index */
    mp->loopstart += loopstart;
  }

  memcpy(mesh->mloop + loopstart, me->mloop, me->totloop * sizeof(MLoop));

  for (i = 0, ml = mesh->mloop + loopstart; i < me->totloop; ++i, ++ml) {
    /* adjust vertex index */
    ml->v += vertstart;
    ml->e += edgestart;
  }

  memcpy(mesh->medge + edgestart, me->medge, me->totedge * sizeof(MEdge));

  for (i = 0, e = mesh->medge + edgestart; i < me->totedge; ++i, ++e) {
    /* adjust vertex indices */
    e->v1 += vertstart;
    e->v2 += vertstart;
  }

  remesh_copy_customdata(
      &me->vdata, &mesh->vdata, CD_MASK_DERIVEDMESH.vmask, 0, vertstart, me->totvert, num_verts);
  remesh_copy_customdata(
      &me->edata, &mesh->edata, CD_MASK_DERIVEDMESH.emask, 0, edgestart, me->totedge, num_edges);
  remesh_copy_customdata(
      &me->ldata, &mesh->ldata, CD_MASK_DERIVEDMESH.lmask, 0, loopstart, me->totloop, num_loops);
  remesh_copy_customdata(
      &me->pdata, &mesh->pdata, CD_MASK_DERIVEDMESH.pmask, 0, polystart, me->totpoly, num_polys);
}

static Object *join_mesh_and_operands(VoxelMesherModifierData *vmd,
                                      const ModifierEvalContext *ctx,
                                      Mesh *messh)
{
  float imat[4][4], omat[4][4];
  Mesh *mesh = NULL;
  CSGVolume_Object *vcob;
  int vertstart, polystart, loopstart, edgestart, num_verts, num_polys, num_loops, num_edges;
  vertstart = polystart = loopstart = edgestart = num_verts = num_polys = num_loops = num_edges =
      0;

  num_verts = messh->totvert;
  num_edges = messh->totedge;
  num_loops = messh->totloop;
  num_polys = messh->totpoly;

  for (vcob = vmd->csg_operands.first; vcob; vcob = vcob->next) {
    if (vcob->object && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED)) {
      Mesh *me = BKE_mesh_new_from_object(ctx->depsgraph, vcob->object, true);
      if (me) {
        num_verts += me->totvert;
        num_polys += me->totpoly;
        num_loops += me->totloop;
        num_edges += me->totedge;
        BKE_mesh_free(me);
        MEM_freeN(me);
      }
    }
  }

  mesh = BKE_mesh_new_nomain(num_verts, num_edges, 0, num_loops, num_polys);

  invert_m4_m4(imat, ctx->object->obmat);
  unit_m4(omat);

  join_mesh_to_mesh(mesh,
                    messh,
                    vertstart,
                    edgestart,
                    loopstart,
                    polystart,
                    num_verts,
                    num_edges,
                    num_loops,
                    num_polys,
                    omat);

  vertstart += messh->totvert;
  polystart += messh->totpoly;
  loopstart += messh->totloop;
  edgestart += messh->totedge;

  for (vcob = vmd->csg_operands.first; vcob; vcob = vcob->next) {
    if (vcob->object && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED)) {
      Mesh *me = BKE_mesh_new_from_object(ctx->depsgraph, vcob->object, true);
      if (me) {
        mul_m4_m4m4(omat, imat, vcob->object->obmat);

        join_mesh_to_mesh(mesh,
                          me,
                          vertstart,
                          edgestart,
                          loopstart,
                          polystart,
                          num_verts,
                          num_edges,
                          num_loops,
                          num_polys,
                          omat);

        vertstart += me->totvert;
        polystart += me->totpoly;
        loopstart += me->totloop;
        edgestart += me->totedge;
        BKE_mesh_free(me);
        MEM_freeN(me);
      }
    }
  }

  // force normals calculation here for datatransfer
  mesh->runtime.cd_dirty_poly |= CD_MASK_NORMAL;
  mesh->runtime.cd_dirty_vert |= CD_MASK_NORMAL;

  BKE_mesh_ensure_normals_for_display(mesh);
  BKE_mesh_calc_normals_split(mesh);

  // create a nomain object here, too bad datatransfer needs 2 objects...
  Object *fob = BKE_id_new_nomain(ID_OB, NULL);
  fob->type = OB_MESH;
  fob->data = mesh;
  // fake evaluated mesh here, just because datatransfer doesnt accept a source mesh, sheesh
  fob->runtime.data_orig = (ID *)mesh;
  fob->runtime.data_eval = (ID *)mesh;
  fob->runtime.mesh_deform_eval = mesh;  // seems this is being retrieved by datatransfer

  return fob;
}

static void transfer_materials(Mesh *src, Mesh *dst)
{
  BVHTreeFromMesh bvhtree_src = {
      .nearest_callback = NULL,
  };
  BKE_bvhtree_from_mesh_get(&bvhtree_src, src, BVHTREE_FROM_LOOPTRI, 2);

  const MLoopTri *lt_dst = BKE_mesh_runtime_looptri_ensure(dst);
  int len_dst = BKE_mesh_runtime_looptri_len(dst);

  const MLoopTri *lt_src = BKE_mesh_runtime_looptri_ensure(src);

  for (int i = 0; i < len_dst; i++) {
    float from_co[3] = {0, 0, 0};
    BVHTreeNearest nearest_src;
    nearest_src.index = -1;
    nearest_src.dist_sq = FLT_MAX;
    for (int j = 0; j < 3; j++) {
      MVert mv = dst->mvert[dst->mloop[lt_dst[i].tri[j]].v];
      add_v3_v3(from_co, mv.co);
    }
    mul_v3_fl(from_co, 0.333333f);

    BLI_bvhtree_find_nearest(
        bvhtree_src.tree, from_co, &nearest_src, bvhtree_src.nearest_callback, &bvhtree_src);

    if (nearest_src.index != -1) {
      dst->mpoly[lt_dst[i].poly].mat_nr = src->mpoly[lt_src[nearest_src.index].poly].mat_nr;
    }
  }

  free_bvhtree_from_mesh(&bvhtree_src);
}

static void transfer_data(VoxelMesherModifierData *vmd,
                          const ModifierEvalContext *ctx,
                          Mesh *mesh,
                          Mesh *result)
{
  Scene *sc = DEG_get_evaluated_scene(ctx->depsgraph);

  Object *obs = join_mesh_and_operands(vmd, ctx, mesh);
  int layers_select_src[4];
  int layers_select_dst[4];
  char defgrp[1] = {'\0'};
  int data_types = DT_TYPE_VERT_ALL | DT_TYPE_EDGE_ALL | DT_TYPE_LOOP_ALL | DT_TYPE_POLY_ALL;

  int map_vert_mode = MREMAP_MODE_VERT_POLYINTERP_VNORPROJ;
  int map_edge_mode = MREMAP_MODE_EDGE_EDGEINTERP_VNORPROJ;
  int map_loop_mode = MREMAP_MODE_LOOP_POLYINTERP_LNORPROJ;
  int map_poly_mode = MREMAP_MODE_POLY_POLYINTERP_PNORPROJ;

  CustomData_MeshMasks mask = CD_MASK_BAREMESH;
  ReportList reports;
  BKE_reports_init(&reports, RPT_STORE);

  for (int i = 0; i < DT_MULTILAYER_INDEX_MAX; i++) {
    layers_select_src[i] = DT_LAYERS_ALL_SRC;
    layers_select_dst[i] = DT_LAYERS_NAME_DST;
  }

  BKE_object_data_transfer_dttypes_to_cdmask(data_types, &mask);
  BKE_mesh_remap_calc_source_cddata_masks_from_map_modes(
      map_vert_mode, map_edge_mode, map_loop_mode, map_poly_mode, &mask);

  // craft a matching cd mask too...
  obs->runtime.last_data_mask = mask;

  BKE_object_data_transfer_ex(ctx->depsgraph,
                              sc,
                              obs,
                              ctx->object,
                              result,
                              data_types,
                              false,
                              map_vert_mode,
                              map_edge_mode,
                              map_loop_mode,
                              map_poly_mode,
                              NULL,
                              false,
                              FLT_MAX,
                              0.0f,
                              0.0f,
                              layers_select_src,
                              layers_select_dst,
                              CDT_MIX_TRANSFER,
                              1.0f,
                              defgrp,
                              false,
                              &reports);

  transfer_materials(obs->data, result);

  BKE_mesh_free(obs->data);
  MEM_freeN(obs->data);
  obs->data = NULL;
  MEM_freeN(obs);
}

struct ExtractParticleData {
  int i;
  float (*vel)[3];
  float (*rot)[4];
  float *size;
  float *psize;
  float *velX;
  float *velY;
  float *velZ;
  float *quatX;
  float *quatY;
  float *quatZ;
  float *quatW;
} ExtractParticleData;

static bool extract_particle_data_cb(void *user_data,
                                     int index,
                                     const float UNUSED(co[3]),
                                     float UNUSED(dist_sq))
{
  struct ExtractParticleData *xp = (struct ExtractParticleData *)user_data;
  int k = index;
  int i = xp->i;
  xp->velX[k] = xp->vel[i][0];
  xp->velY[k] = xp->vel[i][1];
  xp->velZ[k] = xp->vel[i][2];
  xp->psize[k] = xp->size[i];

  xp->quatX[k] = xp->rot[i][0];
  xp->quatY[k] = xp->rot[i][1];
  xp->quatZ[k] = xp->rot[i][2];
  xp->quatW[k] = xp->rot[i][3];

  return true;
}

static void transfer_mblur_data(VoxelMesherModifierData *vmd,
                                const ModifierEvalContext *ctx,
                                ParticleSystem *psys,
                                Mesh *result)
{
  KDTree_3d *kdtree;
  float(*pos)[3] = NULL, (*vel)[3] = NULL, (*rot)[4] = NULL;
  float *size = NULL, *psize = NULL, *velX = NULL, *velY = NULL, *velZ = NULL, *quatX = NULL,
        *quatY = NULL, *quatZ = NULL, *quatW = NULL;
  int n = 0, *index, t = result->totvert;
  MemArena *pardata = NULL;
  // KDTreeNearest_3d *nearest = NULL;

  if (psys) {
    kdtree = BLI_kdtree_3d_new(result->totvert);

    for (int i = 0; i < result->totvert; i++) {
      float co[3];
      // mul_v3_m4v3(co, ctx->object->obmat, result->mvert[i].co);
      copy_v3_v3(co, result->mvert[i].co);
      BLI_kdtree_3d_insert(kdtree, i, co);
    }

    BLI_kdtree_3d_balance(kdtree);
  }

  // need the (empty) layers too, in case we dont use the particles (for cycles)
  psize = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "psize");
  velX = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "velX");
  velY = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "velY");
  velZ = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "velZ");

  quatX = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "quatX");
  quatY = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "quatY");
  quatZ = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "quatZ");
  quatW = CustomData_add_layer_named(&result->vdata, CD_PROP_FLT, CD_CALLOC, NULL, t, "quatW");

  if (psys) {
    pardata = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "pardata");
    n = get_particle_data(vmd, psys, ctx->object, &pos, &size, &vel, &rot, &index, pardata);
    struct ExtractParticleData xp;
    xp.psize = psize;
    xp.size = size;
    xp.rot = rot;
    xp.vel = vel;
    xp.velX = velX;
    xp.velY = velY;
    xp.velZ = velZ;
    xp.quatX = quatX;
    xp.quatY = quatY;
    xp.quatZ = quatZ;
    xp.quatW = quatW;

    for (int i = 0; i < n; i++) {
      // int r = BLI_kdtree_3d_range_search(
      //    kdtree, pos[i], &nearest, psys->part->size * vmd->part_scale_factor * 2);
      xp.i = i;
      BLI_kdtree_3d_range_search_cb(kdtree,
                                    pos[i],
                                    psys->part->size /* vmd->part_scale_factor*/ * 4,
                                    extract_particle_data_cb,
                                    &xp);
    }

    BLI_kdtree_3d_free(kdtree);
    BLI_memarena_free(pardata);
  }
}

static Mesh *applyModifier(ModifierData *md, const ModifierEvalContext *ctx, Mesh *mesh)
{
  VoxelMesherModifierData *vmd = (VoxelMesherModifierData *)md;
  Mesh *result = NULL;

  if (vmd->mode == MOD_VOXELMESHER_VOXEL) {
#  if defined WITH_OPENVDB
    CSGVolume_Object *vcob = vmd->csg_operands.first;
    struct OpenVDBLevelSet *level_set = NULL;

    Object *ob_orig = DEG_get_original_object(ctx->object);
    VoxelMesherModifierData *vmd_orig = (VoxelMesherModifierData *)modifiers_findByName(ob_orig,
                                                                                        md->name);

    if (((vmd->flag & MOD_VOXELMESHER_LIVE_REMESH) == 0)) {
      // access mesh cache on ORIGINAL object, cow should not copy / free this over and over again
      if (vmd_orig->mesh_cached) {
        Mesh *ret = copy_mesh(vmd_orig->mesh_cached);
        BKE_mesh_copy_settings(ret, mesh);
        return ret;
      }
    }

    if (vmd->flag & MOD_VOXELMESHER_ACCUMULATE) {
      if (vmd_orig->levelset_cached) {
        level_set = OpenVDBLevelSet_copy(vmd_orig->levelset_cached);
      }
    }

    if (vcob->voxel_size > 0.0f) {

      struct OpenVDBTransform *xform = OpenVDBTransform_create();
      OpenVDBTransform_create_linear_transform(xform, vmd->voxel_size);

      if ((vcob->input & MOD_VOXELMESHER_VERTICES) && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED))
      {
        bool do_convert = false;
        if (!level_set) {
          level_set = OpenVDBLevelSet_create(false, 0.0f, 0.0f);
          do_convert = true;
        }

        if (do_convert || (!do_convert && (vmd->flag & MOD_VOXELMESHER_SHARPEN_FEATURES))) {
          BKE_remesh_voxel_ovdb_mesh_to_level_set(
              level_set, mesh, xform, do_convert, false, NULL, NULL, 0);
        }
      }

      if ((vcob->input & MOD_VOXELMESHER_PARTICLES) && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED))
      {
        bool render = ctx->flag & MOD_APPLY_RENDER;
        Scene *scene = DEG_get_input_scene(ctx->depsgraph);
        Object *ob = ctx->object;  // the same as first csg volume object
        ParticleSystem *psys = get_psys(vmd, vcob, ob, scene, render);
        if (psys) {

          if (!level_set) {
            level_set = OpenVDBLevelSet_create(true, vmd->voxel_size, 3.0f);
          }

          BKE_remesh_voxel_ovdb_particles_to_level_set(level_set,
                                                       psys,
                                                       scene,
                                                       ob,
                                                       ctx->depsgraph,
                                                       vcob->part_scale_factor,
                                                       vcob->part_vel_factor,
                                                       vcob->part_min_radius,
                                                       vcob->part_trail,
                                                       vcob->part_trail_size);
        }
      }

      OpenVDBTransform_free(xform);

      if (!level_set) {
        level_set = OpenVDBLevelSet_create(true, vcob->voxel_size, 3.0f);
      }

      if (level_set) {
        // start with the 2nd object, since the self object has been handled before already
        CSGVolume_Object *start = vmd->csg_operands.first;
        start = start->next;

        for (vcob = start; vcob; vcob = vcob->next) {
          if (vcob->object && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED)) {
            level_set = csgOperation(level_set, vcob, ctx->object, vmd, ctx);
          }
        }

        if (vmd_orig->levelset_cached) {
          OpenVDBLevelSet_free(vmd_orig->levelset_cached);
        }

        vmd_orig->levelset_cached = OpenVDBLevelSet_copy(level_set);

        result = voxel_remesh(vmd_orig, mesh, level_set);
      }

      if (result) {
        // new ... need to copy materials etc on our own
        BKE_mesh_copy_settings(result, mesh);

        // adaptivity can mess up normals, try to recalc them by tagging them as dirty
        if (vmd->adaptivity > 0.0f)
          result->runtime.cd_dirty_vert |= CD_MASK_NORMAL;

        if (vmd->flag & MOD_VOXELMESHER_FIX_POLES) {
          result = BKE_mesh_remesh_voxel_fix_poles(
              result, (vmd->flag & MOD_VOXELMESHER_SHARPEN_FEATURES) == 0);
        }

        {
          CustomData_MeshMasks mask = CD_MASK_DERIVEDMESH;
          CustomData_merge(&mesh->vdata, &result->vdata, mask.vmask, CD_CALLOC, result->totvert);
          CustomData_merge(&mesh->edata, &result->edata, mask.emask, CD_CALLOC, result->totedge);
          CustomData_merge(&mesh->ldata, &result->ldata, mask.lmask, CD_CALLOC, result->totloop);
          CustomData_merge(&mesh->pdata, &result->pdata, mask.pmask, CD_CALLOC, result->totpoly);
        }

        if (vmd->flag & MOD_VOXELMESHER_REPROJECT_DATA) {
          transfer_data(vmd, ctx, mesh, result);
        }

        // map particle velocities and rotations, size
        // just create the CD layers, even without particles;
        // null check happens in transfer_mblur_data
        {
          bool render = ctx->flag & MOD_APPLY_RENDER;
          Scene *scene = DEG_get_input_scene(ctx->depsgraph);
          Object *ob = ctx->object;
          ParticleSystem *psys = get_psys(vmd, NULL, ob, scene, render);
          transfer_mblur_data(vmd, ctx, psys, result);
        }

        // update cache
        if (vmd_orig->mesh_cached) {
          BKE_mesh_free(vmd_orig->mesh_cached);
          MEM_freeN(vmd_orig->mesh_cached);
        }

        // save a copy
        vmd_orig->mesh_cached = copy_mesh(result);
      }

      return result;
    }
    else {
      return mesh;
    }
#  else
    modifier_setError((ModifierData *)vmd,
                      "Built without OpenVDB support, cant execute voxel remesh");
    return mesh;
#  endif
  }

  if (vmd->mode == MOD_VOXELMESHER_METABALL) {
    ParticleSystem *psys = NULL;
    bool render = ctx->flag & MOD_APPLY_RENDER;
    psys = get_psys(vmd, NULL, ctx->object, DEG_get_input_scene(ctx->depsgraph), render);
    result = repolygonize(vmd, ctx->object, mesh, psys, render);

    if (result) {
      transfer_mblur_data(vmd, ctx, psys, result);
    }

    if (result && (vmd->flag & MOD_VOXELMESHER_SMOOTH_SHADING)) {
      MPoly *mpoly = result->mpoly;
      int i, totpoly = result->totpoly;

      /* Apply smooth shading to output faces */
      for (i = 0; i < totpoly; i++) {
        mpoly[i].flag |= ME_SMOOTH;
      }
    }

    if (result)
    {
      BKE_mesh_copy_settings(result, mesh);
      BKE_mesh_calc_edges(result, true, false);
      result->runtime.cd_dirty_vert |= CD_MASK_NORMAL;
      return result;
    }

    return mesh;
  }
  
  return mesh;
}

#else /* !WITH_MOD_REMESH */

static Mesh *applyModifier(ModifierData *UNUSED(md),
                           const ModifierEvalContext *UNUSED(ctx),
                           Mesh *mesh)
{
  return mesh;
}

#endif /* !WITH_MOD_REMESH */

static void foreachObjectLink(ModifierData *md, Object *ob, ObjectWalkFunc walk, void *userData)
{
  VoxelMesherModifierData *vmd = (VoxelMesherModifierData *)md;
  CSGVolume_Object *vcob;

  if (vmd->mode != MOD_VOXELMESHER_VOXEL) {
    return;
  }

  if (((vmd->flag & MOD_VOXELMESHER_LIVE_REMESH) == 0) && vmd->mesh_cached) {
    return;
  }

  for (vcob = vmd->csg_operands.first; vcob; vcob = vcob->next) {
    if (vcob->object) {
      walk(userData, ob, &vcob->object, IDWALK_CB_NOP);
    }
  }
}

static void updateDepsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  VoxelMesherModifierData *vmd = (VoxelMesherModifierData *)md;
  CSGVolume_Object *vcob;

  if (vmd->mode != MOD_VOXELMESHER_VOXEL) {
    return;
  }

  if (((vmd->flag & MOD_VOXELMESHER_LIVE_REMESH) == 0) && vmd->mesh_cached) {
    return;
  }

  for (vcob = vmd->csg_operands.first; vcob; vcob = vcob->next) {
    if (vcob->object && (vcob->flag & MOD_VOXELMESHER_CSG_OBJECT_ENABLED)) {
      DEG_add_object_relation(
          ctx->node, vcob->object, DEG_OB_COMP_TRANSFORM, "VoxelMesher Modifier");
      DEG_add_object_relation(
          ctx->node, vcob->object, DEG_OB_COMP_GEOMETRY, "VoxelMesher Modifier");
    }
  }

  if (vmd->csg_operands.first) {
    /* We need own transformation as well in case we have operands */
    DEG_add_modifier_to_transform_relation(ctx->node, "VoxelMesher Modifier");
  }
}

static void copyData(const ModifierData *md_src, ModifierData *md_dst, const int flag)
{
  VoxelMesherModifierData *vmd_src = (VoxelMesherModifierData *)md_src;
  VoxelMesherModifierData *vmd_dst = (VoxelMesherModifierData *)md_dst;
  Mesh *me_src = vmd_src->mesh_cached;
  struct OpenVDBLevelSet *lvl_src = vmd_src->levelset_cached;

  modifier_copyData_generic(md_src, md_dst, flag);
  BLI_duplicatelist(&vmd_dst->csg_operands, &vmd_src->csg_operands);

  if (me_src) {
    Mesh *me_dst = BKE_mesh_new_nomain(
        me_src->totvert, me_src->totedge, me_src->totface, me_src->totloop, me_src->totpoly);

    BKE_mesh_nomain_to_mesh(me_src, me_dst, NULL, &CD_MASK_MESH, false);
    vmd_dst->mesh_cached = me_dst;
  }

  if (lvl_src) {
    struct OpenVDBLevelSet *lvl_dst = OpenVDBLevelSet_copy(lvl_src);
    vmd_dst->levelset_cached = lvl_dst;
  }
}

static void freeData(ModifierData *md)
{
  VoxelMesherModifierData *vmd = (VoxelMesherModifierData *)md;

  if (vmd->mesh_cached) {
    BKE_mesh_free(vmd->mesh_cached);
    MEM_freeN(vmd->mesh_cached);
    vmd->mesh_cached = NULL;
  }

  if (vmd->levelset_cached) {
    OpenVDBLevelSet_free(vmd->levelset_cached);
    vmd->levelset_cached = NULL;
  }

  BLI_freelistN(&vmd->csg_operands);
}

ModifierTypeInfo modifierType_VoxelMesher = {
    /* name */ "Voxel Mesher",
    /* structName */ "VoxelMesherModifierData",
    /* structSize */ sizeof(VoxelMesherModifierData),
    /* type */ eModifierTypeType_Constructive,
    /* flags */ eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_AcceptsCVs |
        eModifierTypeFlag_SupportsEditmode | eModifierTypeFlag_SupportsMapping,

    /* copyData */ copyData,

    /* deformVerts */ NULL,
    /* deformMatrices */ NULL,
    /* deformVertsEM */ NULL,
    /* deformMatricesEM */ NULL,
    /* applyModifier */ applyModifier,

    /* initData */ initData,
    /* requiredDataMask */ NULL,
    /* freeData */ freeData,
    /* isDisabled */ NULL,
    /* updateDepsgraph */ updateDepsgraph,
    /* dependsOnTime */ NULL,
    /* dependsOnNormals */ NULL,
    /* foreachObjectLink */ foreachObjectLink,
    /* foreachIDLink */ NULL,
    /* freeRuntimeData */ NULL,
};

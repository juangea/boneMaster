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
 */

/** \file
 * \ingroup modifiers
 */

#include <string.h>

#include "BLI_math_vector.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_cachefile_types.h"
#include "DNA_defaults.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "MEM_guardedalloc.h"

#include "BKE_cachefile.h"
#include "BKE_context.h"
#include "BKE_curve_to_mesh.hh"
#include "BKE_geometry_set.hh"
#include "BKE_lib_query.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
#include "BKE_spline.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "RNA_access.h"

#include "BLO_read_write.h"

#include "DEG_depsgraph_build.h"
#include "DEG_depsgraph_query.h"

#include "MOD_modifiertypes.h"
#include "MOD_ui_common.h"

#if defined(WITH_USD) || defined(WITH_ALEMBIC)
#  include "BKE_global.h"
#  include "BKE_lib_id.h"
#endif

#ifdef WITH_ALEMBIC
#  include "ABC_alembic.h"
#endif

#ifdef WITH_USD
#  include "usd.h"
#endif

static void initData(ModifierData *md)
{
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(mcmd, modifier));

  mcmd->cache_file = NULL;
  mcmd->object_path[0] = '\0';
  mcmd->read_flag = MOD_MESHSEQ_READ_ALL;

  MEMCPY_STRUCT_AFTER(mcmd, DNA_struct_default_get(MeshSeqCacheModifierData), modifier);
}

static void copyData(const ModifierData *md, ModifierData *target, const int flag)
{
#if 0
  const MeshSeqCacheModifierData *mcmd = (const MeshSeqCacheModifierData *)md;
#endif
  MeshSeqCacheModifierData *tmcmd = (MeshSeqCacheModifierData *)target;

  BKE_modifier_copydata_generic(md, target, flag);

  tmcmd->reader = nullptr;
  tmcmd->reader_object_path[0] = '\0';
}

static void freeData(ModifierData *md)
{
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  if (mcmd->reader) {
    mcmd->reader_object_path[0] = '\0';
    BKE_cachefile_reader_free(mcmd->cache_file, &mcmd->reader);
  }
}

static bool isDisabled(const struct Scene *UNUSED(scene),
                       ModifierData *md,
                       bool UNUSED(useRenderParams))
{
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  /* leave it up to the modifier to check the file is valid on calculation */
  return (mcmd->cache_file == nullptr) || (mcmd->object_path[0] == '\0');
}

/* If this invocation is for the ORCO mesh, and the mesh hasn't changed topology, we
 * must return the mesh as-is instead of deforming it. */
static bool need_orco_mesh(MeshSeqCacheModifierData *mcmd,
                           const ModifierEvalContext *ctx,
                           Mesh *mesh,
                           float time,
                           const char **err_str)
{
  CacheFile *cache_file = mcmd->cache_file;

  if (ctx->flag & MOD_APPLY_ORCO) {
    switch (cache_file->type) {
      case CACHEFILE_TYPE_ALEMBIC:
#ifdef WITH_ALEMBIC
        if (!ABC_mesh_topology_changed(mcmd->reader, ctx->object, mesh, time, err_str)) {
          return true;
        }
#endif
        break;
      case CACHEFILE_TYPE_USD:
#ifdef WITH_USD
        if (!USD_mesh_topology_changed(mcmd->reader, ctx->object, mesh, time, err_str)) {
          return true;
        }
#endif
        break;
      case CACHE_FILE_TYPE_INVALID:
        break;
    }
  }

  return false;
}

static Mesh *generate_bounding_box_mesh(Object *object, Mesh *org_mesh)
{
  BoundBox *bb = BKE_object_boundbox_get(object);
  Mesh *result = BKE_mesh_new_nomain_from_template(org_mesh, 8, 0, 0, 24, 6);

  MVert *mvert = result->mvert;
  for (int i = 0; i < 8; ++i) {
    copy_v3_v3(mvert[i].co, bb->vec[i]);
  }

  /* See DNA_object_types.h for the diagram showing the order of the vertices for a BoundBox. */
  static unsigned int loops_v[6][4] = {
      {0, 4, 5, 1},
      {4, 7, 6, 5},
      {7, 3, 2, 6},
      {3, 0, 1, 2},
      {1, 5, 6, 2},
      {3, 7, 4, 0},
  };

  MLoop *mloop = result->mloop;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 4; ++j, ++mloop) {
      mloop->v = loops_v[i][j];
    }
  }

  MPoly *mpoly = result->mpoly;
  for (int i = 0; i < 6; ++i) {
    mpoly[i].loopstart = i * 4;
    mpoly[i].totloop = 4;
  }

  BKE_mesh_calc_edges(result, false, false);

  return result;
}

static void modifyGeometry(ModifierData *md,
                           const ModifierEvalContext *ctx,
                           GeometrySet &geometry_set)
{
#ifdef WITH_ALEMBIC
  MeshSeqCacheModifierData *mcmd = reinterpret_cast<MeshSeqCacheModifierData *>(md);

  Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  CacheFile *cache_file = mcmd->cache_file;
  const float frame = DEG_get_ctime(ctx->depsgraph);
  const float time = BKE_cachefile_time_offset(cache_file, frame, FPS);
  const char *err_str = nullptr;

  if (!mcmd->reader || !STREQ(mcmd->reader_object_path, mcmd->object_path)) {
    STRNCPY(mcmd->reader_object_path, mcmd->object_path);
    BKE_cachefile_reader_open(cache_file, &mcmd->reader, ctx->object, mcmd->object_path);
    if (!mcmd->reader) {
      BKE_modifier_set_error(
          ctx->object, md, "Could not create Alembic reader for file %s", cache_file->filepath);
      return;
    }
  }

  if (geometry_set.has_mesh()) {
    Mesh *mesh = geometry_set.get_mesh_for_write();
    if (need_orco_mesh(mcmd, ctx, mesh, time, &err_str)) {
      return;
    }
  }

  /* Do not process data if using a render procedural, return a box instead for displaying in the
   * viewport. */
  if (BKE_cache_file_uses_render_procedural(cache_file, scene, DEG_get_mode(ctx->depsgraph))) {
    Mesh *org_mesh = nullptr;
    if (geometry_set.has_mesh()) {
      org_mesh = geometry_set.get_mesh_for_write();
    }

    Mesh *bbox = generate_bounding_box_mesh(ctx->object, org_mesh);
    geometry_set = GeometrySet::create_with_mesh(bbox, GeometryOwnershipType::Editable);
    return;
  }

  /* Time (in frames or seconds) between two velocity samples. Automatically computed to
   * scale the velocity vectors at render time for generating proper motion blur data. */
  float velocity_scale = mcmd->velocity_scale;
  if (mcmd->cache_file->velocity_unit == CACHEFILE_VELOCITY_UNIT_FRAME) {
    velocity_scale *= FPS;
  }

  switch (cache_file->type) {
    default: {
      break;
    }
#  ifdef WITH_ALEMBIC
    case CACHEFILE_TYPE_ALEMBIC: {
      ABCReadParams params;
      params.time = time;
      params.read_flags = mcmd->read_flag;
      params.velocity_name = mcmd->cache_file->velocity_name;
      params.velocity_scale = velocity_scale;
      params.mappings = &mcmd->cache_file->attribute_mappings;
      ABC_read_geometry(mcmd->reader, ctx->object, geometry_set, &params, &err_str);
      break;
    }
#  endif
#  ifdef WITH_USD
    case CACHEFILE_TYPE_USD: {
      USD_read_geometry(mcmd->reader, ctx->object, geometry_set, time, &err_str, mcmd->read_flag);
    }
#  endif
  }

  if (err_str) {
    BKE_modifier_set_error(ctx->object, md, "%s", err_str);
  }

#else
  UNUSED_VARS(ctx, md, geometry_set);
  return;
#endif
}

static Mesh *modifyMesh(ModifierData *md, const ModifierEvalContext *ctx, Mesh *mesh)
{
#if defined(WITH_USD) || defined(WITH_ALEMBIC)
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  /* Only used to check whether we are operating on org data or not... */
  Mesh *me = (ctx->object->type == OB_MESH) ? static_cast<Mesh *>(ctx->object->data) : nullptr;
  Mesh *org_mesh = mesh;

  Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  CacheFile *cache_file = mcmd->cache_file;
  const float frame = DEG_get_ctime(ctx->depsgraph);
  const float time = BKE_cachefile_time_offset(cache_file, frame, FPS);
  const char *err_str = nullptr;

  if (!mcmd->reader || !STREQ(mcmd->reader_object_path, mcmd->object_path)) {
    STRNCPY(mcmd->reader_object_path, mcmd->object_path);
    BKE_cachefile_reader_open(cache_file, &mcmd->reader, ctx->object, mcmd->object_path);
    if (!mcmd->reader) {
      BKE_modifier_set_error(
          ctx->object, md, "Could not create reader for file %s", cache_file->filepath);
      return mesh;
    }
  }

  /* Do not process data if using a render procedural, return a box instead for displaying in the
   * viewport. */
  if (BKE_cache_file_uses_render_procedural(cache_file, scene, DEG_get_mode(ctx->depsgraph))) {
    return generate_bounding_box_mesh(ctx->object, org_mesh);
  }

  if (need_orco_mesh(mcmd, ctx, mesh, time, &err_str)) {
    return mesh;
  }

  if (me != nullptr) {
    MVert *mvert = mesh->mvert;
    MEdge *medge = mesh->medge;
    MPoly *mpoly = mesh->mpoly;

    /* TODO(sybren+bastien): possibly check relevant custom data layers (UV/color depending on
     * flags) and duplicate those too. */
    if ((me->mvert == mvert) || (me->medge == medge) || (me->mpoly == mpoly)) {
      /* We need to duplicate data here, otherwise we'll modify org mesh, see T51701. */
      mesh = (Mesh *)BKE_id_copy_ex(nullptr,
                                    &mesh->id,
                                    nullptr,
                                    LIB_ID_CREATE_NO_MAIN | LIB_ID_CREATE_NO_USER_REFCOUNT |
                                        LIB_ID_CREATE_NO_DEG_TAG | LIB_ID_COPY_NO_PREVIEW);
    }
  }

  Mesh *result = nullptr;

  GeometrySet geometry_set;

  if (ctx->object->type == OB_CURVE) {
    std::unique_ptr<CurveEval> curve_eval = curve_eval_from_dna_curve(
        *static_cast<Curve *>(ctx->object->data));
    geometry_set = GeometrySet::create_with_curve(curve_eval.release(),
                                                  GeometryOwnershipType::Editable);
  }
  else {
    geometry_set = GeometrySet::create_with_mesh(mesh, GeometryOwnershipType::Editable);
  }

  modifyGeometry(md, ctx, geometry_set);

  if (ctx->object->type == OB_CURVE) {
    CurveEval *curve_eval = geometry_set.get_component_for_write<CurveComponent>().release();

    if (curve_eval) {
      result = blender::bke::curve_to_wire_mesh(*curve_eval);
      delete curve_eval;
    }
  }
  else {
    result = geometry_set.get_component_for_write<MeshComponent>().release();
  }

  if (!ELEM(result, nullptr, mesh) && (mesh != org_mesh)) {
    BKE_id_free(nullptr, mesh);
    mesh = org_mesh;
  }

  return result ? result : mesh;
#else
  UNUSED_VARS(ctx, md, generate_bounding_box_mesh);
  return mesh;
#endif
}

static void modifyGeometrySet(ModifierData *md,
                              const ModifierEvalContext *ctx,
                              GeometrySet *geometry_set)
{
  modifyGeometry(md, ctx, *geometry_set);
}

static bool dependsOnTime(Scene *scene, ModifierData *md, const int dag_eval_mode)
{
#if defined(WITH_USD) || defined(WITH_ALEMBIC)
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;
  /* Do not evaluate animations if using the render engine procedural. */
  return (mcmd->cache_file != NULL) &&
         !BKE_cache_file_uses_render_procedural(mcmd->cache_file, scene, dag_eval_mode);
#else
  UNUSED_VARS(scene, md, dag_eval_mode);
  return false;
#endif
}

static void foreachIDLink(ModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  walk(userData, ob, (ID **)&mcmd->cache_file, IDWALK_CB_USER);
}

static void updateDepsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  MeshSeqCacheModifierData *mcmd = (MeshSeqCacheModifierData *)md;

  if (mcmd->cache_file != nullptr) {
    DEG_add_object_cache_relation(
        ctx->node, mcmd->cache_file, DEG_OB_COMP_CACHE, "Mesh Cache File");
  }
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA cache_file_ptr = RNA_pointer_get(ptr, "cache_file");
  bool has_cache_file = !RNA_pointer_is_null(&cache_file_ptr);

  uiLayoutSetPropSep(layout, true);

  uiTemplateCacheFile(layout, C, ptr, "cache_file");

  if (has_cache_file) {
    uiItemPointerR(
        layout, ptr, "object_path", &cache_file_ptr, "object_paths", nullptr, ICON_NONE);
  }

  if (RNA_enum_get(&ob_ptr, "type") == OB_MESH) {
    uiItemR(layout, ptr, "read_data", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
    uiItemR(layout, ptr, "use_vertex_interpolation", 0, nullptr, ICON_NONE);
  }

  modifier_panel_end(layout, ptr);
}

static void velocity_panel_draw(const bContext *UNUSED(C), Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA fileptr;
  if (!uiTemplateCacheFilePointer(ptr, "cache_file", &fileptr)) {
    return;
  }

  uiLayoutSetPropSep(layout, true);
  uiTemplateCacheFileVelocity(layout, &fileptr);
  uiItemR(layout, ptr, "velocity_scale", 0, nullptr, ICON_NONE);
}

static void attribute_remapping_panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA fileptr;
  if (!uiTemplateCacheFilePointer(ptr, "cache_file", &fileptr)) {
    return;
  }

  uiLayoutSetPropSep(layout, true);
  uiTemplateCacheFileAttributeRemapping(layout, C, &fileptr);
}

static void override_layers_panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA fileptr;
  if (!uiTemplateCacheFilePointer(ptr, "cache_file", &fileptr)) {
    return;
  }

  uiLayoutSetPropSep(layout, true);
  uiTemplateCacheFileLayers(layout, C, &fileptr);
}

static void time_settings_panel_draw(const bContext *UNUSED(C), Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA fileptr;
  if (!uiTemplateCacheFilePointer(ptr, "cache_file", &fileptr)) {
    return;
  }

  uiLayoutSetPropSep(layout, true);
  uiTemplateCacheFileTimeSettings(layout, &fileptr);
}

static void render_procedural_panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA fileptr;
  if (!uiTemplateCacheFilePointer(ptr, "cache_file", &fileptr)) {
    return;
  }

  uiLayoutSetPropSep(layout, true);
  uiTemplateCacheFileProcedural(layout, C, &fileptr);
}

static void panelRegister(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_MeshSequenceCache, panel_draw);
  modifier_subpanel_register(region_type,
                             "time_settings",
                             "Time Settings",
                             nullptr,
                             time_settings_panel_draw,
                             panel_type);
  modifier_subpanel_register(region_type,
                             "render_procedural",
                             "Render Procedural",
                             nullptr,
                             render_procedural_panel_draw,
                             panel_type);
  modifier_subpanel_register(region_type,
                             "override_layers",
                             "Override Layers",
                             nullptr,
                             override_layers_panel_draw,
                             panel_type);
  modifier_subpanel_register(region_type,
                             "velocity_parameters",
                             "Velocity Parameters",
                             nullptr,
                             velocity_panel_draw,
                             panel_type);
  modifier_subpanel_register(region_type,
                             "attribute_remapping",
                             "Attribute Remapping",
                             nullptr,
                             attribute_remapping_panel_draw,
                             panel_type);
}

static void blendRead(BlendDataReader *UNUSED(reader), ModifierData *md)
{
  MeshSeqCacheModifierData *msmcd = (MeshSeqCacheModifierData *)md;
  msmcd->reader = nullptr;
  msmcd->reader_object_path[0] = '\0';
}

ModifierTypeInfo modifierType_MeshSequenceCache = {
    /* name */ "MeshSequenceCache",
    /* structName */ "MeshSeqCacheModifierData",
    /* structSize */ sizeof(MeshSeqCacheModifierData),
    /* srna */ &RNA_MeshSequenceCacheModifier,
    /* type */ eModifierTypeType_Constructive,
    /* flags */
    static_cast<ModifierTypeFlag>(eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_AcceptsCVs),
    /* icon */ ICON_MOD_MESHDEFORM, /* TODO: Use correct icon. */

    /* copyData */ copyData,

    /* deformVerts */ nullptr,
    /* deformMatrices */ nullptr,
    /* deformVertsEM */ nullptr,
    /* deformMatricesEM */ nullptr,
    /* modifyMesh */ modifyMesh,
    /* modifyHair */ nullptr,
    /* modifyGeometrySet */ modifyGeometrySet,

    /* initData */ initData,
    /* requiredDataMask */ nullptr,
    /* freeData */ freeData,
    /* isDisabled */ isDisabled,
    /* updateDepsgraph */ updateDepsgraph,
    /* dependsOnTime */ dependsOnTime,
    /* dependsOnNormals */ nullptr,
    /* foreachIDLink */ foreachIDLink,
    /* foreachTexLink */ nullptr,
    /* freeRuntimeData */ nullptr,
    /* panelRegister */ panelRegister,
    /* blendWrite */ nullptr,
    /* blendRead */ blendRead,
};

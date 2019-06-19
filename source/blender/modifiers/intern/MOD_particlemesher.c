/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
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
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 by the Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_particlemesher.c
 *  \ingroup modifiers
 */

/* Particle mesher modifier: generates a mesh from a particle system */

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_listbase.h"

#include "BKE_modifier.h"
#include "BKE_particle.h"
#include "BKE_library.h"
#include "BKE_library_query.h"

#include "DNA_scene_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "MOD_util.h"
#include "MOD_openvdb_util.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "openvdb_capi.h"

static void initData(ModifierData *md)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;

	pmmd->psys = NULL;
	pmmd->part_list = NULL;
	pmmd->level_set = NULL;
	pmmd->mesher_mask = NULL;
	pmmd->mesher_mask_ob = NULL;
	pmmd->voxel_size = 0.2f;
	pmmd->min_part_radius = 1.5f;
	pmmd->half_width = 0.75f;
	pmmd->generate_trails = false;
	pmmd->part_scale_factor = 1.0f;
	pmmd->part_vel_factor = 1.0f;
	pmmd->trail_size = 1.0f;
	pmmd->adaptivity = 0.0f;
	pmmd->isovalue = 0.0f;
	pmmd->generate_mask = false;
	pmmd->mask_width = 0.0f;
	pmmd->mask_offset = 0.0f;
	pmmd->invert_mask = false;
	pmmd->ext_band = 3.0f;
	pmmd->int_band = 3.0f;
}

static void copyData(ModifierData *md, ModifierData *target, int flag)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;
	ParticleMesherModifierData *tpmmd = (ParticleMesherModifierData *) target;

	modifier_copyData_generic(md, target, flag);
	/* only for cow copy, because cow does shallow copy only*/
	if (flag & LIB_ID_CREATE_NO_MAIN) {
		BLI_duplicatelist(&tpmmd->filters, &pmmd->filters);
	}
}

static bool isDisabled(const struct Scene *UNUSED(scene), ModifierData *md, bool UNUSED(useRenderParams))
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;

	return !pmmd->psys;
}

static void foreachObjectLink(ModifierData *md, Object *ob, ObjectWalkFunc walk,
                              void *userData)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;

	if (pmmd->mesher_mask_ob) {
		walk(userData, ob, &pmmd->mesher_mask_ob, IDWALK_CB_NOP);
	}
}

static void updateDepsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;

	if (pmmd->mesher_mask_ob != NULL) {
		DEG_add_object_relation(ctx->node, pmmd->mesher_mask_ob, DEG_OB_COMP_TRANSFORM,
		                        "Particle Mesher Modifier");
	}
}

#ifdef WITH_MOD_PARTMESHER
static Mesh *applyModifier(ModifierData *md, const ModifierEvalContext *ctx, Mesh *mesh)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;
	Mesh *result, *mask_me = NULL;
    Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);

	if (pmmd->mesher_mask_ob) {
        Object *ob_eval = DEG_get_evaluated_object(ctx->depsgraph, pmmd->mesher_mask_ob);
        mask_me = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_eval, false);
	}

	result = NewParticleMesh(mesh, ctx->object, mask_me, pmmd->mesher_mask_ob, pmmd, scene, ctx->depsgraph);

	if (result) {
		return result;
	}
	else {
		modifier_setError(md, "Cannot execute modifier");
	}

	return mesh;
}
#else /* WITH_MOD_PARTMESHER */
static Mesh *applyModifier(ModifierData *md, const ModifierEvalContext *ctx, Mesh *mesh)
{
	return mesh;
	UNUSED_VARS(md, ctx);
}
#endif /* WITH_MOD_PARTMESHER */

static void requiredDataMask(Object *UNUSED(ob),
                             ModifierData *UNUSED(md),
                             CustomData_MeshMasks *r_cddata_masks)
{
    r_cddata_masks->fmask |= CD_MASK_MTFACE;
    r_cddata_masks->emask |= CD_MASK_MEDGE;
    r_cddata_masks->vmask |= CD_MASK_MDEFORMVERT;
}

static void freeData(ModifierData *md)
{
	ParticleMesherModifierData *pmmd = (ParticleMesherModifierData *) md;
	LevelSetFilter *filter = pmmd->filters.first;

	for (; filter; filter = filter->next) {
		MEM_freeN(filter);
	}

	if (pmmd->part_list) {
		OpenVDB_part_list_free(pmmd->part_list);
	}

	if (pmmd->level_set) {
		OpenVDBPrimitive_free(pmmd->level_set);
	}

	if (pmmd->mesher_mask) {
		OpenVDBPrimitive_free(pmmd->mesher_mask);
	}
}

ModifierTypeInfo modifierType_ParticleMesher = {
    /* name */              "Particle Mesher",
    /* structName */        "ParticleMesherModifierData",
    /* structSize */        sizeof(ParticleMesherModifierData),
    /* type */              eModifierTypeType_Constructive,
    /* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_SupportsEditmode |
							eModifierTypeFlag_EnableInEditmode,

    /* copyData */          copyData,
    /* deformVerts */       NULL,
    /* deformMatrices */    NULL,
    /* deformVertsEM */     NULL,
    /* deformMatricesEM */  NULL,
    /* applyModifier */     applyModifier,
   
    /* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          freeData,
    /* isDisabled */        isDisabled,
    /* updateDepsgraph */   updateDepsgraph,
    /* dependsOnTime */     NULL,
    /* dependsOnNormals */  NULL,
    /* foreachObjectLink */ foreachObjectLink,
    /* foreachIDLink */     NULL,
    /* foreachTexLink */    NULL,
    /* freeRuntimeData */   NULL,
};

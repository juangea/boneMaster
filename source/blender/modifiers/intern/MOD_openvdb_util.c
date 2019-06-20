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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include <limits.h>

#include "MEM_guardedalloc.h"

#include "BLI_math.h"

#include "BKE_lattice.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_modifier.h"
#include "BKE_scene.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"

#include "DNA_object_types.h"
#include "DNA_modifier_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_particle_types.h"
#include "DNA_scene_types.h"

#include "MOD_openvdb_util.h"
#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "openvdb_capi.h"

static void populate_particle_list(ParticleMesherModifierData *pmmd, Scene *scene, Object *ob, Depsgraph *depsgraph)
{
	ParticleSystem *psys = pmmd->psys;

	if (psys) {
		ParticleSimulationData sim = {0};
		ParticleKey state;
		int p;

		if (psys->part->size > 0.0f) {
//			part_list.has_radius(true);
		}

		/* TODO(kevin): this isn't right at all */
		if (psys->part->normfac > 0.0f) {
//			part_list.has_velocity(true);
		}

		sim.depsgraph = depsgraph;
		sim.scene = scene;
		sim.ob = ob;
		sim.psys = psys;

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
			sub_v3_v3v3(vel, state.co, psys->particles[p].prev_state.co);

			mul_v3_fl(vel, psys->part->normfac);

			OpenVDB_add_particle(pmmd->part_list,
			                     pos,
			                     psys->particles[p].size,
			                     vel);
		}

		if (psys->lattice_deform_data) {
			end_latt_deform(psys->lattice_deform_data);
			psys->lattice_deform_data = NULL;
		}
	}
}

static void compute_vdb_prim_matrix(struct Object *ob,
                                    struct OpenVDBPrimitive *level_set,
                                    const float voxel_size)
{
	float level_set_mat[4][4];
	float cell_size[3];

	copy_v3_fl(cell_size, voxel_size);
	size_to_mat4(level_set_mat, cell_size);

	mul_m4_m4m4(level_set_mat, ob->obmat, level_set_mat);

	OpenVDBPrimitive_set_transform(level_set, level_set_mat);
}

static struct OpenVDBGeom *VDBGeom_from_Mesh(Object *ob, Mesh *me)
{
	struct OpenVDBGeom *geom = NULL;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	const int num_verts = me->totvert;
	const int num_polys = me->totpoly;
	int i;

	geom = OpenVDBGeom_create(num_verts, num_polys);

	mverts = me->mvert;
	for (i = 0; i < num_verts; i++) {
		float coord[3];
		mul_v3_m4v3(coord, ob->obmat,  mverts[i].co);
		OpenVDBGeom_add_point(geom, coord);
	}

	mpolys = me->mpoly;
	mloops = me->mloop;
	for (i = 0; i < num_polys; i++) {
		MPoly *poly = &mpolys[i];
		MLoop *loop = mloops + poly->loopstart;
		//TODO deal with ngons properly !!!! (else stack smashing may happen here)
		//triangulate temporarily for example, use looptri or so
		int quad[4] = { INT_MAX };
		int j;

		for (j = 0; j < poly->totloop; j++) {
			quad[j] = loop->v;
		}

		OpenVDBGeom_add_quad(geom, quad);
	}

	return geom;
}

static Mesh *Mesh_from_VDBGeom(struct OpenVDBGeom *geom, Object *ob)
{
	Mesh *me = NULL;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	int num_verts = OpenVDBGeom_get_num_points(geom);
	int num_quads = OpenVDBGeom_get_num_quads(geom);
	int num_tris = OpenVDBGeom_get_num_tris(geom);
	int num_polys = num_tris + num_quads;
	int num_loops = num_tris * 3 + num_quads * 4;
	int i, loop_index = 0, poly_index = 0;

	me = BKE_mesh_new_nomain(num_verts, 0, 0, num_loops, num_polys);
	mverts = me->mvert;
	mpolys = me->mpoly;
	mloops = me->mloop;

	for (i = 0; i < num_verts; i++) {
		OpenVDBGeom_get_point(geom, i, mverts[i].co);
	}

	for (i = 0; i < num_quads; i++, poly_index++) {
		MPoly *poly = &mpolys[poly_index];
		int start_loop_index = loop_index;
		int quad[4];
		int j;

		OpenVDBGeom_get_quad(geom, i, quad);
		for (j = 3; j >= 0; j--) {
			MLoop *loop = &mloops[loop_index++];
			loop->v = quad[j];
		}

		poly->loopstart = start_loop_index;
		poly->totloop = 4;
		poly->flag |= ME_SMOOTH;
	}

	for (i = 0; i < num_tris; i++, poly_index++) {
		MPoly *poly = &mpolys[poly_index];
		int start_loop_index = loop_index;
		int triangle[3];
		int j;

		OpenVDBGeom_get_triangle(geom, i, triangle);
		for (j = 2; j >= 0; j--) {
			MLoop *loop = &mloops[loop_index++];
			loop->v = triangle[j];
		}

		poly->loopstart = start_loop_index;
		poly->totloop = 3;
		poly->flag |= ME_SMOOTH;
	}

	BKE_mesh_calc_edges(me, true, true);
	BKE_mesh_calc_normals(me);
	BKE_mesh_tessface_ensure(me);
	BKE_mesh_runtime_looptri_ensure(me);
	me->runtime.cd_dirty_vert |= CD_MASK_NORMAL;

	return me;
}

Mesh *NewParticleMesh(Mesh *me, struct Object *ob,
					  Mesh *cutter_me, struct Object *cutter_ob,
				      ParticleMesherModifierData *pmmd, Scene *scene, Depsgraph *depsgraph)
{
	struct OpenVDBPrimitive *filter_mask = NULL;
	struct OpenVDBGeom *mask_geom = NULL, *output_geom = NULL;
	LevelSetFilter *filter = NULL;
	Mesh *output_me = NULL;

	if (me == NULL) {
		return NULL;
	}

	pmmd->level_set = OpenVDBPrimitive_create_level_set(pmmd->voxel_size, pmmd->half_width);
	compute_vdb_prim_matrix(ob, pmmd->level_set, pmmd->voxel_size);

	/* Generate a particle list */
	pmmd->part_list = OpenVDB_create_part_list(pmmd->psys->totpart,
	                                           pmmd->part_scale_factor,
	                                           pmmd->part_vel_factor);

	populate_particle_list(pmmd, scene, ob, depsgraph);

	if (pmmd->generate_mask) {
		filter_mask = OpenVDBPrimitive_create_level_set(pmmd->voxel_size, pmmd->half_width);
	}

	/* Create a level set from the particle list */

	OpenVDB_from_particles(pmmd->level_set, filter_mask, pmmd->part_list,
	                       pmmd->generate_mask, pmmd->mask_width, pmmd->min_part_radius,
	                       pmmd->generate_trails, pmmd->trail_size);

	/* Apply some filters to the level set */
	for (filter = pmmd->filters.first; filter; filter = filter->next) {
		if (!(filter->flag & LVLSETFILTER_MUTE)) {
			OpenVDB_filter_level_set(pmmd->level_set, filter_mask,
			                         filter->accuracy, filter->type,
			                         filter->iterations, filter->width,
			                         filter->offset);
		}
	}

	/* Convert the level set to a mesh and output it */
	if (cutter_me != NULL) {
		mask_geom = VDBGeom_from_Mesh(cutter_ob, cutter_me);

		pmmd->mesher_mask = OpenVDBPrimitive_create(VDB_GRID_FLOAT);
		compute_vdb_prim_matrix(cutter_ob, pmmd->mesher_mask, pmmd->voxel_size);
		pmmd->mesher_mask = OpenVDB_from_polygons(mask_geom, pmmd->mesher_mask, pmmd->voxel_size, pmmd->int_band, pmmd->ext_band);
	}
	/* XXX - hack, perhaps OpenVDBPrimitive should be part of Object struct? */
	else {
		if (pmmd->mesher_mask) {
			OpenVDBPrimitive_free(pmmd->mesher_mask);
		}
	}

	output_geom = OpenVDB_to_polygons(pmmd->level_set, pmmd->mesher_mask, pmmd->isovalue, pmmd->adaptivity, pmmd->mask_offset, pmmd->invert_mask);
	output_me = Mesh_from_VDBGeom(output_geom, ob);
	OpenVDBGeom_free(output_geom);

	output_me->cd_flag |= me->cd_flag;

	if (cutter_me != NULL) {
		OpenVDBGeom_free(mask_geom);
	}

	if (pmmd->generate_mask) {
		OpenVDBPrimitive_free(filter_mask);
	}

	return output_me;
}

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

#include <vector>

#include "openvdb_capi.h"
#include "openvdb_intern.h"
#include "openvdb_primitive.h"
#include "particle_tools.h"

using namespace openvdb;

int OpenVDB_getVersionHex()
{
	return OPENVDB_LIBRARY_VERSION;
}

/* ****************************** Particle List ****************************** */

ParticleList *OpenVDB_create_part_list(size_t totpart, float rad_scale, float vel_scale)
{
	return new ParticleList(totpart, rad_scale, vel_scale);
}

void OpenVDB_part_list_free(ParticleList *part_list)
{
	delete part_list;
	part_list = NULL;
}

void OpenVDB_add_particle(ParticleList *part_list,
                          float pos[3], float rad, float vel[3])
{
	Vec3R nvel(vel);
	float nrad = rad * part_list->radius_scale();
	nvel *= part_list->velocity_scale();

	part_list->add(pos, nrad, nvel);
}

void OpenVDB_from_particles(OpenVDBPrimitive *level_set, OpenVDBPrimitive *mask_grid,
                            ParticleList *Pa, bool mask, float mask_width,
                            float min_radius, bool trail, float trail_size)
{
	internal::OpenVDB_from_particles(level_set, mask_grid, *Pa, mask, mask_width,
	                                 min_radius, trail, trail_size);
}

void OpenVDB_set_part_list_flags(struct ParticleList *part_list, const bool has_radius, const bool has_velocity)
{
	part_list->set_flags(has_radius, has_velocity);
}

/* **************************** OpenVDB Primitive **************************** */

OpenVDBPrimitive *OpenVDBPrimitive_create(int grid_type)
{
	OpenVDBPrimitive *vdb_prim = new OpenVDBPrimitive();
	GridBase::Ptr grid;

	switch (grid_type) {
		case VDB_GRID_FLOAT:
			grid = FloatGrid::create(0.0f);
			break;
		case VDB_GRID_DOUBLE:
			grid = DoubleGrid::create(0.0);
			break;
		case VDB_GRID_INT32:
			grid = Int32Grid::create(0);
			break;
		case VDB_GRID_INT64:
			grid = Int64Grid::create(0);
			break;
		case VDB_GRID_BOOL:
			grid = BoolGrid::create(false);
			break;
		case VDB_GRID_VEC3F:
			grid = Vec3fGrid::create(Vec3f(0.0f));
			break;
		case VDB_GRID_VEC3D:
			grid = Vec3dGrid::create(Vec3d(0.0));
			break;
		case VDB_GRID_VEC3I:
			grid = Vec3IGrid::create(Vec3I(0));
			break;
	}

	vdb_prim->setGrid(grid);

	return vdb_prim;
}

OpenVDBPrimitive *OpenVDBPrimitive_create_level_set(float voxel_size, float half_width)
{
	using namespace openvdb;

	OpenVDBPrimitive *vdb_prim = new OpenVDBPrimitive();

	FloatGrid::Ptr grid = createLevelSet<FloatGrid>(voxel_size, half_width);
	grid->setTransform(math::Transform::createLinearTransform(voxel_size));
	vdb_prim->setGrid(grid);

	return vdb_prim;
}

void OpenVDBPrimitive_free(OpenVDBPrimitive *vdb_prim)
{
	delete vdb_prim;
	vdb_prim = NULL;
}

void OpenVDBPrimitive_set_transform(OpenVDBPrimitive *vdb_prim, float mat[4][4])
{
	vdb_prim->setTransform(mat);
}

void OpenVDB_filter_level_set(OpenVDBPrimitive *level_set, OpenVDBPrimitive *filter_mask,
                              int accuracy, int type, int iterations, int width, float offset)
{
	internal::OpenVDB_filter_level_set(level_set, filter_mask, accuracy, type, iterations, width, offset);
}

OpenVDBPrimitive *OpenVDB_from_polygons(OpenVDBGeom *geom, OpenVDBPrimitive *level_set, float voxel_size, float int_band, float ext_band)
{
	return internal::OpenVDB_from_polygons(geom, level_set, voxel_size, int_band, ext_band);
}

OpenVDBGeom *OpenVDB_to_polygons(OpenVDBPrimitive *level_set, OpenVDBPrimitive *mask_grid, float isovalue, float adaptivity, float mask_offset, bool invert_mask)
{
	return internal::OpenVDB_to_polygons(level_set, mask_grid, isovalue, adaptivity, mask_offset, invert_mask);
}

/* ******************************* OpenVDBGeom ******************************* */

struct OpenVDBGeom *OpenVDBGeom_create(size_t num_points, size_t num_polys)
{
	OpenVDBGeom *geom = new OpenVDBGeom;
	geom->m_points.reserve(num_points);
	geom->m_polys.reserve(num_polys);

	return geom;
}

void OpenVDBGeom_free(struct OpenVDBGeom *geom)
{
	delete geom;
	geom = NULL;
}

void OpenVDBGeom_add_point(OpenVDBGeom *geom, const float point[3])
{
	geom->m_points.push_back(point);
}

void OpenVDBGeom_add_quad(OpenVDBGeom *geom, const int quad[4])
{
	geom->m_polys.push_back(quad);
}

void OpenVDBGeom_addTriangle(OpenVDBGeom *geom, const int tri[3])
{
	geom->m_tris.push_back(tri);
}

void OpenVDBGeom_get_point(OpenVDBGeom *geom, const size_t idx, float r_point[3])
{
	Vec3s point = geom->m_points[idx];
	r_point[0] = point.x();
	r_point[1] = point.y();
	r_point[2] = point.z();
}

void OpenVDBGeom_get_quad(OpenVDBGeom *geom, const size_t idx, int r_quad[4])
{
	Vec4I quad = geom->m_polys[idx];
	r_quad[0] = quad.x();
	r_quad[1] = quad.y();
	r_quad[2] = quad.z();
	r_quad[3] = quad.w();
}

void OpenVDBGeom_get_triangle(OpenVDBGeom *geom, const size_t idx, int r_triangle[3])
{
	Vec3I triangle = geom->m_tris[idx];
	r_triangle[0] = triangle.x();
	r_triangle[1] = triangle.y();
	r_triangle[2] = triangle.z();
}

size_t OpenVDBGeom_get_num_points(OpenVDBGeom *geom)
{
	return geom->m_points.size();
}

size_t OpenVDBGeom_get_num_quads(OpenVDBGeom *geom)
{
	return geom->m_polys.size();
}

size_t OpenVDBGeom_get_num_tris(struct OpenVDBGeom *geom)
{
	return geom->m_tris.size();
}

void OpenVDB_draw_primitive(struct OpenVDBPrimitive *vdb_prim,
                            const bool draw_root,
                            const bool draw_level_1,
                            const bool draw_level_2,
                            const bool draw_leaves)
{
	internal::OpenVDBPrimitive_draw_tree(vdb_prim, draw_root, draw_level_1, draw_level_2, draw_leaves);
}

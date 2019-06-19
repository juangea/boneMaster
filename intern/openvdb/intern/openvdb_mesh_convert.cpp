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
 * The Original Code is Copyright (C) 2014 Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

#include "openvdb_capi.h"
#include "openvdb_intern.h"
#include "openvdb_primitive.h"

using namespace openvdb;

namespace internal {

/* TODO(kevin): - figure out how/when to use tools::MeshToVoxelEdgeData, would
 *				probably need vertex normals.
 *				- handle case where we'd want a fog volume instead of a plain
 *				level set.
 */
OpenVDBPrimitive *OpenVDB_from_polygons(OpenVDBGeom *geom,
                                        OpenVDBPrimitive *input_prim,
                                        float voxel_size, float int_band, float ext_band)
{
	math::Transform::Ptr transform;

	if (input_prim) {
		transform = input_prim->getGridPtr()->transform().copy();
	}
	else {
		transform = math::Transform::createLinearTransform(voxel_size);
	}

    //tools::MeshToVolume<FloatGrid> voxelizer(transform);
    //voxelizer.convertToLevelSet(geom->m_points, geom->m_polys, int_band, ext_band);

	OpenVDBPrimitive *vdb_prim = new OpenVDBPrimitive();
    vdb_prim->setGrid(tools::meshToSignedDistanceField<FloatGrid>(
                *transform, geom->m_points, geom->m_tris, geom->m_polys, int_band, ext_band));

	/* XXX - should be done in a better way */
	if (input_prim) {
		delete input_prim;
	}

	return vdb_prim;
}

OpenVDBGeom *OpenVDB_to_polygons(OpenVDBPrimitive *level_set,
                                 OpenVDBPrimitive *mask_grid,
                                 float isovalue, float adaptivity,
                                 float mask_offset, bool invert_mask)
{
	FloatGrid::Ptr ls_grid = gridPtrCast<FloatGrid>(level_set->getGridPtr());

	tools::VolumeToMesh mesher(isovalue, adaptivity);

	if (mask_grid) {
		FloatGrid::ConstPtr grid = gridConstPtrCast<FloatGrid>(mask_grid->getConstGridPtr());
		mesher.setSurfaceMask(tools::sdfInteriorMask(*grid, mask_offset),
		                      invert_mask);
	}

	mesher(*ls_grid);

	const tools::PointList &points = mesher.pointList();
	tools::PolygonPoolList &polygonPoolList = mesher.polygonPoolList();

	OpenVDBGeom *geom = new OpenVDBGeom;
	geom->m_points.reserve(mesher.pointListSize());

	for (size_t i = 0; i < mesher.pointListSize(); ++i) {
		geom->m_points.push_back(points[i]);
	}

	for (size_t i = 0; i < mesher.polygonPoolListSize(); ++i) {
		tools::PolygonPool &polygons = polygonPoolList[i];

		for (size_t j = 0; j < polygons.numQuads(); ++j) {
			geom->m_polys.push_back(polygons.quad(j));
		}

		for (size_t j = 0; j < polygons.numTriangles(); ++j) {
			geom->m_tris.push_back(polygons.triangle(j));
		}
	}

	return geom;
}

}

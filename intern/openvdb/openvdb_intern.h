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

#ifndef __OPENVDB_INTERN_H__
#define __OPENVDB_INTERN_H__

#include <openvdb/openvdb.h>

#include "openvdb_capi.h"

class ParticleList;

namespace internal {

void OpenVDB_filter_level_set(OpenVDBPrimitive *level_set,
                              OpenVDBPrimitive *filter_mask,
                              int accuracy, int type, int iterations,
                              int width, float offset);

void OpenVDB_from_particles(OpenVDBPrimitive *level_set,
                            OpenVDBPrimitive *&mask_grid,
                            ParticleList Pa, bool mask, float mask_width,
                            float min_radius, bool trail, float trail_size);

OpenVDBPrimitive *OpenVDB_from_polygons(OpenVDBGeom *geom,
                                        OpenVDBPrimitive *input_prim,
                                        float voxel_size, float int_band, float ext_band);

OpenVDBGeom *OpenVDB_to_polygons(OpenVDBPrimitive *level_set,
                                 OpenVDBPrimitive *mask_grid,
                                 float isovalue, float adaptivity,
                                 float mask_offset, bool invert_mask);

/*void OpenVDBPrimitive_draw_tree(struct OpenVDBPrimitive *vdb_prim,
                                const bool draw_root,
	                            const bool draw_level_1,
	                            const bool draw_level_2,
                                const bool draw_leaves);*/

}

#endif /* __OPENVDB_INTERN_H__ */

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

#include <openvdb/tools/LevelSetFilter.h>

#include "openvdb_capi.h"
#include "openvdb_intern.h"
#include "openvdb_primitive.h"

using namespace openvdb;

namespace internal {

void OpenVDB_filter_level_set(OpenVDBPrimitive *level_set,
                              OpenVDBPrimitive *filter_mask,
                              int accuracy, int type, int iterations,
                              int width, float offset)
{
	FloatGrid::Ptr ls_grid = gridPtrCast<FloatGrid>(level_set->getGridPtr());

	typedef FloatGrid Mask;
	typedef tools::LevelSetFilter<FloatGrid, Mask> Filter;
	Mask *mask = NULL;

	if (filter_mask) {
		FloatGrid::Ptr mask_grid = gridPtrCast<FloatGrid>(filter_mask->getGridPtr());
		mask = mask_grid.get();
	}

	Filter filter(*ls_grid);

	filter.setTemporalScheme(math::TVD_RK1);

	switch (accuracy) {
		case LEVEL_FILTER_ACC_FISRT:   filter.setSpatialScheme(math::FIRST_BIAS);   break;
		case LEVEL_FILTER_ACC_SECOND:  filter.setSpatialScheme(math::SECOND_BIAS);  break;
		case LEVEL_FILTER_ACC_THIRD:   filter.setSpatialScheme(math::THIRD_BIAS);   break;
		case LEVEL_FILTER_ACC_WENO5:   filter.setSpatialScheme(math::WENO5_BIAS);   break;
		case LEVEL_FILTER_ACC_HJWENO5: filter.setSpatialScheme(math::HJWENO5_BIAS); break;
	}

	switch (type) {
		case LEVEL_FILTER_MEDIAN:
			for (int i = 0; i < iterations; ++i) {
				filter.median(width, mask);
			}
			break;
		case LEVEL_FILTER_MEAN:
			for (int i = 0; i < iterations; ++i) {
				filter.mean(width, mask);
			}
			break;
		case LEVEL_FILTER_GAUSSIAN:
			for (int i = 0; i < iterations; ++i) {
				filter.gaussian(width, mask);
			}
			break;
		case LEVEL_FILTER_MEAN_CURV:
			for (int i = 0; i < iterations; ++i) {
				filter.meanCurvature(mask);
			}
			break;
		case LEVEL_FILTER_LAPLACIAN:
			for (int i = 0; i < iterations; ++i) {
				filter.laplacian(mask);
			}
			break;
		case LEVEL_FILTER_OFFSET:
			for (int i = 0; i < iterations; ++i) {
				filter.offset(offset, mask);
			}
			break;
	}
}

}

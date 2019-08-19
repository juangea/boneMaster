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

#include <GL/glew.h>
#include <openvdb/openvdb.h>

#include "openvdb_intern.h"
#include "openvdb_primitive.h"

namespace internal {

void OpenVDBPrimitive_draw_tree(OpenVDBPrimitive *vdb_prim, const bool draw_root, const bool draw_level_1, const bool draw_level_2, const bool draw_leaves)
{
	using namespace openvdb;
	using namespace openvdb::math;

	FloatGrid::Ptr grid = gridPtrCast<FloatGrid>(vdb_prim->getGridPtr());

	math::Vec3d wmin, wmax;
	math::Vec3s color(0.0f);

	math::Vec3s node_color[4] = {
	    math::Vec3s(0.0060f, 0.2790f, 0.6250f), // leaf nodes
	    math::Vec3s(0.8710f, 0.3940f, 0.0191f), // intermediate internal node levels
	    math::Vec3s(0.0432f, 0.3300f, 0.0411f), // first internal node level
	    math::Vec3s(0.0450f, 0.0450f, 0.0450f)  // root
	};

	math::CoordBBox bbox;

	glBegin(GL_LINES);

	for (FloatTree::NodeCIter node_iter = grid->tree().cbeginNode(); node_iter; ++node_iter) {
		node_iter.getBoundingBox(bbox);

		const Vec3d min(bbox.min().x() - 0.5, bbox.min().y() - 0.5, bbox.min().z() - 0.5);
		const Vec3d max(bbox.max().x() + 0.5, bbox.max().y() + 0.5, bbox.max().z() + 0.5);

		wmin = grid->indexToWorld(min);
		wmax = grid->indexToWorld(max);

		const int level = node_iter.getLevel();

		if (level == 0) {
			if (!draw_leaves) {
				continue;
			}
			color = node_color[0];
		}

		if (level == 1) {
			if (!draw_level_2) {
				continue;
			}
			color = node_color[1];
		}

		if (level == 2) {
			if (!draw_level_1) {
				continue;
			}
			color = node_color[2];
		}

		if (level == 3) {
			if (!draw_root) {
				continue;
			}
			color = node_color[3];
		}

		glColor3f(color[0], color[1], color[2]);

		glVertex3f(wmin.x(), wmax.y(), wmax.z());
		glVertex3f(wmax.x(), wmax.y(), wmax.z());

		glVertex3f(wmin.x(), wmax.y(), wmin.z());
		glVertex3f(wmax.x(), wmax.y(), wmin.z());

		glVertex3f(wmin.x(), wmin.y(), wmax.z());
		glVertex3f(wmax.x(), wmin.y(), wmax.z());

		glVertex3f(wmin.x(), wmin.y(), wmin.z());
		glVertex3f(wmax.x(), wmin.y(), wmin.z());

		glVertex3f(wmin.x(), wmin.y(), wmin.z());
		glVertex3f(wmin.x(), wmin.y(), wmax.z());

		glVertex3f(wmin.x(), wmax.y(), wmin.z());
		glVertex3f(wmin.x(), wmax.y(), wmax.z());

		glVertex3f(wmax.x(), wmin.y(), wmin.z());
		glVertex3f(wmax.x(), wmin.y(), wmax.z());

		glVertex3f(wmax.x(), wmax.y(), wmin.z());
		glVertex3f(wmax.x(), wmax.y(), wmax.z());

		glVertex3f(wmin.x(), wmin.y(), wmin.z());
		glVertex3f(wmin.x(), wmax.y(), wmin.z());

		glVertex3f(wmax.x(), wmin.y(), wmin.z());
		glVertex3f(wmax.x(), wmax.y(), wmin.z());

		glVertex3f(wmax.x(), wmin.y(), wmax.z());
		glVertex3f(wmax.x(), wmax.y(), wmax.z());

		glVertex3f(wmin.x(), wmin.y(), wmax.z());
		glVertex3f(wmin.x(), wmax.y(), wmax.z());
	}

	glEnd();
}

}

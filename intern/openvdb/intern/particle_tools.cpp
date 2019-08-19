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

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>

#include "openvdb_intern.h"
#include "openvdb_primitive.h"

#include "particle_tools.h"

using namespace openvdb;

ParticleList::ParticleList(size_t size, Real rad_scale, Real vel_scale)
    : m_radius_scale(rad_scale)
    , m_velocity_scale(vel_scale)
{
	m_has_radius = false;
	m_has_velocity = false;
	m_particle_list.reserve(size);
}

void ParticleList::add(const Vec3R &p, const Real &r, const Vec3R &v)
{
	Particle pa;
	pa.pos = p;
	pa.rad = r;
	pa.vel = v;
	m_particle_list.push_back(pa);
}

Real &ParticleList::radius_scale()
{
	return m_radius_scale;
}

const Real &ParticleList::radius_scale() const
{
	return m_radius_scale;
}

Real &ParticleList::velocity_scale()
{
	return m_velocity_scale;
}

bool ParticleList::has_radius() const
{
	return m_has_radius;
}

bool ParticleList::has_velocity() const
{
	return m_has_velocity;
}

void ParticleList::set_flags(const bool has_radius, const bool has_velocity)
{
	m_has_radius = has_radius;
	m_has_velocity = has_velocity;
}

CoordBBox ParticleList::getBBox(const GridBase &grid)
{
	CoordBBox bbox;
	Coord &min = bbox.min(), &max = bbox.max();

	for (int n = 0, e = this->size(); n < e; ++n) {
		const Vec3d xyz = grid.worldToIndex(this->pos(n));
		const Real r = this->radius(n) / grid.voxelSize()[0];

		for (int i = 0; i < 3; ++i) {
			min[i] = math::Min(min[i], math::Floor(xyz[i] - r));
			max[i] = math::Max(max[i], math::Ceil(xyz[i] + r));
		}
	}

	return bbox;
}

Vec3R ParticleList::pos(int n) const
{
	return m_particle_list[n].pos;
}

Vec3R ParticleList::vel(int n) const
{
	return m_velocity_scale * m_particle_list[n].vel;
}

Real ParticleList::radius(int n) const
{
	return m_radius_scale * m_particle_list[n].rad;
}

int ParticleList::size() const
{
	return m_particle_list.size();
}

void ParticleList::getPos(size_t n, Vec3R &pos) const
{
	pos = m_particle_list[n].pos;
}

void ParticleList::getPosRad(size_t n, Vec3R &pos, Real &rad) const
{
	pos = m_particle_list[n].pos;
	rad = m_radius_scale * m_particle_list[n].rad;
}

void ParticleList::getPosRadVel(size_t n, Vec3R &pos, Real &rad, Vec3R &vel) const
{
	pos = m_particle_list[n].pos;
	rad = m_radius_scale * m_particle_list[n].rad;
	vel = m_velocity_scale * m_particle_list[n].vel;
}

void ParticleList::getAtt(size_t n, Index32 &att) const
{
	att = Index32(n);
}

namespace internal {

static void convert_to_levelset(ParticleList part_list, FloatGrid::Ptr grid,
                                float min_radius, bool trail, float trail_size)
{
	/* Note: the second template argument here is the particles' attributes type,
	 * if any. As this function will later call ParticleList::getAtt(index, attribute),
	 * we pass void for two reasons: first, no attributes are defined for the
	 * particles (yet), and second, disable using attributes for generating
	 * the level set.
	 *
	 * TODO(kevin): quite useless to know that if we don't have the third argument...
	 */
	tools::ParticlesToLevelSet<FloatGrid, void> raster(*grid);
	/* a grain size of zero disables threading */
	raster.setGrainSize(1);
	raster.setRmin(min_radius);
	raster.setRmax(1e15f);

	if (trail && part_list.has_velocity()) {
		raster.rasterizeTrails(part_list, trail_size);
	}
	else {
		raster.rasterizeSpheres(part_list);
	}

	if (raster.ignoredParticles()) {
		if (raster.getMinCount() > 0) {
			std::cout << "Minimun voxel radius is too high!\n";
			std::cout << raster.getMinCount() << " particles are ignored!\n";
		}
		if (raster.getMaxCount() > 0) {
			std::cout << "Maximum voxel radius is too low!\n";
			std::cout << raster.getMaxCount() << " particles are ignored!\n";
		}
	}
}

void OpenVDB_from_particles(OpenVDBPrimitive *level_set,
                            OpenVDBPrimitive *&mask_grid,
                            ParticleList Pa, bool mask, float mask_width,
                            float min_radius, bool trail, float trail_size)
{
	FloatGrid::Ptr ls_grid = gridPtrCast<FloatGrid>(level_set->getGridPtr());

	convert_to_levelset(Pa, ls_grid, min_radius, trail, trail_size);

	if (mask) {
		FloatGrid::Ptr mask = gridPtrCast<FloatGrid>(mask_grid->getGridPtr());

		if (mask == NULL) {
			mask = FloatGrid::create(ls_grid->background());
			mask->setGridClass(GRID_LEVEL_SET);
		}

		if (mask_width > 0.0f) {
			std::cout << "Generating mask from level set\n";
			mask->setTransform(ls_grid->transform().copy());
			Pa.radius_scale() *= (1.0f + mask_width);
			convert_to_levelset(Pa, mask, min_radius, trail, trail_size);

			if (mask_width < 1.0f) {
				FloatGrid::Ptr mask_grid_min = FloatGrid::create(ls_grid->background());
				mask_grid_min->setGridClass(GRID_LEVEL_SET);
				mask_grid_min->setTransform(ls_grid->transform().copy());
				Pa.radius_scale() *= (1.0f - mask_width) / (1.0f + mask_width);
				convert_to_levelset(Pa, mask_grid_min, min_radius, trail, trail_size);

				tools::csgDifference(*mask, *mask_grid_min);
			}
		}

		tools::sdfToFogVolume(*mask);
		mask_grid->setGrid(mask);
	}
}

}

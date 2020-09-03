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

#include "particle_tools.h"

using namespace openvdb;

ParticleList::ParticleList(size_t size, Real rad_scale, Real vel_scale)
    : m_radius_scale(rad_scale), m_velocity_scale(vel_scale)
{
  m_has_radius = true;
  m_has_velocity = true;
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

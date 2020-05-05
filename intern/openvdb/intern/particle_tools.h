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

#ifndef __PARTICLE_TOOLS_H__
#define __PARTICLE_TOOLS_H__

#include <openvdb/openvdb.h>

/* This class implements the interface needed by tools::ParticlesToLevelSet.
 * It is based on the one found in openvdb/unittest/TestParticlesToLevelSet.cc.
 */
class ParticleList {
 protected:
  struct Particle {
    openvdb::Vec3R pos, vel;
    openvdb::Real rad;
  };

  openvdb::Real m_radius_scale;
  openvdb::Real m_velocity_scale;
  std::vector<Particle> m_particle_list;
  bool m_has_radius, m_has_velocity;

 public:
  /* Required for bucketing */
  typedef openvdb::Vec3R PosType;  // value_type;

  ParticleList(size_t size, openvdb::Real rad_scale = 1.0, openvdb::Real vel_scale = 1.0);

  void add(const openvdb::Vec3R &p, const openvdb::Real &r, const openvdb::Vec3R &v);

  openvdb::Real &radius_scale();
  const openvdb::Real &radius_scale() const;
  openvdb::Real &velocity_scale();

  bool has_radius() const;
  bool has_velocity() const;
  void set_flags(const bool has_radius, const bool has_velocity);

  /*! \return coordinate bbox in the space of the specified transfrom */
  openvdb::math::CoordBBox getBBox(const openvdb::GridBase &grid);

  openvdb::Vec3R pos(int n) const;
  openvdb::Vec3R vel(int n) const;
  openvdb::Real radius(int n) const;

  /* The methods below are the only ones required by tools::ParticleToLevelSet
   * We return by value since the radius and velocities are modified by the
   * scaling factors!
   * Also these methods are all assumed to be thread-safe.
   */

  /* Return the total number of particles in list.
   * Always required!
   */
  int size() const;

  /* Get the world space position of n'th particle.
   * Required by ParticlesToLevelSet::rasterizeSphere(*this, radius).
   */
  void getPos(size_t n, openvdb::Vec3R &pos) const;
  void getPosRad(size_t n, openvdb::Vec3R &pos, openvdb::Real &rad) const;
  void getPosRadVel(size_t n, openvdb::Vec3R &pos, openvdb::Real &rad, openvdb::Vec3R &vel) const;

  /* The method below is only required for attribute transfer. */
  void getAtt(size_t n, openvdb::Index32 &att) const;
};

#endif /* __PARTICLE_TOOLS_H__ */

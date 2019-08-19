/*
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
 */

#include "openvdb_level_set.h"
#include "openvdb_util.h"
#include "openvdb_capi.h"
#include "MEM_guardedalloc.h"
#include "openvdb/tools/Composite.h"
#include "openvdb/tools/ParticlesToLevelSet.h"
#include "intern/particle_tools.h"

OpenVDBLevelSet::OpenVDBLevelSet()
{
  openvdb::initialize();
}

OpenVDBLevelSet::~OpenVDBLevelSet()
{
}

void OpenVDBLevelSet::OpenVDB_mesh_to_level_set(const float *vertices,
                                                const unsigned int *faces,
                                                const unsigned int totvertices,
                                                const unsigned int totfaces,
                                                openvdb::math::Transform::Ptr xform)
{
  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<openvdb::Vec4I> quads;

  for (unsigned int i = 0; i < totvertices; i++) {
    openvdb::Vec3s v(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
    points.push_back(v);
  }

  for (unsigned int i = 0; i < totfaces; i++) {
    openvdb::Vec3I f(faces[i * 3], faces[i * 3 + 1], faces[i * 3 + 2]);
    triangles.push_back(f);
  }

  this->grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
      *xform, points, triangles, quads, 1);
}

void OpenVDBLevelSet::OpenVDB_volume_to_mesh(OpenVDBVolumeToMeshData *mesh,
                                             const double isovalue,
                                             const double adaptivity,
                                             const bool relax_disoriented_triangles)
{
  std::vector<openvdb::Vec3s> out_points;
  std::vector<openvdb::Vec4I> out_quads;
  std::vector<openvdb::Vec3I> out_tris;
  openvdb::tools::volumeToMesh<openvdb::FloatGrid>(*this->grid,
                                                   out_points,
                                                   out_tris,
                                                   out_quads,
                                                   isovalue,
                                                   adaptivity,
                                                   relax_disoriented_triangles);
  mesh->vertices = (float *)MEM_malloc_arrayN(
      out_points.size(), 3 * sizeof(float), "openvdb remesher out verts");
  mesh->quads = (unsigned int *)MEM_malloc_arrayN(
      out_quads.size(), 4 * sizeof(unsigned int), "openvdb remesh out quads");
  mesh->triangles = NULL;
  if (out_tris.size() > 0) {
    mesh->triangles = (unsigned int *)MEM_malloc_arrayN(
        out_tris.size(), 3 * sizeof(unsigned int), "openvdb remesh out tris");
  }

  mesh->totvertices = out_points.size();
  mesh->tottriangles = out_tris.size();
  mesh->totquads = out_quads.size();

  for (unsigned int i = 0; i < out_points.size(); i++) {
    mesh->vertices[i * 3] = out_points[i].x();
    mesh->vertices[i * 3 + 1] = out_points[i].y();
    mesh->vertices[i * 3 + 2] = out_points[i].z();
  }

  for (unsigned int i = 0; i < out_quads.size(); i++) {
    mesh->quads[i * 4] = out_quads[i].x();
    mesh->quads[i * 4 + 1] = out_quads[i].y();
    mesh->quads[i * 4 + 2] = out_quads[i].z();
    mesh->quads[i * 4 + 3] = out_quads[i].w();
  }

  for (unsigned int i = 0; i < out_tris.size(); i++) {
    mesh->triangles[i * 3] = out_tris[i].x();
    mesh->triangles[i * 3 + 1] = out_tris[i].y();
    mesh->triangles[i * 3 + 2] = out_tris[i].z();
  }
}

void OpenVDBLevelSet::OpenVDB_level_set_filter(OpenVDBLevelSet_FilterType filter_type,
                                               int width,
                                               int iterations,
                                               int filter_bias)
{

  if (!this->grid) {
    return;
  }

  if (this->grid && this->grid->getGridClass() != openvdb::GRID_LEVEL_SET) {
    return;
  }

  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*this->grid);
  filter.setSpatialScheme((openvdb::math::BiasedGradientScheme)filter_bias);
  switch (filter_type) {
    case OPENVDB_LEVELSET_FILTER_GAUSSIAN:
      filter.gaussian(width);
      break;
    case OPENVDB_LEVELSET_FILTER_MEDIAN:
      filter.median(width);
      break;
    case OPENVDB_LEVELSET_FILTER_MEAN:
      filter.mean(width);
      break;
    case OPENVDB_LEVELSET_FILTER_MEAN_CURVATURE:
      filter.meanCurvature();
      break;
    case OPENVDB_LEVELSET_FILTER_LAPLACIAN:
      filter.laplacian();
      break;
    case OPENVDB_LEVELSET_FILTER_DILATE:
      filter.offset(-iterations / 100.0);
      break;
    case OPENVDB_LEVELSET_FILTER_ERODE:
      filter.offset(iterations / 100.0);
      break;
  }
}
openvdb::FloatGrid::Ptr OpenVDBLevelSet::OpenVDB_CSG_operation(
    openvdb::FloatGrid::Ptr gridA,
    openvdb::FloatGrid::Ptr gridB,
    OpenVDBLevelSet_CSGOperation operation)
{
  openvdb::FloatGrid::Ptr gridA_copy = gridA;  //->deepCopy();
  openvdb::FloatGrid::Ptr gridB_copy = gridB;  //->deepCopy();

  switch (operation) {
    case OPENVDB_LEVELSET_CSG_UNION:
      openvdb::tools::csgUnion(*gridA_copy, *gridB_copy);
      break;
    case OPENVDB_LEVELSET_CSG_DIFFERENCE:
      openvdb::tools::csgDifference(*gridA_copy, *gridB_copy);
      break;
    case OPENVDB_LEVELSET_CSG_INTERSECTION:
      openvdb::tools::csgIntersection(*gridA_copy, *gridB_copy);
      break;
  }

  return gridA_copy;
}

openvdb::FloatGrid::Ptr OpenVDBLevelSet::OpenVDB_level_set_get_grid()
{
  return this->grid;
}

void OpenVDBLevelSet::OpenVDB_level_set_set_grid(openvdb::FloatGrid::Ptr grid)
{
  this->grid = grid;
}

void OpenVDBLevelSet::OpenVDB_particles_to_level_set(ParticleList part_list,
                                    float min_radius,
                                    bool trail,
                                    float trail_size)
{
  /* Note: the second template argument here is the particles' attributes type,
   * if any. As this function will later call ParticleList::getAtt(index, attribute),
   * we pass void for two reasons: first, no attributes are defined for the
   * particles (yet), and second, disable using attributes for generating
   * the level set.
   *
   * TODO(kevin): quite useless to know that if we don't have the third argument...
   */
  openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid, void> raster(*(this->grid));
  /* a grain size of zero disables threading */
  raster.setGrainSize(0);
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

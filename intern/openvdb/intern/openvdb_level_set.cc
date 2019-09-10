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
#include "intern/openvdb_filter.h"
#include "intern/openvdb_mesher.h"

OpenVDBLevelSet::OpenVDBLevelSet()
{
  openvdb::initialize();
}

OpenVDBLevelSet::~OpenVDBLevelSet()
{
}

void OpenVDBLevelSet::mesh_to_level_set(const float *vertices,
                                        const unsigned int *faces,
                                        const unsigned int totvertices,
                                        const unsigned int totfaces,
                                        const openvdb::math::Transform::Ptr &xform)
{
  std::vector<openvdb::Vec3s> points(totvertices);
  // maximum if all primitives are split, mark 4th vertex as invalid since we have tris only
  std::vector<uint32_t> vert_tri(totfaces * 4); 
  std::vector<openvdb::Vec3I> triangles(totfaces);
  std::vector<openvdb::Vec4I> quads;

  for (unsigned int i = 0; i < totvertices; i++) {
    points[i] = openvdb::Vec3s(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
  }

  for (unsigned int i = 0; i < totfaces; i++) {
    triangles[i] = openvdb::Vec3I(faces[i * 3], faces[i * 3 + 1], faces[i * 3 + 2]);
    vert_tri[i * 4] = triangles[i][0];
    vert_tri[i * 4 + 1] = triangles[i][1];
    vert_tri[i * 4 + 2] = triangles[i][2];
    vert_tri[i * 4 + 3] = openvdb::util::INVALID_IDX;
  }

  this->set_points(points);
  this->set_triangles(triangles);
  this->set_vert_tri(vert_tri);

  this->grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
      *xform, points, triangles, quads, 1);
}

using BoolTreeType = openvdb::FloatGrid::TreeType::template ValueConverter<bool>::Type;
void OpenVDBLevelSet::sharpenFeaturesPre(float edge_tolerance)
{
  using IntGridT = typename openvdb::FloatGrid::template ValueConverter<openvdb::Int32>::Type;
  typename IntGridT::Ptr indexGrid;  // replace

  std::vector<openvdb::Vec3s> pointList;
  std::vector<openvdb::Vec4I> primList;

  pointList.resize(this->get_points().size());
  primList.resize(this->get_triangles().size());

  // Check for reference mesh
  //tbb::parallel_for(tbb::blocked_range<size_t>(0, this->get_points().size()),
  //                  openvdb::tools::TransformOp(*this, *transform, pointList));
  openvdb::tools::TransformOp op = openvdb::tools::TransformOp(*this, this->grid->transform(), pointList);
  op(tbb::blocked_range<size_t>(0, this->get_points().size()));

  // UTparallelFor(GA_SplittableRange(refGeo->getPointRange()),
  //               hvdb::TransformOp(refGeo, *transform, pointList));

  //tbb::parallel_for(tbb::blocked_range<size_t>(0, this->get_triangles().size()),
  //                  openvdb::tools::PrimCpyOp(*this, primList));
  openvdb::tools::PrimCpyOp op2 = openvdb::tools::PrimCpyOp(*this, primList);
  op2(tbb::blocked_range<size_t>(0, this->get_triangles().size()));

  // UTparallelFor(GA_SplittableRange(refGeo->getPrimitiveRange()),
  //              hvdb::PrimCpyOp(refGeo, primList));

  openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> mesh(pointList,
                                                                                  primList);

  float bandWidth = 3.0;

#if 0
  if (this->grid->getGridClass() != openvdb::GRID_LEVEL_SET) {
    bandWidth = float(backgroundValue) / float(transform->voxelSize()[0]);
  }
#endif

  indexGrid.reset(new IntGridT(0));

  this->refGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
      mesh, this->grid->transform(), bandWidth, bandWidth, 0, indexGrid.get());

  edgeData.convert(pointList, primList);

  this->maskTree = typename BoolTreeType::Ptr(new BoolTreeType(false));
  this->maskTree->topologyUnion(indexGrid->tree());
  openvdb::tree::LeafManager<BoolTreeType> maskLeafs(*(this->maskTree));

  openvdb::tools::GenAdaptivityMaskOp<IntGridT::TreeType, BoolTreeType> op3(
      *this, indexGrid->tree(), maskLeafs, edge_tolerance);
  op3.run();

  openvdb::tools::pruneInactive(*(this->maskTree));

  openvdb::tools::dilateVoxels(*this->maskTree, 2);
}

void OpenVDBLevelSet::volume_to_mesh(OpenVDBVolumeToMeshData *mesh,
                                     const double isovalue,
                                     const double adaptivity,
                                     const bool relax_disoriented_triangles)
{
  std::vector<openvdb::Vec3s> out_points_tmp, out_points;
  std::vector<openvdb::Vec4I> out_quads;
  std::vector<openvdb::Vec3I> out_tris;

  openvdb::tools::volumeToMesh<openvdb::FloatGrid>(*this->grid,
                                                   out_points_tmp,
                                                   out_tris,
                                                   out_quads,
                                                   isovalue,
                                                   adaptivity,
                                                   relax_disoriented_triangles,
                                                   this->maskTree,
                                                   this->refGrid);

  // sharpen Features, after meshing
  this->set_out_points(out_points_tmp);

  if (this->sharpen_features) {

   // tbb::parallel_for(
   //     tbb::blocked_range<size_t>(0, this->get_out_points().size()),
          openvdb::tools::SharpenFeaturesOp op = openvdb::tools::SharpenFeaturesOp(*this, this->edgeData, this->grid->transform(), this->maskTree.get());
          op(tbb::blocked_range<size_t>(0, this->get_out_points().size()));
    // UTparallelFor(GA_SplittableRange(gdp->getPointRange()),
    //              hvdb::SharpenFeaturesOp(
    //                  *gdp, ref_mesh, edgeData, *transform, surfaceGroup, maskTree.get()));
  }

  out_points = this->get_out_points();

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

  for (size_t i = 0; i < out_points.size(); i++) {
    mesh->vertices[i * 3] = out_points[i].x();
    mesh->vertices[i * 3 + 1] = out_points[i].y();
    mesh->vertices[i * 3 + 2] = out_points[i].z();
  }

  for (size_t i = 0; i < out_quads.size(); i++) {
    mesh->quads[i * 4] = out_quads[i].x();
    mesh->quads[i * 4 + 1] = out_quads[i].y();
    mesh->quads[i * 4 + 2] = out_quads[i].z();
    mesh->quads[i * 4 + 3] = out_quads[i].w();
  }

  for (size_t i = 0; i < out_tris.size(); i++) {
    mesh->triangles[i * 3] = out_tris[i].x();
    mesh->triangles[i * 3 + 1] = out_tris[i].y();
    mesh->triangles[i * 3 + 2] = out_tris[i].z();
  }
}

static const openvdb::Vec3s normal_tri_v3(const openvdb::Vec3s v1,
                                          const openvdb::Vec3s v2,
                                          const openvdb::Vec3s v3)
{
  openvdb::Vec3s n, n1, n2;

  n1[0] = v1[0] - v2[0];
  n2[0] = v2[0] - v3[0];
  n1[1] = v1[1] - v2[1];
  n2[1] = v2[1] - v3[1];
  n1[2] = v1[2] - v2[2];
  n2[2] = v2[2] - v3[2];
  n[0] = n1[1] * n2[2] - n1[2] * n2[1];
  n[1] = n1[2] * n2[0] - n1[0] * n2[2];
  n[2] = n1[0] * n2[1] - n1[1] * n2[0];

  n.normalize();

  return n;
}

openvdb::Vec3s OpenVDBLevelSet::face_normal(uint32_t pntOffset)
{
  //std::vector<openvdb::Vec3s> vnormals = this->get_vert_normals();
  std::vector<openvdb::Vec3s> verts = this->get_points();
  std::vector<uint32_t> vert_tri = this->get_vert_tri();
  int x = (pntOffset / 4); 
  int v1 = vert_tri[x * 4];
  int v2 = vert_tri[x * 4 + 1];
  int v3 = vert_tri[x * 4 + 2];

  openvdb::Vec3s normal = normal_tri_v3(verts[v1], verts[v2], verts[v3]);
  return normal;
}

void OpenVDBLevelSet::filter(OpenVDBLevelSet_FilterType filter_type,
                             int width,
                             int iterations,
                             float sigma,
                             float distance,
                             OpenVDBLevelSet_FilterBias filter_bias,
                             bool sharpen_features,
                             float edge_tolerance)
{

  if (!this->grid) {
    return;
  }

  if (this->grid->getGridClass() != openvdb::GRID_LEVEL_SET) {
    return;
  }

  openvdb::tools::ExtendedLevelSetFilter<openvdb::FloatGrid> filter(*this->grid);
  filter.setSpatialScheme((openvdb::math::BiasedGradientScheme)filter_bias);
  switch (filter_type) {
    case OPENVDB_LEVELSET_FILTER_GAUSSIAN:
      filter.gaussianI(width, iterations, sigma);
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
      filter.offset(-distance);
      break;
    case OPENVDB_LEVELSET_FILTER_ERODE:
      filter.offset(distance);
      break;
    case OPENVDB_LEVELSET_FILTER_NONE:
      break;
  }

  this->sharpen_features = sharpen_features;
  if (sharpen_features) {
    this->sharpenFeaturesPre(edge_tolerance);
  }
}
openvdb::FloatGrid::Ptr OpenVDBLevelSet::CSG_operation_apply(
    const openvdb::FloatGrid::Ptr &gridA,
    const openvdb::FloatGrid::Ptr &gridB,
    OpenVDBLevelSet_CSGOperation operation)
{
  switch (operation) {
    case OPENVDB_LEVELSET_CSG_UNION:
      openvdb::tools::csgUnion(*gridA, *gridB);
      break;
    case OPENVDB_LEVELSET_CSG_DIFFERENCE:
      openvdb::tools::csgDifference(*gridA, *gridB);
      break;
    case OPENVDB_LEVELSET_CSG_INTERSECTION:
      openvdb::tools::csgIntersection(*gridA, *gridB);
      break;
  }

  return gridA;
}

const openvdb::FloatGrid::Ptr &OpenVDBLevelSet::get_grid()
{
  return this->grid;
}

const std::vector<openvdb::Vec3s> &OpenVDBLevelSet::get_points()
{
  return this->points;
}

const std::vector<openvdb::Vec3s> &OpenVDBLevelSet::get_out_points()
{
  return this->out_points;
}

const std::vector<openvdb::Vec3I> &OpenVDBLevelSet::get_triangles()
{
  return this->triangles;
}

const std::vector<uint32_t> &OpenVDBLevelSet::get_vert_tri()
{
  return this->vert_tri;
}

void OpenVDBLevelSet::set_grid(const openvdb::FloatGrid::Ptr &grid)
{
  this->grid = grid;
}

void OpenVDBLevelSet::set_points(const std::vector<openvdb::Vec3s> &points)
{
  this->points = points;
}

void OpenVDBLevelSet::set_out_points(const std::vector<openvdb::Vec3s> &out_points)
{
  this->out_points = out_points;
}

void OpenVDBLevelSet::set_triangles(const std::vector<openvdb::Vec3I> &triangles)
{
  this->triangles = triangles;
}

void OpenVDBLevelSet::set_vert_tri(const std::vector<uint32_t> &vert_tri)
{
  this->vert_tri = vert_tri;
}

void OpenVDBLevelSet::particles_to_level_set(ParticleList part_list,
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
      std::cout << "Minimum voxel radius is too high!\n";
      std::cout << raster.getMinCount() << " particles are ignored!\n";
    }
    if (raster.getMaxCount() > 0) {
      std::cout << "Maximum voxel radius is too low!\n";
      std::cout << raster.getMaxCount() << " particles are ignored!\n";
    }
  }
}

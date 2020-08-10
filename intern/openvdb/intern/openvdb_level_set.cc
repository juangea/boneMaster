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
#include "MEM_guardedalloc.h"
#include "intern/openvdb_filter.h"
#include "intern/openvdb_mesher.h"
#include "intern/particle_tools.h"
#include "openvdb/tools/Composite.h"
#include "openvdb/tools/ParticlesToLevelSet.h"
#include "openvdb_capi.h"
#include "openvdb_util.h"

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
                                        const openvdb::math::Transform::Ptr &xform,
                                        bool do_convert,
                                        bool do_add,
                                        OpenVDBLevelSet_CSGOperation op)
{
  unsigned int v = 0;
  unsigned int f = 0;
  std::vector<openvdb::Vec3s> points;
  std::vector<uint32_t> vert_tri;
  std::vector<openvdb::Vec3I> triangles;

  if (do_add) {
    points = this->get_points();
    vert_tri = this->get_vert_tri();
    triangles = this->get_triangles();
    v = points.size();
    f = triangles.size();

    points.resize(points.size() + totvertices);
    vert_tri.resize(vert_tri.size() + 4 * totfaces);
    triangles.resize(triangles.size() + totfaces);
  }
  else {
    points = std::vector<openvdb::Vec3s>(totvertices);
    // maximum if all primitives are split, mark 4th vertex as invalid since we have tris only
    vert_tri = std::vector<uint32_t>(totfaces * 4);
    triangles = std::vector<openvdb::Vec3I>(totfaces);
  }

  std::vector<openvdb::Vec4I> quads;

  CSGOperand operand;
  operand.face_start = f;
  operand.face_count = totfaces;
  operand.vert_start = v;
  operand.vert_count = totvertices;
  operand.operation = op;

  this->csg_operands.push_back(operand);

  for (unsigned int i = 0; i < totvertices; i++) {
    points[i + v] = openvdb::Vec3s(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
  }

  for (unsigned int i = 0; i < totfaces; i++) {
    if (op == 0) {
      // union, keep normals as is
      triangles[i + f] = openvdb::Vec3I(
          faces[i * 3] + v, faces[i * 3 + 1] + v, faces[i * 3 + 2] + v);
    }
    else {
      // difference or intersect, try to flip "normals" here by changing vert order in triangle
      triangles[i + f] = openvdb::Vec3I(
          faces[i * 3] + v, faces[i * 3 + 1] + v, faces[i * 3 + 2] + v);
    }

    vert_tri[(i + f) * 4] = triangles[i + f][0];
    vert_tri[(i + f) * 4 + 1] = triangles[i + f][1];
    vert_tri[(i + f) * 4 + 2] = triangles[i + f][2];
    vert_tri[(i + f) * 4 + 3] = openvdb::util::INVALID_IDX;
  }

  this->set_points(points);
  this->set_triangles(triangles);
  this->set_vert_tri(vert_tri);

  if (do_convert) {
    this->grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
        *xform, points, triangles, quads, 1);
  }
}

using BoolTreeType = openvdb::FloatGrid::TreeType::template ValueConverter<bool>::Type;
void OpenVDBLevelSet::sharpenFeaturesPre(float edge_tolerance)
{
  using IntGridT = typename openvdb::FloatGrid::template ValueConverter<openvdb::Int32>::Type;
  typename IntGridT::Ptr indexGrid;  // replace

  std::vector<openvdb::Vec3s> pointList_total;
  std::vector<openvdb::Vec4I> primList_total;

  for (int i = 0; i < this->csg_operands.size(); i++) {
    pointList_total.resize(pointList_total.size() + this->csg_operands[i].vert_count);
    primList_total.resize(primList_total.size() + this->csg_operands[i].face_count);
  }

  for (int i = 0; i < this->csg_operands.size(); i++) {

    CSGOperand &operand = this->csg_operands[i];
    std::vector<openvdb::Vec3s> pointList;
    std::vector<openvdb::Vec4I> primList;
    pointList.resize(operand.vert_count);
    primList.resize(operand.face_count);

    // Check for reference mesh
    openvdb::tools::TransformOp op = openvdb::tools::TransformOp(
        *this, this->grid->transform(), pointList, pointList_total, operand.vert_start);
    tbb::blocked_range<size_t> range = tbb::blocked_range<size_t>(0, operand.vert_count);
    tbb::parallel_for(range, op);
    // op(range);

    openvdb::tools::PrimCpyOp op2 = openvdb::tools::PrimCpyOp(
        *this, primList, primList_total, operand.vert_start, operand.face_start);
    tbb::blocked_range<size_t> range2 = tbb::blocked_range<size_t>(0, operand.face_count);
    tbb::parallel_for(range2, op2);
    // op2(range2);

    openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> mesh(pointList,
                                                                                    primList);

    float bandWidth = 3.0;

    if (i == 0) {
      indexGrid.reset(new IntGridT(0));
      this->refGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
          mesh, this->grid->transform(), bandWidth, bandWidth, 0, indexGrid.get());
    }
    else {
      typename IntGridT::Ptr tmpIndexGrid;
      tmpIndexGrid.reset(new IntGridT(0));
      openvdb::FloatGrid::Ptr tmpGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
          mesh, this->grid->transform(), bandWidth, bandWidth, 0, tmpIndexGrid.get());

      // compSum, compMax, compMul ?
      openvdb::tools::compMax(this->refGrid->tree(), tmpGrid->tree());
      openvdb::tools::compMax(indexGrid->tree(), tmpIndexGrid->tree());
    }
  }

  edgeData.convert(pointList_total, primList_total);

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
                                     const bool relax_disoriented_triangles,
                                     OpenVDBLevelSet *mask)
{
  std::vector<openvdb::Vec3s> out_points_tmp, out_points;
  std::vector<openvdb::Vec4I> out_quads_tmp, out_quads;
  std::vector<openvdb::Vec3I> out_tris;

  openvdb::tools::volumeToMesh<openvdb::FloatGrid>(*this->grid,
                                                   out_points_tmp,
                                                   out_tris,
                                                   out_quads_tmp,
                                                   isovalue,
                                                   adaptivity,
                                                   relax_disoriented_triangles,
                                                   this->maskTree,
                                                   this->refGrid,
                                                   mask ? mask->grid : nullptr);

  // sharpen Features, after meshing
  this->set_out_points(out_points_tmp);
  this->set_out_quads(out_quads_tmp);

  out_quads = this->get_out_quads();

  if (this->sharpen_features) {
    out_points.resize(out_points_tmp.size());
    openvdb::tools::SharpenFeaturesOp op = openvdb::tools::SharpenFeaturesOp(
        *this, this->edgeData, this->grid->transform(), this->maskTree.get(), out_points);
    tbb::blocked_range<size_t> range = tbb::blocked_range<size_t>(0, out_points.size());
    tbb::parallel_for(range, op);
    // op(range);
  }
  else {
    out_points = this->get_out_points();
  }

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
  // std::vector<openvdb::Vec3s> vnormals = this->get_vert_normals();
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

const std::vector<openvdb::Vec4I> &OpenVDBLevelSet::get_out_quads()
{
  return this->out_quads;
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

void OpenVDBLevelSet::set_out_quads(const std::vector<openvdb::Vec4I> &out_quads)
{
  this->out_quads = out_quads;
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

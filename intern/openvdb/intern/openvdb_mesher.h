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
 * The Original Code is Copyright (C) 2019 Martin Felke.
 * All rights reserved.
 */

#ifndef __OPENVDB_MESHER_H__
#define __OPENVDB_MESHER_H__

#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include "openvdb_level_set.h"

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {

template<typename GridType>
inline typename std::enable_if<std::is_scalar<typename GridType::ValueType>::value, void>::type
doVolumeToMesh(const GridType &grid,
               std::vector<Vec3s> &points,
               std::vector<Vec3I> &triangles,
               std::vector<Vec4I> &quads,
               double isovalue,
               double adaptivity,
               bool relaxDisorientedTriangles,
               BoolTreeType::Ptr adaptivityMask,
               FloatGrid::Ptr refGrid,
               FloatGrid::Ptr surfaceMask)
{

  VolumeToMesh mesher(isovalue, adaptivity, relaxDisorientedTriangles);

  mesher.setAdaptivityMask(adaptivityMask);
  mesher.setRefGrid(refGrid, adaptivity);
  mesher.setSurfaceMask(surfaceMask, true);
  mesher(grid);

  // Preallocate the point list
  points.clear();
  points.resize(mesher.pointListSize());

  {  // Copy points
    volume_to_mesh_internal::PointListCopy ptnCpy(mesher.pointList(), points);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()), ptnCpy);
    mesher.pointList().reset(nullptr);
  }

  PolygonPoolList &polygonPoolList = mesher.polygonPoolList();

  {  // Preallocate primitive lists
    size_t numQuads = 0, numTriangles = 0;
    for (size_t n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
      openvdb::tools::PolygonPool &polygons = polygonPoolList[n];
      numTriangles += polygons.numTriangles();
      numQuads += polygons.numQuads();
    }

    triangles.clear();
    triangles.resize(numTriangles);
    quads.clear();
    quads.resize(numQuads);
  }

  // Copy primitives
  size_t qIdx = 0, tIdx = 0;
  for (size_t n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
    openvdb::tools::PolygonPool &polygons = polygonPoolList[n];

    for (size_t i = 0, I = polygons.numQuads(); i < I; ++i) {
      quads[qIdx++] = polygons.quad(i);
    }

    for (size_t i = 0, I = polygons.numTriangles(); i < I; ++i) {
      triangles[tIdx++] = polygons.triangle(i);
    }
  }
}

template<typename GridType>
inline void volumeToMesh(const GridType &grid,
                         std::vector<Vec3s> &points,
                         std::vector<Vec3I> &triangles,
                         std::vector<Vec4I> &quads,
                         double isovalue,
                         double adaptivity,
                         bool relaxDisorientedTriangles,
                         BoolTreeType::Ptr adaptivityMask,
                         FloatGrid::Ptr refGrid,
                         FloatGrid::Ptr surfaceMask)
{
  doVolumeToMesh(grid,
                 points,
                 triangles,
                 quads,
                 isovalue,
                 adaptivity,
                 relaxDisorientedTriangles,
                 adaptivityMask,
                 refGrid,
                 surfaceMask);
}

// ray bbox helper code, since houdini function aint useable here
// taken from scratchapixel.com
class Ray {
 public:
  Ray(const Vec3f &orig, const Vec3f &dir) : orig(orig), dir(dir)
  {
    invdir = 1 / dir;
    sign[0] = (invdir.x() < 0);
    sign[1] = (invdir.y() < 0);
    sign[2] = (invdir.z() < 0);
  }
  Vec3f orig, dir;  // ray orig and dir
  Vec3f invdir;
  int sign[3];
};

class AABBox {
 public:
  AABBox(const Vec3f &b0, const Vec3f &b1)
  {
    bounds[0] = b0, bounds[1] = b1;
  }
  bool intersect(const Ray &r, float &t) const
  {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = (bounds[r.sign[0]].x() - r.orig.x()) * r.invdir.x();
    tmax = (bounds[1 - r.sign[0]].x() - r.orig.x()) * r.invdir.x();
    tymin = (bounds[r.sign[1]].y() - r.orig.y()) * r.invdir.y();
    tymax = (bounds[1 - r.sign[1]].y() - r.orig.y()) * r.invdir.y();

    if ((tmin > tymax) || (tymin > tmax))
      return false;

    if (tymin > tmin)
      tmin = tymin;
    if (tymax < tmax)
      tmax = tymax;

    tzmin = (bounds[r.sign[2]].z() - r.orig.z()) * r.invdir.z();
    tzmax = (bounds[1 - r.sign[2]].z() - r.orig.z()) * r.invdir.z();

    if ((tmin > tzmax) || (tzmin > tmax))
      return false;

    if (tzmin > tmin)
      tmin = tzmin;
    if (tzmax < tmax)
      tmax = tzmax;

    t = tmin;

    if (t < 0) {
      t = tmax;
      if (t < 0)
        return false;
    }

    return true;
  }
  Vec3f bounds[2];
};

/// TBB body object for threaded sharp feature construction
template<typename IndexTreeType, typename BoolTreeType> class GenAdaptivityMaskOp {
 public:
  using BoolLeafManager = openvdb::tree::LeafManager<BoolTreeType>;

  GenAdaptivityMaskOp(OpenVDBLevelSet &lvl,
                      const IndexTreeType &indexTree,
                      BoolLeafManager &,
                      float edgetolerance = 0.0);

  void run(bool threaded = true);

  void operator()(const tbb::blocked_range<size_t> &) const;

 private:
  OpenVDBLevelSet &mLvl;
  const IndexTreeType &mIndexTree;
  BoolLeafManager &mLeafs;
  float mEdgeTolerance;
};

template<typename IndexTreeType, typename BoolTreeType>
GenAdaptivityMaskOp<IndexTreeType, BoolTreeType>::GenAdaptivityMaskOp(
    OpenVDBLevelSet &lvl,
    const IndexTreeType &indexTree,
    BoolLeafManager &leafMgr,
    float edgetolerance)
    : mLvl(lvl), mIndexTree(indexTree), mLeafs(leafMgr), mEdgeTolerance(edgetolerance)
{
  mEdgeTolerance = std::max(0.0001f, mEdgeTolerance);
  mEdgeTolerance = std::min(1.0f, mEdgeTolerance);
}

template<typename IndexTreeType, typename BoolTreeType>
void GenAdaptivityMaskOp<IndexTreeType, BoolTreeType>::run(bool threaded)
{
  if (threaded) {
    tbb::parallel_for(mLeafs.getRange(), *this);
  }
  else {
    (*this)(mLeafs.getRange());
  }
}

template<typename IndexTreeType, typename BoolTreeType>
void GenAdaptivityMaskOp<IndexTreeType, BoolTreeType>::operator()(
    const tbb::blocked_range<size_t> &range) const
{
  using IndexAccessorType = typename openvdb::tree::ValueAccessor<const IndexTreeType>;
  IndexAccessorType idxAcc(mIndexTree);

  // UT_Vector3 tmpN, normal;
  // GA_Offset primOffset;
  int tmpIdx;

  openvdb::Coord ijk, nijk;
  typename BoolTreeType::LeafNodeType::ValueOnIter iter;

  for (size_t n = range.begin(); n < range.end(); ++n) {
    iter = mLeafs.leaf(n).beginValueOn();
    for (; iter; ++iter) {
      ijk = iter.getCoord();

      bool edgeVoxel = false;

      int idx = idxAcc.getValue(ijk);
      if (idx < 0) {
        // iter.setValueOff();
        edgeVoxel = true;
        break;
      }
      // calculate face normal...
      // normal = mRefGeo.getGEOPrimitive(primOffset)->computeNormal();
      openvdb::Vec3s normal = mLvl.face_normal(idx * 4);

      for (size_t i = 0; i < 18; ++i) {
        nijk = ijk + openvdb::util::COORD_OFFSETS[i];
        if (idxAcc.probeValue(nijk, tmpIdx) && tmpIdx != idx) {
          if (tmpIdx < 0) {
            edgeVoxel = true;
            break;
            // continue;
          }

          openvdb::Vec3s tmpN = mLvl.face_normal(tmpIdx * 4);

          // dot can be negative too !
          float dot = normal.dot(tmpN);
          if (dot < mEdgeTolerance) {
            edgeVoxel = true;
            break;
          }
        }
      }

      if (!edgeVoxel)
        iter.setValueOff();
    }
  }
}

////////////////////////////////////////

////////////////////////////////////////
using RangeT = tbb::blocked_range<size_t>;

/// TBB body object for threaded world to voxel space transformation and copy of points
class TransformOp {
 public:
  TransformOp(OpenVDBLevelSet &lvl,
              const openvdb::math::Transform &transform,
              std::vector<openvdb::Vec3s> &pointList,
              std::vector<openvdb::Vec3s> &pointListTotal,
              unsigned int vert_start);

  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  const openvdb::math::Transform &mTransform;
  std::vector<openvdb::Vec3s> *const mPointList;
  std::vector<openvdb::Vec3s> *const mPointListTotal;
  unsigned int mVertStart;
};

////////////////////////////////////////

TransformOp::TransformOp(OpenVDBLevelSet &lvl,
                         const openvdb::math::Transform &transform,
                         std::vector<openvdb::Vec3s> &pointList,
                         std::vector<openvdb::Vec3s> &pointListTotal,
                         unsigned int vert_start)
    : mLvl(lvl),
      mTransform(transform),
      mPointList(&pointList),
      mPointListTotal(&pointListTotal),
      mVertStart(vert_start)
{
}

void TransformOp::operator()(const RangeT &r) const
{
  openvdb::Vec3s pos;
  openvdb::Vec3d ipos;

  for (int i = r.begin(); i < r.end(); i++) {
    pos = mLvl.get_points()[i + mVertStart];
    ipos = mTransform.worldToIndex(openvdb::Vec3d(pos.x(), pos.y(), pos.z()));
    (*mPointList)[i] = openvdb::Vec3s(ipos);
    (*mPointListTotal)[i + mVertStart] = openvdb::Vec3s(ipos);
  }
}

/// @brief   TBB body object for threaded primitive copy
/// @details Produces a primitive-vertex index list.
class PrimCpyOp {
 public:
  PrimCpyOp(OpenVDBLevelSet &mLvl,
            std::vector<openvdb::Vec4I> &primList,
            std::vector<openvdb::Vec4I> &primListTotal,
            unsigned int vert_start,
            unsigned int face_start);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<openvdb::Vec4I> *const mPrimList;
  std::vector<openvdb::Vec4I> *const mPrimListTotal;
  unsigned int mVertStart;
  unsigned int mFaceStart;
};

////////////////////////////////////////

PrimCpyOp::PrimCpyOp(OpenVDBLevelSet &lvl,
                     std::vector<openvdb::Vec4I> &primList,
                     std::vector<openvdb::Vec4I> &primListTotal,
                     unsigned int vert_start,
                     unsigned int face_start)
    : mLvl(lvl),
      mPrimList(&primList),
      mPrimListTotal(&primListTotal),
      mVertStart(vert_start),
      mFaceStart(face_start)
{
}

void PrimCpyOp::operator()(const RangeT &r) const
{
  openvdb::Vec4I prim;
  openvdb::Vec4I primT;

  for (int i = r.begin(); i < r.end(); i++) {
    for (int vtx = 0; vtx < 3; ++vtx) {
      prim[vtx] = mLvl.get_triangles()[i + mFaceStart][vtx] - mVertStart;
      primT[vtx] = mLvl.get_triangles()[i + mFaceStart][vtx];
    }
    prim[3] = openvdb::util::INVALID_IDX;
    primT[3] = openvdb::util::INVALID_IDX;
    (*mPrimList)[i] = prim;
    (*mPrimListTotal)[i + mFaceStart] = primT;
  }
}

#if 0
class PointCpyOp {
 public:
  PointCpyOp(OpenVDBLevelSet &mLvl, std::vector<openvdb::Vec3s> &pointList);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<openvdb::Vec3s> *const mPointList;
};

PointCpyOp::PointCpyOp(OpenVDBLevelSet &lvl, std::vector<openvdb::Vec3s> &pointList)
    : mLvl(lvl), mPointList(&pointList)
{
}

void PointCpyOp::operator()(const RangeT &r) const
{
  openvdb::Vec3s pos;
  for (int i = r.begin(); i < r.end(); i++) {
    pos = mLvl.get_out_points[i];
    (*mPointList)[i] = openvdb::Vec3s(pos.x(), pos.y(), pos.z());
  }
}

/// @brief   TBB body object for threaded vertex normal generation
/// @details Averages face normals from all similarly oriented primitives,
///          that share the same vertex-point, to maintain sharp features.
class VertexNormalOp {
 public:
  VertexNormalOp(OpenVDBLevelSet &mLvl, float angle = 0.7f);
  void operator()(const RangeT &);

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<openvdb::Vec3s> mNormalHandle;
  const float mAngle;
};

////////////////////////////////////////

VertexNormalOp::VertexNormalOp(OpenVDBLevelSet &lvl, float angle) : mLvl(lvl), mAngle(angle)
{
  mNormalHandle = std::vector<openvdb::Vec3s>(lvl.get_triangles().size());
}

void VertexNormalOp::operator()(const RangeT &r)
{
  std::vector<openvdb::Vec3I> tris = mLvl.get_triangles();
  std::vector<openvdb::Vec3s> verts = mLvl.get_points();
  std::vector<openvdb::Vec3s> vnorms = mLvl.get_vert_normals();
  openvdb::Vec3s tmpN, avgN;
  openvdb::Vec3I tri;
  int i;

  for (i = r.begin(); i < r.end(); i++) {
    tri = tris[i];
    avgN = vnorms[tri[0]];
    for (int vtx = 0; vtx < 2; vtx++) {
      tmpN = vnorms[tri[vtx]];
      if (tmpN.dot(avgN) > mAngle)
        avgN += tmpN;
    }
    avgN.normalize();
    mNormalHandle[i] = avgN;
  }
}
#endif
/*class BuildQuadMapOp {
 public:
  BuildQuadMapOp(OpenVDBLevelSet &lvl, std::vector<std::vector<openvdb::Index32>> &quad_map);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<std::vector<openvdb::Index32>> *const mQuadMap;
};

BuildQuadMapOp::BuildQuadMapOp(OpenVDBLevelSet &lvl,
                               std::vector<std::vector<openvdb::Index32>> &point_quad_map)
    : mLvl(lvl), mQuadMap(&point_quad_map)
{
}

void BuildQuadMapOp::operator()(const RangeT &r) const
{
  std::vector<openvdb::Vec4I> quads_in(mLvl.get_out_quads());

  for (int i = r.begin(); i < r.end(); i++) {
    for (int v = 0; v < 4; v++) {
      (*mQuadMap)[quads_in[i][v]].push_back(i);
    }
  }
} */

class BuildQuadMapOp {
 public:
  BuildQuadMapOp(OpenVDBLevelSet &lvl, std::vector<std::vector<openvdb::Index32>> &quad_map);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<std::vector<openvdb::Index32>> *const mQuadMap;
};

BuildQuadMapOp::BuildQuadMapOp(OpenVDBLevelSet &lvl,
                               std::vector<std::vector<openvdb::Index32>> &point_quad_map)
    : mLvl(lvl), mQuadMap(&point_quad_map)
{
}

void BuildQuadMapOp::operator()(const RangeT &r) const
{
  std::vector<openvdb::Vec4I> quads_in(mLvl.get_out_quads());

  for (int i = r.begin(); i < r.end(); i++) {
    for (int v = 0; v < 4; v++) {
      (*mQuadMap)[quads_in[i][v]].push_back(i);
    }
  }
}
class SmoothOp {
 public:
  SmoothOp(OpenVDBLevelSet &lvl,
           std::vector<openvdb::Vec3s> &vertsIn,
           std::vector<openvdb::Vec4I> &quadsIn,
           std::vector<openvdb::Index32> &offset_map,
           std::vector<std::vector<openvdb::Index32>> &quad_map,
           std::vector<openvdb::Index32> &quad_offset_map,
           std::vector<openvdb::Vec3s> &verts,
           std::vector<openvdb::Vec4I> &quads);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<openvdb::Vec3s> mVertsIn;
  std::vector<openvdb::Vec4I> mQuadsIn;
  std::vector<openvdb::Index32> mOffsetMap;
  std::vector<openvdb::Vec3s> *const mVerts;
  std::vector<openvdb::Vec4I> *const mQuads;
  std::vector<std::vector<openvdb::Index32>> mQuadMap;
  std::vector<openvdb::Index32> mQuadOffsetMap;
};

SmoothOp::SmoothOp(OpenVDBLevelSet &lvl,
                   std::vector<openvdb::Vec3s> &vertsIn,
                   std::vector<openvdb::Vec4I> &quadsIn,
                   std::vector<openvdb::Index32> &offset_map,
                   std::vector<std::vector<openvdb::Index32>> &quad_map,
                   std::vector<openvdb::Index32> &quad_offset_map,
                   std::vector<openvdb::Vec3s> &verts,
                   std::vector<openvdb::Vec4I> &quads)
    : mLvl(lvl),
      mVerts(&verts),
      mQuads(&quads),
      mVertsIn(vertsIn),
      mQuadsIn(quadsIn),
      mOffsetMap(offset_map),
      mQuadOffsetMap(quad_offset_map),
      mQuadMap(quad_map)
{
}

bool validQuad(openvdb::Vec4I quad)
{
  return !((quad[0] == openvdb::util::INVALID_IDX) || (quad[1] == openvdb::util::INVALID_IDX) ||
           (quad[2] == openvdb::util::INVALID_IDX) || (quad[3] == openvdb::util::INVALID_IDX));
}

bool validVert(openvdb::Vec3s vert)
{
  return !(std::isnan(vert[0]) || std::isnan(vert[1]) || std::isnan(vert[2]));
}

void SmoothOp::operator()(const RangeT &r) const
{
  int j = 0;
  int k = 0;
  for (int i = r.begin(); i < r.end(); i++) {
    if (validQuad(mQuadsIn[i])) {
      bool ok = true;
      for (int v = 0; v < 4; v++) {
        ok = ok && validVert(mVertsIn[mQuadsIn[i][v]]);
      }

      if (!ok) {
        continue;
      }

      (*mQuads)[j] = mQuadsIn[i];
      for (int v = 0; v < 4; v++) {
        int o = (*mQuads)[j][v];
        int off = o;
        if (i > 0) {
          off = o - mOffsetMap[i];
        }
        if (off < 0) {
          off = o;
        }

        (*mVerts)[off][0] = mVertsIn[mQuadsIn[i][v]][0];
        (*mVerts)[off][1] = mVertsIn[mQuadsIn[i][v]][1];
        (*mVerts)[off][2] = mVertsIn[mQuadsIn[i][v]][2];

        // do smoothing along "edge lengths" (across different quads), TODO
        // find quads "around" this vertex
        openvdb::Vec3s co = (*mVerts)[off];
        int numV = 1;
        // int k = i - o;

        int quads = mQuadMap[i].size();
        for (int q = 0; q < quads; q++) {
          unsigned int quad_index = mQuadMap[i][q];
          // int offset = mOffsetMap[i];
          if (quad_index != i) {
            if (validQuad(mQuadsIn[quad_index])) {
              openvdb::Vec4I quad = mQuadsIn[quad_index];
              int index = v + 1;
              // which verts share an "edge" ?
              // like...1-2 2-3 3-4 4-1
              // me = v, so vert[v+1]
              if (index == 4) {
                index = 0;
              }
              openvdb::Vec3s vert = mVertsIn[quad[index]];
              if (validVert(vert)) {
                co[0] += vert[0];
                co[1] += vert[1];
                co[2] += vert[2];
                numV++;
              }
            }
          }
        }

        co[0] /= numV;
        co[1] /= numV;
        co[2] /= numV;

        // (*mVerts)[off] = co;
      }
      j++;
    }
  }
}

class FixPolesOp {
 public:
  FixPolesOp(OpenVDBLevelSet &lvl,
             std::vector<openvdb::Vec3s> &out_points,
             std::vector<std::vector<openvdb::Index32>> &quad_map,
             std::vector<openvdb::Vec4I> &out_quads,
             std::vector<openvdb::Index32> &offset_map,
             std::vector<openvdb::Index32> &quad_offset_map,
             int &removedVerts,
             int &removedQuads);
  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mLvl;
  std::vector<openvdb::Vec3s> *const mPointList;
  std::vector<std::vector<openvdb::Index32>> *const mQuadMap;
  std::vector<openvdb::Vec4I> *const mQuadList;
  std::vector<openvdb::Index32> *const mOffsetMap;
  std::vector<openvdb::Index32> *const mQuadOffsetMap;
  int *mRemovedVerts;
  int *mRemovedQuads;
};

FixPolesOp::FixPolesOp(OpenVDBLevelSet &lvl,
                       std::vector<openvdb::Vec3s> &out_points,  // empty vector, correct size
                       std::vector<std::vector<openvdb::Index32>> &point_quad_map,
                       std::vector<openvdb::Vec4I> &out_quads,  // empty vector, correct size
                       std::vector<openvdb::Index32> &offset_map,
                       std::vector<openvdb::Index32> &quad_offset_map,
                       int &removedVerts,
                       int &removedQuads)
    : mLvl(lvl),
      mPointList(&out_points),
      mQuadMap(&point_quad_map),
      mQuadList(&out_quads),
      mOffsetMap(&offset_map),
      mQuadOffsetMap(&quad_offset_map),
      mRemovedVerts(&removedVerts),
      mRemovedQuads(&removedQuads)
{
}

void FixPolesOp::operator()(const RangeT &r) const
{
  std::vector<openvdb::Vec3s> points_in(mLvl.get_out_points());
  std::vector<openvdb::Vec4I> quads_in(mLvl.get_out_quads());
  std::vector<bool> visited(quads_in.size(), false);

  for (int i = r.begin(); i < r.end(); i++) {
    // in how many quads do we find i ?, we dont touch the triangles (adaptivity)
    int quads = (*mQuadMap)[i].size();

    for (unsigned int q = 0; q < quads; q++) {
      bool is_diamond_quad = true;
      unsigned int quad_index = (*mQuadMap)[i][q];
      if (visited[quad_index]) {
        continue;
      }
      else {
        visited[quad_index] = true;
      }

      openvdb::Vec4I &quad = quads_in[quad_index];
      // go around the verts of those quads
      for (unsigned int v = 0; v < 4; v++) {
        unsigned int pole = (*mQuadMap)[quad[v]].size();
        // next quad must be 5pole or more
        if ((v == 1 || v == 3) && (pole < 5)) {
          is_diamond_quad = false;
          break;
        }
        if ((v == 0 || v == 2) && (pole != 3)) {
          is_diamond_quad = false;
          break;
        }
      }

      // assign everything first...
      (*mQuadList)[quad_index] = quad;
      for (int v = 0; v < 4; v++) {
        (*mPointList)[quad[v]] = points_in[quad[v]];
      }

      // then check for removal
      if (is_diamond_quad) {
        // re-assign index 2 to 0 in affected quads
        int quads = (*mQuadMap)[quad[2]].size();
        for (int q = 0; q < quads; q++) {
          int q_index = (*mQuadMap)[quad[2]][q];
          openvdb::Vec4I qu = quads_in[q_index];
          for (int v = 0; v < 4; v++) {
            if (qu[v] == quad[2]) {
              (*mQuadList)[q_index][v] = quad[0];
            }
          }
        }

        // remove vertex with index 2, the 2nd 3-pole, keep 1,3, set 0 to center between 0 and
        // 2
        openvdb::Vec3s center = (points_in[quad[0]] + points_in[quad[2]]) * 0.5f;
        (*mPointList)[quad[2]] = openvdb::Vec3s(NAN, NAN, NAN);
        (*mPointList)[quad[0]] = center;
        (*mPointList)[quad[1]] = points_in[quad[1]];
        (*mPointList)[quad[3]] = points_in[quad[3]];

        // remove this quad from list, as in dont transfer it here
        const openvdb::Index32 ix = openvdb::util::INVALID_IDX;
        (*mQuadList)[quad_index] = openvdb::Vec4I(ix, ix, ix, ix);
        //(*mRemovedQuads)++;
        //(*mRemovedVerts)++;
      }

      bool valid = true;
      openvdb::Vec4I &qud = quads_in[quad_index];
      for (int v = 0; v < 4; v++) {
        if (!validVert((*mPointList)[qud[v]])) {
          (*mRemovedVerts)++;
          valid = false;
        }
      }
      if (!valid) {
        (*mRemovedQuads)++;
      }

      (*mOffsetMap)[quad_index] = *mRemovedQuads;
    }
  }
}

using RangeT = tbb::blocked_range<size_t>;
/// TBB body object for threaded sharp feature construction
class SharpenFeaturesOp {
 public:
  using EdgeData = openvdb::tools::MeshToVoxelEdgeData;

  SharpenFeaturesOp(OpenVDBLevelSet &refGeo,
                    EdgeData &edgeData,
                    const openvdb::math::Transform &xform,
                    const openvdb::BoolTree *mask,
                    std::vector<openvdb::Vec3s> &pointList);

  void operator()(const RangeT &) const;

 private:
  OpenVDBLevelSet &mRefGeo;
  EdgeData &mEdgeData;
  const openvdb::math::Transform &mXForm;
  const openvdb::BoolTree *mMaskTree;
  std::vector<openvdb::Vec3s> *const mPointList;
};

////////////////////////////////////////

SharpenFeaturesOp::SharpenFeaturesOp(OpenVDBLevelSet &refGeo,
                                     EdgeData &edgeData,
                                     const openvdb::math::Transform &xform,
                                     const openvdb::BoolTree *mask,
                                     std::vector<openvdb::Vec3s> &pointList)
    : mRefGeo(refGeo), mEdgeData(edgeData), mXForm(xform), mMaskTree(mask), mPointList(&pointList)
{
}

void SharpenFeaturesOp::operator()(const RangeT &r) const
{
  int i;
  openvdb::tools::MeshToVoxelEdgeData::Accessor acc = mEdgeData.getAccessor();
  std::vector<openvdb::Vec3s> input(mRefGeo.get_out_points());

  using BoolAccessor = openvdb::tree::ValueAccessor<const openvdb::BoolTree>;
  std::unique_ptr<BoolAccessor> maskAcc;

  if (mMaskTree) {
    maskAcc.reset(new BoolAccessor(*mMaskTree));
  }

  openvdb::Vec3s avgP;
  openvdb::BBoxd cell;

  openvdb::Vec3d pos, normal;
  openvdb::Coord ijk;

  std::vector<openvdb::Vec3d> points(12), normals(12);
  std::vector<openvdb::Index32> primitives(12);

  for (i = r.begin(); i < r.end(); i++) {
    pos = openvdb::Vec3s(input[i][0], input[i][1], input[i][2]);
    pos = mXForm.worldToIndex(pos);

    ijk[0] = int(std::floor(pos[0]));
    ijk[1] = int(std::floor(pos[1]));
    ijk[2] = int(std::floor(pos[2]));

    if (maskAcc && !maskAcc->isValueOn(ijk)) {
      pos = mXForm.indexToWorld(pos);
      (*mPointList)[i] = pos;
      continue;
    }

    points.clear();
    normals.clear();
    primitives.clear();

    // get voxel-edge intersections
    mEdgeData.getEdgeData(acc, ijk, points, primitives);

    avgP = openvdb::Vec3s(0.0, 0.0, 0.0);

    // get normal list
    for (size_t n = 0, N = points.size(); n < N; ++n) {

      avgP += points[n];
      normal = mRefGeo.face_normal(primitives[n] * 4);
      normals.push_back(normal);
    }

    // Calculate feature point position
    if (points.size() > 1) {

      pos = openvdb::tools::findFeaturePoint(points, normals);

      // Constrain points to stay inside their initial
      // coordinate cell.
      cell = openvdb::BBoxd(
          openvdb::Vec3d(double(ijk[0]), double(ijk[1]), double(ijk[2])),
          openvdb::Vec3d(double(ijk[0] + 1), double(ijk[1] + 1), double(ijk[2] + 1)));

      // cell.expand(openvdb::Vec3d(0.3, 0.3, 0.3));
      cell.expand(0.6);

      if (!cell.isInside(openvdb::Vec3d(pos[0], pos[1], pos[2]))) {

        openvdb::Vec3s org(pos[0], pos[1], pos[2]);

        avgP *= 1.f / float(points.size());
        openvdb::Vec3s dir = avgP - org;
        dir.normalize();

        // double distance;
        // if (cell.intersectRay(org, dir, 1E17F, &distance) > 0)
        float distance;
        Ray ray(org, dir);
        AABBox box(cell.min(), cell.max());
        if (box.intersect(ray, distance)) {
          pos = org + dir * distance;
        }
      }
    }

    pos = mXForm.indexToWorld(pos);
    (*mPointList)[i] = pos;
  }
}
}  // namespace tools
}  // namespace OPENVDB_VERSION_NAME
}  // namespace openvdb

#endif /* __OPENVDB_MESHER_H__ */

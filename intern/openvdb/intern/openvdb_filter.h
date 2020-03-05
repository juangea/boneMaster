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

#ifndef __OPENVDB_FILTER_H__
#define __OPENVDB_FILTER_H__

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetFilter.h>
#include <cmath>
#include "openvdb_capi.h"

namespace openvdb {

OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {

#define PI 3.141592653589793f
#define EE 2.718281828459045f
#define PISQRT 2.5066282746310002f

template<typename GridT,
         typename MaskT = typename GridT::template ValueConverter<float>::Type,
         typename InterruptT = util::NullInterrupter>
class ExtendedLevelSetFilter : public LevelSetFilter<GridT, MaskT, InterruptT> {

 public:
  using BaseType = LevelSetFilter<GridT, MaskT, InterruptT>;
  using GridType = GridT;
  using MaskType = MaskT;
  using TreeType = typename GridType::TreeType;
  using LeafType = typename TreeType::LeafNodeType;
  using ValueType = typename TreeType::ValueType;
  using ValueType2 = typename GridType::ValueType;
  using AlphaType = typename MaskType::ValueType;
  using LeafManagerType = typename tree::LeafManager<TreeType>;
  using RangeType = typename LeafManagerType::LeafRange;
  using BufferType = typename LeafManagerType::BufferType;

  ExtendedLevelSetFilter(GridType &grid, InterruptT *interrupt = nullptr)
      : BaseType(grid, interrupt)
  {
  }

  void gaussianI(int width = 1,
                 int iterations = 1,
                 float sigma = 1.0,
                 const MaskType *mask = nullptr)
  {
    ExtendedFilter f(this, mask);
    f.gaussian(width, iterations, sigma);
  }

  // struct that implements all the filtering.
  // why the fuck was this private in Levelset filter... gah...
  // in order not to modify openvdb, i have to copy this now here
  struct ExtendedFilter {
    using LeafT = typename TreeType::LeafNodeType;
    using VoxelIterT = typename LeafT::ValueOnIter;
    using VoxelCIterT = typename LeafT::ValueOnCIter;
    using BufferT = typename tree::LeafManager<TreeType>::BufferType;
    using LeafRange = typename tree::LeafManager<TreeType>::LeafRange;
    using LeafIterT = typename LeafRange::Iterator;
    using AlphaMaskT = tools::AlphaMask<GridT, MaskT>;

    ExtendedFilter(ExtendedLevelSetFilter *parent, const MaskType *mask)
        : mParent(parent), mMask(mask)
    {
    }
    ExtendedFilter(const ExtendedFilter &) = default;
    virtual ~ExtendedFilter()
    {
    }

    void operator()(const LeafRange &r) const
    {
      if (mTask)
        mTask(const_cast<ExtendedFilter *>(this), r);
      else
        OPENVDB_THROW(ValueError, "task is undefined - don\'t call this method directly");
    }
    void cook(bool swap)
    {
      const int n = mParent->getGrainSize();
      if (n > 0) {
        tbb::parallel_for(mParent->leafs().leafRange(n), *this);
      }
      else {
        (*this)(mParent->leafs().leafRange());
      }
      if (swap)
        mParent->leafs().swapLeafBuffer(1, n == 0);
    }

    /// @brief One iteration of a fast separable Gaussian filter.
    ///
    /// @param width The width of the mean-value filter is 2*width+1 voxels.
    /// @param iterations Number of times the mean-value filter is applied.
    /// @param sigma of the Gaussian distribution
    /// @param mask Optional alpha mask.
    void gaussian(int width, int iterations, float sigma);
    void box(int width, float sigma);

    template<size_t Axis> struct Avg {
      Avg(const GridT &grid, Int32 w, float s) : acc(grid.tree()), width(w), sigma(s)
      {
      }
      inline ValueType operator()(Coord xyz)
      {
#if 0
        ValueType sum = zeroVal<ValueType>();
        Int32 &i = xyz[Axis], j = i + width;
        for (i -= width; i <= j; ++i)
          sum += acc.getValue(xyz);
        return sum * frac;
#endif
        /// taken from modified Gaussian function of Openvdb remesher addon

        // Sampled Gaussian kernel
        ValueType sum = zeroVal<ValueType>();
        float multiplier = 0.f, error = 0.f;

        // float sigma = std::sqrt((float)width / 4.0f);
        // M = C*sqrt(t) + 1
        // int M = (int)(5.0f * sigma + 1.0f);
        int M = width;
        Int32 &i = xyz[Axis], j = i + M;
        Int32 center = i;

        for (i -= M; i <= j; ++i) {
          // G(x) = 1/sqrt(2*PI)/sigma * e ** -(x**2/(2*sigma**2))
          Int32 x = i - center;
          multiplier = 1.f / PISQRT / sigma * std::pow(EE, -x * x / (2 * sigma * sigma));
          error += multiplier;
          sum = sum + (multiplier * acc.getValue(xyz));
        }

        return static_cast<ValueType>(sum / error);
        // return static_cast<ValueType>(sum);
      }
      typename GridT::ConstAccessor acc;
      const Int32 width;
      const Real sigma;
    };

    template<typename AvgT> void boxImpl(const LeafRange &r, Int32 w, float q);

    void boxXImpl(const LeafRange &r, Int32 w, float q)
    {
      this->boxImpl<Avg<0>>(r, w, q);
    }
    void boxZImpl(const LeafRange &r, Int32 w, float q)
    {
      this->boxImpl<Avg<1>>(r, w, q);
    }
    void boxYImpl(const LeafRange &r, Int32 w, float q)
    {
      this->boxImpl<Avg<2>>(r, w, q);
    }

    LevelSetFilter<GridT, MaskT, InterruptT> *mParent;
    const MaskType *mMask;
    typename std::function<void(ExtendedFilter *, const LeafRange &)> mTask;
  };  // end of ExtendedFilter struct
};

template<typename GridT, typename MaskT, typename InterruptT>
inline void ExtendedLevelSetFilter<GridT, MaskT, InterruptT>::ExtendedFilter::gaussian(
    int width, int iterations, float sigma)
{
  mParent->startInterrupter("Gaussian flow of level set");

  for (int i = 0; i < iterations; i++) {
    for (int n = 0; n < 4; ++n) {
      this->box(width, sigma);
    }
  }
  mParent->endInterrupter();
}

template<typename GridT, typename MaskT, typename InterruptT>
inline void ExtendedLevelSetFilter<GridT, MaskT, InterruptT>::ExtendedFilter::box(int width,
                                                                                  float sigma)
{
  mParent->leafs().rebuildAuxBuffers(1, mParent->getGrainSize() == 0);

  width = std::max(1, width);

  mTask = std::bind(
      &ExtendedFilter::boxXImpl, std::placeholders::_1, std::placeholders::_2, width, sigma);
  this->cook(true);

  mTask = std::bind(
      &ExtendedFilter::boxYImpl, std::placeholders::_1, std::placeholders::_2, width, sigma);
  this->cook(true);

  mTask = std::bind(
      &ExtendedFilter::boxZImpl, std::placeholders::_1, std::placeholders::_2, width, sigma);
  this->cook(true);

  mParent->track();
}

template<typename GridT, typename MaskT, typename InterruptT>
template<typename AvgT>
inline void ExtendedLevelSetFilter<GridT, MaskT, InterruptT>::ExtendedFilter::boxImpl(
    const LeafRange &range, Int32 w, float sigma)
{
  mParent->checkInterrupter();
  AvgT avg(mParent->grid(), w, sigma);
  if (mMask) {
    typename AlphaMaskT::FloatType a, b;
    AlphaMaskT alpha(mParent->grid(),
                     *mMask,
                     mParent->minMask(),
                     mParent->maxMask(),
                     mParent->isMaskInverted());
    for (LeafIterT leafIter = range.begin(); leafIter; ++leafIter) {
      ValueType *buffer = leafIter.buffer(1).data();
      for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
        const Coord xyz = iter.getCoord();
        if (alpha(xyz, a, b))
          buffer[iter.pos()] = b * (*iter) + a * avg(xyz);
      }
    }
  }
  else {
    for (LeafIterT leafIter = range.begin(); leafIter; ++leafIter) {
      ValueType *buffer = leafIter.buffer(1).data();
      for (VoxelCIterT iter = leafIter->cbeginValueOn(); iter; ++iter) {
        buffer[iter.pos()] = avg(iter.getCoord());
      }
    }
  }
}

}  // namespace tools
}  // namespace OPENVDB_VERSION_NAME
}  // namespace openvdb

#endif /* __OPENVDB_LEVEL_SET_H__ */

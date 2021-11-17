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
 * Adapted from the Blender Alembic importer implementation.
 *
 * Modifications Copyright (C) 2021 Tangent Animation.
 * All rights reserved.
 */

#include "usd_reader_nurbs.h"

#include "BKE_curve.h"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_spline.hh"

#include "BLI_listbase.h"

#include "DNA_curve_types.h"
#include "DNA_object_types.h"

#include "MEM_guardedalloc.h"

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/types.h>

#include <pxr/usd/usdGeom/curves.h>

static bool set_knots(const pxr::VtDoubleArray &knots, float *&nu_knots)
{
  if (knots.empty()) {
    return false;
  }

  /* Skip first and last knots, as they are used for padding. */
  const size_t num_knots = knots.size();
  nu_knots = static_cast<float *>(MEM_callocN(num_knots * sizeof(float), __func__));

  for (size_t i = 0; i < num_knots; i++) {
    nu_knots[i] = (float)knots[i];
  }

  return true;
}

namespace blender::io::usd {

void USDNurbsReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  curve_ = BKE_curve_add(bmain, name_.c_str(), OB_CURVE);

  curve_->flag |= CU_3D;
  curve_->actvert = CU_ACT_NONE;
  curve_->resolu = 2;

  object_ = BKE_object_add_only_object(bmain, OB_CURVE, name_.c_str());
  object_->data = curve_;
}

void USDNurbsReader::read_object_data(Main *bmain, const double motionSampleTime)
{
  Curve *cu = (Curve *)object_->data;
  read_curve_sample(cu, motionSampleTime);

  if (curve_prim_.GetPointsAttr().ValueMightBeTimeVarying()) {
    add_cache_modifier();
  }

  USDXformReader::read_object_data(bmain, motionSampleTime);
}

void USDNurbsReader::read_curve_sample(Curve *cu, const double motionSampleTime)
{
  curve_prim_ = pxr::UsdGeomNurbsCurves(prim_);

  pxr::UsdAttribute widthsAttr = curve_prim_.GetWidthsAttr();
  pxr::UsdAttribute vertexAttr = curve_prim_.GetCurveVertexCountsAttr();
  pxr::UsdAttribute pointsAttr = curve_prim_.GetPointsAttr();

  pxr::VtIntArray usdCounts;
  vertexAttr.Get(&usdCounts, motionSampleTime);

  pxr::VtVec3fArray usdPoints;
  pointsAttr.Get(&usdPoints, motionSampleTime);

  pxr::VtFloatArray usdWidths;
  widthsAttr.Get(&usdWidths, motionSampleTime);

  pxr::VtIntArray orders;
  curve_prim_.GetOrderAttr().Get(&orders, motionSampleTime);

  pxr::VtDoubleArray knots;
  curve_prim_.GetKnotsAttr().Get(&knots, motionSampleTime);

  pxr::VtVec3fArray usdNormals;
  curve_prim_.GetNormalsAttr().Get(&usdNormals, motionSampleTime);

  /* If normals, extrude, else bevel.
   * Perhaps to be replaced by Blender USD Schema. */
  if (!usdNormals.empty()) {
    /* Set extrusion to 1. */
    curve_->ext1 = 1.0f;
  }
  else {
    /* Set bevel depth to 1. */
    curve_->ext2 = 1.0f;
  }

  size_t idx = 0;
  for (size_t i = 0; i < usdCounts.size(); i++) {
    const int num_verts = usdCounts[i];

    Nurb *nu = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), __func__));
    nu->flag = CU_SMOOTH;
    nu->type = CU_NURBS;

    nu->resolu = cu->resolu;
    nu->resolv = cu->resolv;

    nu->pntsu = num_verts;
    nu->pntsv = 1;

    if (i < orders.size()) {
      nu->orderu = static_cast<short>(orders[i]);
    }
    else {
      nu->orderu = 4;
      nu->orderv = 4;
    }

    /* TODO(makowalski): investigate setting Cyclic U and Endpoint U options. */
#if 0
     if (knots.size() > 3) {
       if ((knots[0] == knots[1]) && (knots[knots.size()] == knots[knots.size() - 1])) {
         nu->flagu |= CU_NURB_ENDPOINT;
       } else {
         nu->flagu |= CU_NURB_CYCLIC;
       }
     }
#endif

    float weight = 1.0f;

    nu->bp = static_cast<BPoint *>(MEM_callocN(sizeof(BPoint) * nu->pntsu, __func__));
    BPoint *bp = nu->bp;

    for (int j = 0; j < nu->pntsu; j++, bp++, idx++) {
      bp->vec[0] = (float)usdPoints[idx][0];
      bp->vec[1] = (float)usdPoints[idx][1];
      bp->vec[2] = (float)usdPoints[idx][2];
      bp->vec[3] = weight;
      bp->f1 = SELECT;
      bp->weight = weight;

      float radius = 0.1f;
      if (idx < usdWidths.size()) {
        radius = usdWidths[idx];
      }

      bp->radius = radius;
    }

    if (!set_knots(knots, nu->knotsu)) {
      BKE_nurb_knot_calc_u(nu);
    }

    BLI_addtail(BKE_curve_nurbs_get(cu), nu);
  }
}

static bool topology_changed(CurveEval *curve_eval, const pxr::VtIntArray &usdCounts)
{
  if (!curve_eval) {
    return true;
  }

  if (curve_eval->splines().size() != usdCounts.size()) {
    return true;
  }

  int curve_idx = 0;
  for (const SplinePtr &spline : curve_eval->splines()) {
    const int num_in_usd = usdCounts[curve_idx++];
    const int num_in_blender = spline->positions().size();

    if (num_in_usd != num_in_blender) {
      return true;
    }
  }

  return false;
}

void USDNurbsReader::read_geometry(GeometrySet &geometry_set,
                                   double motionSampleTime,
                                   int /* read_flag */,
                                   const char ** /* err_str */)
{
  pxr::UsdGeomCurves curve_prim_(prim_);

  pxr::UsdAttribute widthsAttr = curve_prim_.GetWidthsAttr();
  pxr::UsdAttribute vertexAttr = curve_prim_.GetCurveVertexCountsAttr();
  pxr::UsdAttribute pointsAttr = curve_prim_.GetPointsAttr();

  pxr::VtIntArray usdCounts;

  vertexAttr.Get(&usdCounts, motionSampleTime);

  pxr::VtVec3fArray usdPoints;
  pointsAttr.Get(&usdPoints, motionSampleTime);

  int vertex_idx = 0;
  int curve_idx;
  CurveEval *curve_eval = geometry_set.get_curve_for_write();

  if (blender::io::usd::topology_changed(curve_eval, usdCounts)) {
    Curve *curve = static_cast<Curve *>(object_->data);
    BKE_nurbList_free(&curve->nurb);
    read_curve_sample(curve, motionSampleTime);

    std::unique_ptr<CurveEval> new_curve_eval = curve_eval_from_dna_curve(*curve);
    geometry_set.replace_curve(new_curve_eval.release(), GeometryOwnershipType::Editable);
  }
  else {
    BLI_assert_msg(curve_eval, "curve_eval is null although the topology is the same !");
    const int curve_count = curve_eval->splines().size();
    for (curve_idx = 0; curve_idx < curve_count; curve_idx++) {
      Spline *spline = curve_eval->splines()[curve_idx].get();

      for (float3 &position : spline->positions()) {
        position.x = usdPoints[vertex_idx][0];
        position.y = usdPoints[vertex_idx][0];
        position.z = usdPoints[vertex_idx][0];
        vertex_idx++;
      }
    }
  }
}

}  // namespace blender::io::usd

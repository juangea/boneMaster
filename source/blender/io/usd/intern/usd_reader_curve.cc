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
 * Adapted from the Blender Alembic importer implementation,
 * Copyright (C) 2016 Kévin Dietrich.
 *
 * Modifications Copyright (C) 2021 Tangent Animation.
 * All rights reserved.
 */

#include "usd_reader_curve.h"

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

#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/curves.h>

namespace blender::io::usd {

void USDCurvesReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  curve_ = BKE_curve_add(bmain, name_.c_str(), OB_CURVE);

  curve_->flag |= CU_3D;
  curve_->actvert = CU_ACT_NONE;
  curve_->resolu = 2;

  object_ = BKE_object_add_only_object(bmain, OB_CURVE, name_.c_str());
  object_->data = curve_;
}

void USDCurvesReader::read_object_data(Main *bmain, double motionSampleTime)
{
  Curve *cu = (Curve *)object_->data;
  read_curve_sample(cu, motionSampleTime);

  if (curve_prim_.GetPointsAttr().ValueMightBeTimeVarying()) {
    add_cache_modifier();
  }

  USDXformReader::read_object_data(bmain, motionSampleTime);
}

void USDCurvesReader::read_curve_sample(Curve *cu, const double motionSampleTime)
{
  curve_prim_ = pxr::UsdGeomBasisCurves(prim_);

  if (!curve_prim_) {
    return;
  }

  pxr::UsdAttribute widthsAttr = curve_prim_.GetWidthsAttr();
  pxr::UsdAttribute vertexAttr = curve_prim_.GetCurveVertexCountsAttr();
  pxr::UsdAttribute pointsAttr = curve_prim_.GetPointsAttr();

  pxr::VtIntArray usdCounts;

  vertexAttr.Get(&usdCounts, motionSampleTime);
  int num_subcurves = usdCounts.size();

  pxr::VtVec3fArray usdPoints;
  pointsAttr.Get(&usdPoints, motionSampleTime);

  pxr::VtFloatArray usdWidths;
  widthsAttr.Get(&usdWidths, motionSampleTime);

  pxr::UsdAttribute basisAttr = curve_prim_.GetBasisAttr();
  pxr::TfToken basis;
  basisAttr.Get(&basis, motionSampleTime);

  pxr::UsdAttribute typeAttr = curve_prim_.GetTypeAttr();
  pxr::TfToken type;
  typeAttr.Get(&type, motionSampleTime);

  pxr::UsdAttribute wrapAttr = curve_prim_.GetWrapAttr();
  pxr::TfToken wrap;
  wrapAttr.Get(&wrap, motionSampleTime);

  pxr::VtVec3fArray usdNormals;
  curve_prim_.GetNormalsAttr().Get(&usdNormals, motionSampleTime);

  /* If normals, extrude, else bevel.
   * Perhaps to be replaced by Blender/USD Schema. */
  if (!usdNormals.empty()) {
    /* Set extrusion to 1.0f. */
    curve_->ext1 = 1.0f;
  }
  else {
    /* Set bevel depth to 1.0f. */
    curve_->ext2 = 1.0f;
  }

  size_t idx = 0;
  for (size_t i = 0; i < num_subcurves; i++) {
    const int num_verts = usdCounts[i];
    Nurb *nu = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), __func__));

    if (basis == pxr::UsdGeomTokens->bspline) {
      nu->flag = CU_SMOOTH;
      nu->type = CU_NURBS;
    }
    else if (basis == pxr::UsdGeomTokens->bezier) {
      /* TODO(makowalski): Beziers are not properly imported as beziers. */
      nu->type = CU_POLY;
    }
    else if (basis.IsEmpty()) {
      nu->type = CU_POLY;
    }
    nu->resolu = cu->resolu;
    nu->resolv = cu->resolv;

    nu->pntsu = num_verts;
    nu->pntsv = 1;

    if (type == pxr::UsdGeomTokens->cubic) {
      nu->orderu = 4;
    }
    else if (type == pxr::UsdGeomTokens->linear) {
      nu->orderu = 2;
    }

    if (wrap == pxr::UsdGeomTokens->periodic) {
      nu->flagu |= CU_NURB_CYCLIC;
    }
    else if (wrap == pxr::UsdGeomTokens->pinned) {
      nu->flagu |= CU_NURB_ENDPOINT;
    }

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

      float radius = curve_->width;
      if (idx < usdWidths.size()) {
        radius = usdWidths[idx];
      }

      bp->radius = radius;
    }

    BKE_nurb_knot_calc_u(nu);
    BKE_nurb_knot_calc_v(nu);

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

void USDCurvesReader::read_geometry(GeometrySet &geometry_set,
                                    double motionSampleTime,
                                    int /* read_flag */,
                                    const char ** /* err_str */)
{
  if (!curve_prim_) {
    return;
  }

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

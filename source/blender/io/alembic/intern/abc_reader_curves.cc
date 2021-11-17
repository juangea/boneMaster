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
 * The Original Code is Copyright (C) 2016 Kévin Dietrich.
 * All rights reserved.
 */

/** \file
 * \ingroup balembic
 */

#include "abc_reader_curves.h"
#include "abc_axis_conversion.h"
#include "abc_reader_transform.h"
#include "abc_util.h"

#include <cstdio>

#include "MEM_guardedalloc.h"

#include "DNA_curve_types.h"
#include "DNA_object_types.h"

#include "BLI_listbase.h"

#include "BKE_curve.h"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_spline.hh"

using Alembic::Abc::FloatArraySamplePtr;
using Alembic::Abc::Int32ArraySamplePtr;
using Alembic::Abc::P3fArraySamplePtr;
using Alembic::Abc::PropertyHeader;
using Alembic::Abc::UcharArraySamplePtr;

using Alembic::AbcGeom::CurvePeriodicity;
using Alembic::AbcGeom::ICompoundProperty;
using Alembic::AbcGeom::ICurves;
using Alembic::AbcGeom::ICurvesSchema;
using Alembic::AbcGeom::IFloatGeomParam;
using Alembic::AbcGeom::IInt16Property;
using Alembic::AbcGeom::ISampleSelector;
using Alembic::AbcGeom::kWrapExisting;

namespace blender::io::alembic {

AbcCurveReader::AbcCurveReader(const Alembic::Abc::IObject &object, ImportSettings &settings)
    : AbcObjectReader(object, settings)
{
  ICurves abc_curves(object, kWrapExisting);
  m_curves_schema = abc_curves.getSchema();

  get_min_max_time(m_iobject, m_curves_schema, m_min_time, m_max_time);
}

bool AbcCurveReader::valid() const
{
  return m_curves_schema.valid();
}

bool AbcCurveReader::accepts_object_type(
    const Alembic::AbcCoreAbstract::ObjectHeader &alembic_header,
    const Object *const ob,
    const char **err_str) const
{
  if (!Alembic::AbcGeom::ICurves::matches(alembic_header)) {
    *err_str =
        "Object type mismatch, Alembic object path pointed to Curves when importing, but not any "
        "more.";
    return false;
  }

  if (ob->type != OB_CURVE) {
    *err_str = "Object type mismatch, Alembic object path points to Curves.";
    return false;
  }

  return true;
}

static short get_curve_resolution(const ICurvesSchema &schema,
                                  const Alembic::Abc::ISampleSelector &sample_sel)
{
  ICompoundProperty user_props = schema.getUserProperties();
  if (user_props) {
    const PropertyHeader *header = user_props.getPropertyHeader(ABC_CURVE_RESOLUTION_U_PROPNAME);
    if (header != nullptr && header->isScalar() && IInt16Property::matches(*header)) {
      IInt16Property resolu(user_props, header->getName());
      return resolu.getValue(sample_sel);
    }
  }

  return 1;
}

static short get_curve_order(Alembic::AbcGeom::CurveType abc_curve_type,
                             const UcharArraySamplePtr orders,
                             size_t curve_index)
{
  switch (abc_curve_type) {
    case Alembic::AbcGeom::kCubic:
      return 4;
    case Alembic::AbcGeom::kVariableOrder:
      if (orders && orders->size() > curve_index) {
        return static_cast<short>((*orders)[curve_index]);
      }
      ATTR_FALLTHROUGH;
    case Alembic::AbcGeom::kLinear:
    default:
      return 2;
  }
}

static int get_curve_overlap(Alembic::AbcGeom::CurvePeriodicity periodicity,
                             const P3fArraySamplePtr positions,
                             int idx,
                             int num_verts,
                             short order)
{
  if (periodicity == Alembic::AbcGeom::kPeriodic) {
    /* Check the number of points which overlap, we don't have
     * overlapping points in Blender, but other software do use them to
     * indicate that a curve is actually cyclic. Usually the number of
     * overlapping points is equal to the order/degree of the curve.
     */

    const int start = idx;
    const int end = idx + num_verts;
    int overlap = 0;

    for (int j = start, k = end - order; j < order; j++, k++) {
      const Imath::V3f &p1 = (*positions)[j];
      const Imath::V3f &p2 = (*positions)[k];

      if (p1 != p2) {
        break;
      }

      overlap++;
    }

    /* TODO: Special case, need to figure out how it coincides with knots. */
    if (overlap == 0 && num_verts > 2 && (*positions)[start] == (*positions)[end - 1]) {
      overlap = 1;
    }

    /* There is no real cycles. */
    return overlap;
  }

  /* kNonPeriodic is always assumed to have no overlap. */
  return 0;
}

void AbcCurveReader::readObjectData(Main *bmain,
                                    const AbcReaderManager & /*manager*/,
                                    const Alembic::Abc::ISampleSelector &sample_sel)
{
  Curve *cu = BKE_curve_add(bmain, m_data_name.c_str(), OB_CURVE);

  cu->flag |= CU_3D;
  cu->actvert = CU_ACT_NONE;
  cu->resolu = get_curve_resolution(m_curves_schema, sample_sel);

  m_object = BKE_object_add_only_object(bmain, OB_CURVE, m_object_name.c_str());
  m_object->data = cu;

  read_curve_sample(cu, m_curves_schema, sample_sel);

  if (m_settings->always_add_cache_reader || has_animations(m_curves_schema, m_settings)) {
    addCacheModifier();
  }
}

void AbcCurveReader::read_curve_sample(Curve *cu,
                                       const ICurvesSchema &schema,
                                       const ISampleSelector &sample_sel)
{
  ICurvesSchema::Sample smp;
  try {
    smp = schema.getValue(sample_sel);
  }
  catch (Alembic::Util::Exception &ex) {
    printf("Alembic: error reading curve sample for '%s/%s' at time %f: %s\n",
           m_iobject.getFullName().c_str(),
           schema.getName().c_str(),
           sample_sel.getRequestedTime(),
           ex.what());
    return;
  }

  const Int32ArraySamplePtr num_vertices = smp.getCurvesNumVertices();
  const P3fArraySamplePtr positions = smp.getPositions();
  const FloatArraySamplePtr weights = smp.getPositionWeights();
  const FloatArraySamplePtr knots = smp.getKnots();
  const CurvePeriodicity periodicity = smp.getWrap();
  const UcharArraySamplePtr orders = smp.getOrders();

  const IFloatGeomParam widths_param = schema.getWidthsParam();
  FloatArraySamplePtr radiuses;

  if (widths_param.valid()) {
    IFloatGeomParam::Sample wsample = widths_param.getExpandedValue(sample_sel);
    radiuses = wsample.getVals();
  }

  int knot_offset = 0;

  size_t idx = 0;
  for (size_t i = 0; i < num_vertices->size(); i++) {
    const int num_verts = (*num_vertices)[i];

    Nurb *nu = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), "abc_getnurb"));
    nu->resolu = cu->resolu;
    nu->resolv = cu->resolv;
    nu->pntsu = num_verts;
    nu->pntsv = 1;
    nu->flag |= CU_SMOOTH;
    nu->orderu = get_curve_order(smp.getType(), orders, i);

    const int overlap = get_curve_overlap(periodicity, positions, idx, num_verts, nu->orderu);

    if (overlap == 0) {
      nu->flagu |= CU_NURB_ENDPOINT;
    }
    else {
      nu->flagu |= CU_NURB_CYCLIC;
      nu->pntsu -= overlap;
    }

    const bool do_weights = (weights != nullptr) && (weights->size() > 1);
    float weight = 1.0f;

    const bool do_radius = (radiuses != nullptr) && (radiuses->size() > 1);
    float radius = (radiuses && radiuses->size() == 1) ? (*radiuses)[0] : 1.0f;

    nu->type = CU_NURBS;

    nu->bp = static_cast<BPoint *>(MEM_callocN(sizeof(BPoint) * nu->pntsu, "abc_getnurb"));
    BPoint *bp = nu->bp;

    for (int j = 0; j < nu->pntsu; j++, bp++, idx++) {
      const Imath::V3f &pos = (*positions)[idx];

      if (do_radius) {
        radius = (*radiuses)[idx];
      }

      if (do_weights) {
        weight = (*weights)[idx];
      }

      copy_zup_from_yup(bp->vec, pos.getValue());
      bp->vec[3] = weight;
      bp->f1 = SELECT;
      bp->radius = radius;
      bp->weight = 1.0f;
    }

    if (knots && knots->size() != 0) {
      nu->knotsu = static_cast<float *>(
          MEM_callocN(KNOTSU(nu) * sizeof(float), "abc_setsplineknotsu"));

      /* TODO: second check is temporary, for until the check for cycles is rock solid. */
      if (periodicity == Alembic::AbcGeom::kPeriodic && (KNOTSU(nu) == knots->size() - 2)) {
        /* Skip first and last knots. */
        for (size_t i = 1; i < knots->size() - 1; i++) {
          nu->knotsu[i - 1] = (*knots)[knot_offset + i];
        }
      }
      else {
        /* TODO: figure out how to use the knots array from other
         * software in this case. */
        BKE_nurb_knot_calc_u(nu);
      }

      knot_offset += knots->size();
    }
    else {
      BKE_nurb_knot_calc_u(nu);
    }

    BLI_addtail(BKE_curve_nurbs_get(cu), nu);
  }
}

static bool topology_changed(CurveEval *curve_eval, const Int32ArraySamplePtr &num_vertices)
{
  if (!curve_eval) {
    return true;
  }

  const size_t num_curves = num_vertices->size();
  if (num_curves != curve_eval->splines().size()) {
    return true;
  }

  for (size_t i = 0; i < num_vertices->size(); i++) {
    const Spline *spline = curve_eval->splines()[i].get();

    if (!spline) {
      return true;
    }

    // TODO(kevindietrich) : this should check for the overlap.
    if (spline->positions().size() != (*num_vertices)[i]) {
      return true;
    }
  }

  return false;
}

/* NOTE: Alembic only stores data about control points, but the CurveEval
 * passed from the cache modifier contains the displist, which has more data
 * than the control points, so to avoid corrupting the displist we modify the
 * object directly and create a new CurveEval from that. Also we might need to
 * create new or delete existing NURBS in the curve.
 */
void AbcCurveReader::read_geometry(GeometrySet &geometry_set,
                                   const Alembic::Abc::ISampleSelector &sample_sel,
                                   const AttributeSelector * /*attribute_selector*/,
                                   int /*read_flag*/,
                                   const float /*velocity_scale*/,
                                   const char **err_str)
{
  ICurvesSchema::Sample sample;

  try {
    sample = m_curves_schema.getValue(sample_sel);
  }
  catch (Alembic::Util::Exception &ex) {
    *err_str = "Error reading curve sample; more detail on the console";
    printf("Alembic: error reading curve sample for '%s/%s' at time %f: %s\n",
           m_iobject.getFullName().c_str(),
           m_curves_schema.getName().c_str(),
           sample_sel.getRequestedTime(),
           ex.what());
    return;
  }

  CurveEval *curve_eval = geometry_set.get_curve_for_write();

  const Int32ArraySamplePtr num_vertices = sample.getCurvesNumVertices();
  const P3fArraySamplePtr &positions = sample.getPositions();

  if (blender::io::alembic::topology_changed(curve_eval, num_vertices)) {
    Curve *curve = static_cast<Curve *>(m_object->data);
    BKE_nurbList_free(&curve->nurb);
    read_curve_sample(curve, m_curves_schema, sample_sel);

    std::unique_ptr<CurveEval> new_curve_eval = curve_eval_from_dna_curve(*curve);
    geometry_set.replace_curve(new_curve_eval.release(), GeometryOwnershipType::Editable);
  }
  else {
    const IFloatGeomParam widths_param = m_curves_schema.getWidthsParam();
    FloatArraySamplePtr radiuses;

    if (widths_param.valid()) {
      IFloatGeomParam::Sample wsample = widths_param.getExpandedValue(sample_sel);
      radiuses = wsample.getVals();
    }

    const bool do_radius = (radiuses != nullptr) && (radiuses->size() > 1);
    float radius = (radiuses && radiuses->size() == 1) ? (*radiuses)[0] : 1.0f;

    size_t position_index = 0;
    size_t radius_index = 0;

    for (size_t i = 0; i < num_vertices->size(); i++) {
      Spline *spline = curve_eval->splines()[i].get();

      for (float3 &position : spline->positions()) {
        copy_zup_from_yup(position, (*positions)[position_index++].getValue());
      }

      for (float &r : spline->radii()) {
        r = (do_radius) ? (*radiuses)[radius_index++] : radius;
      }
    }
  }
}

}  // namespace blender::io::alembic

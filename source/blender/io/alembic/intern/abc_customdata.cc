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

#include "abc_customdata.h"
#include "abc_axis_conversion.h"

#include <Alembic/AbcGeom/All.h>
#include <algorithm>
#include <unordered_map>

#include "DNA_cachefile_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"

#include "BLI_assert.h"
#include "BLI_color.hh"
#include "BLI_float2.hh"
#include "BLI_float3.hh"
#include "BLI_math_base.h"
#include "BLI_math_bits.h"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "BKE_attribute.h"
#include "BKE_customdata.h"

/* NOTE: for now only UVs and Vertex Colors are supported for streaming.
 * Although Alembic only allows for a single UV layer per {I|O}Schema, and does
 * not have a vertex color concept, there is a convention between DCCs to write
 * such data in a way that lets other DCC know what they are for. See comments
 * in the write code for the conventions. */

using namespace Alembic::AbcGeom;

using Alembic::AbcGeom::kFacevaryingScope;
using Alembic::AbcGeom::kVaryingScope;
using Alembic::AbcGeom::kVertexScope;

using Alembic::Abc::C4fArraySample;
using Alembic::Abc::UInt32ArraySample;
using Alembic::Abc::V2fArraySample;

using Alembic::AbcGeom::OC4fGeomParam;
using Alembic::AbcGeom::OV2fGeomParam;
using Alembic::AbcGeom::OV3fGeomParam;
namespace blender::io::alembic {

/* ORCO, Generated Coordinates, and Reference Points ("Pref") are all terms for the same thing.
 * Other applications (Maya, Houdini) write these to a property called "Pref". */
static const std::string propNameOriginalCoordinates("Pref");

static void get_uvs(const CDStreamConfig &config,
                    std::vector<Imath::V2f> &uvs,
                    std::vector<uint32_t> &uvidx,
                    void *cd_data)
{
  MLoopUV *mloopuv_array = static_cast<MLoopUV *>(cd_data);

  if (!mloopuv_array) {
    return;
  }

  const int num_poly = config.totpoly;
  MPoly *polygons = config.mpoly;
  MLoop *mloop = config.mloop;

  if (!config.pack_uvs) {
    int cnt = 0;
    uvidx.resize(config.totloop);
    uvs.resize(config.totloop);

    /* Iterate in reverse order to match exported polygons. */
    for (int i = 0; i < num_poly; i++) {
      MPoly &current_poly = polygons[i];
      MLoopUV *loopuv = mloopuv_array + current_poly.loopstart + current_poly.totloop;

      for (int j = 0; j < current_poly.totloop; j++, cnt++) {
        loopuv--;

        uvidx[cnt] = cnt;
        uvs[cnt][0] = loopuv->uv[0];
        uvs[cnt][1] = loopuv->uv[1];
      }
    }
  }
  else {
    /* Mapping for indexed UVs, deduplicating UV coordinates at vertices. */
    std::vector<std::vector<uint32_t>> idx_map(config.totvert);
    int idx_count = 0;

    for (int i = 0; i < num_poly; i++) {
      MPoly &current_poly = polygons[i];
      MLoop *looppoly = mloop + current_poly.loopstart + current_poly.totloop;
      MLoopUV *loopuv = mloopuv_array + current_poly.loopstart + current_poly.totloop;

      for (int j = 0; j < current_poly.totloop; j++) {
        looppoly--;
        loopuv--;

        Imath::V2f uv(loopuv->uv[0], loopuv->uv[1]);
        bool found_same = false;

        /* Find UV already in uvs array. */
        for (uint32_t uv_idx : idx_map[looppoly->v]) {
          if (uvs[uv_idx] == uv) {
            found_same = true;
            uvidx.push_back(uv_idx);
            break;
          }
        }

        /* UV doesn't exists for this vertex, add it. */
        if (!found_same) {
          uint32_t uv_idx = idx_count++;
          idx_map[looppoly->v].push_back(uv_idx);
          uvidx.push_back(uv_idx);
          uvs.push_back(uv);
        }
      }
    }
  }
}

const char *get_uv_sample(UVSample &sample, const CDStreamConfig &config, CustomData *data)
{
  const int active_uvlayer = CustomData_get_active_layer(data, CD_MLOOPUV);

  if (active_uvlayer < 0) {
    return "";
  }

  void *cd_data = CustomData_get_layer_n(data, CD_MLOOPUV, active_uvlayer);

  get_uvs(config, sample.uvs, sample.indices, cd_data);

  return CustomData_get_layer_name(data, CD_MLOOPUV, active_uvlayer);
}

/* Convention to write UVs:
 * - V2fGeomParam on the arbGeomParam
 * - set scope as face varying
 * - (optional due to its behavior) tag as UV using Alembic::AbcGeom::SetIsUV
 */
static void write_uv(const OCompoundProperty &prop,
                     CDStreamConfig &config,
                     void *data,
                     const char *name)
{
  std::vector<uint32_t> indices;
  std::vector<Imath::V2f> uvs;

  get_uvs(config, uvs, indices, data);

  if (indices.empty() || uvs.empty()) {
    return;
  }

  std::string uv_map_name(name);
  OV2fGeomParam param = config.abc_uv_maps[uv_map_name];

  if (!param.valid()) {
    param = OV2fGeomParam(prop, name, true, kFacevaryingScope, 1);
  }
  OV2fGeomParam::Sample sample(V2fArraySample(&uvs.front(), uvs.size()),
                               UInt32ArraySample(&indices.front(), indices.size()),
                               kFacevaryingScope);
  param.set(sample);

  config.abc_uv_maps[uv_map_name] = param;
}

/* Convention to write Vertex Colors:
 * - C3fGeomParam/C4fGeomParam on the arbGeomParam
 * - set scope as vertex varying
 */
static void write_mcol(const OCompoundProperty &prop,
                       const CDStreamConfig &config,
                       void *data,
                       const char *name)
{
  const float cscale = 1.0f / 255.0f;
  MPoly *polys = config.mpoly;
  MLoop *mloops = config.mloop;
  MCol *cfaces = static_cast<MCol *>(data);

  std::vector<Imath::C4f> buffer;
  std::vector<uint32_t> indices;

  buffer.reserve(config.totvert);
  indices.reserve(config.totvert);

  Imath::C4f col;

  for (int i = 0; i < config.totpoly; i++) {
    MPoly *p = &polys[i];
    MCol *cface = &cfaces[p->loopstart + p->totloop];
    MLoop *mloop = &mloops[p->loopstart + p->totloop];

    for (int j = 0; j < p->totloop; j++) {
      cface--;
      mloop--;

      col[0] = cface->a * cscale;
      col[1] = cface->r * cscale;
      col[2] = cface->g * cscale;
      col[3] = cface->b * cscale;

      buffer.push_back(col);
      indices.push_back(buffer.size() - 1);
    }
  }

  OC4fGeomParam param(prop, name, true, kFacevaryingScope, 1);

  OC4fGeomParam::Sample sample(C4fArraySample(&buffer.front(), buffer.size()),
                               UInt32ArraySample(&indices.front(), indices.size()),
                               kVertexScope);

  param.set(sample);
}

void write_generated_coordinates(const OCompoundProperty &prop, CDStreamConfig &config)
{
  const void *customdata = CustomData_get_layer(&config.mesh->vdata, CD_ORCO);
  if (customdata == nullptr) {
    /* Data not available, so don't even bother creating an Alembic property for it. */
    return;
  }
  const float(*orcodata)[3] = static_cast<const float(*)[3]>(customdata);

  /* Convert 3D vertices from float[3] z=up to V3f y=up. */
  std::vector<Imath::V3f> coords(config.totvert);
  float orco_yup[3];
  for (int vertex_idx = 0; vertex_idx < config.totvert; vertex_idx++) {
    copy_yup_from_zup(orco_yup, orcodata[vertex_idx]);
    coords[vertex_idx].setValue(orco_yup[0], orco_yup[1], orco_yup[2]);
  }

  if (!config.abc_orco.valid()) {
    /* Create the Alembic property and keep a reference so future frames can reuse it. */
    config.abc_orco = OV3fGeomParam(prop, propNameOriginalCoordinates, false, kVertexScope, 1);
  }

  OV3fGeomParam::Sample sample(coords, kVertexScope);
  config.abc_orco.set(sample);
}

void write_custom_data(const OCompoundProperty &prop,
                       CDStreamConfig &config,
                       CustomData *data,
                       int data_type)
{
  CustomDataType cd_data_type = static_cast<CustomDataType>(data_type);

  if (!CustomData_has_layer(data, cd_data_type)) {
    return;
  }

  const int active_layer = CustomData_get_active_layer(data, cd_data_type);
  const int tot_layers = CustomData_number_of_layers(data, cd_data_type);

  for (int i = 0; i < tot_layers; i++) {
    void *cd_data = CustomData_get_layer_n(data, cd_data_type, i);
    const char *name = CustomData_get_layer_name(data, cd_data_type, i);

    if (cd_data_type == CD_MLOOPUV) {
      /* Already exported. */
      if (i == active_layer) {
        continue;
      }

      write_uv(prop, config, cd_data, name);
    }
    else if (cd_data_type == CD_MLOOPCOL) {
      write_mcol(prop, config, cd_data, name);
    }
  }
}

/* ************************************************************************** */

using Alembic::Abc::C3fArraySamplePtr;
using Alembic::Abc::C4fArraySamplePtr;
using Alembic::Abc::PropertyHeader;
using Alembic::Abc::UInt32ArraySamplePtr;

using Alembic::AbcGeom::IC3fGeomParam;
using Alembic::AbcGeom::IC4fGeomParam;
using Alembic::AbcGeom::IV2fGeomParam;
using Alembic::AbcGeom::IV3fGeomParam;

/* -------------------------------------------------------------------- */

/** \name BlenderScope
 *
 * This enumeration is used to translate the Alembic scope to a scope more suitable for Blender's
 * possible CustomData and attribute domains.
 * It is defined as a bit field in case routines need to resolve the scope for a set of potential
 * scopes. This can be done by or-ing the candidate scopes together, and validate by checking that
 * only a single bit is set.
 * \{ */

enum class BlenderScope : uint32_t {
  UNKNOWN = 0,
  POINT = (1 << 0),
  POLYGON = (1 << 1),
  LOOPS = (1 << 2),
};

static BlenderScope &operator|=(BlenderScope &lhs, BlenderScope rhs)
{
  lhs = static_cast<BlenderScope>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
  return lhs;
}

/* Check the bits for the scope and return it if only one of them is active. */
static BlenderScope valid_scope_or_unknown(BlenderScope scope)
{
  if (count_bits_i(static_cast<uint32_t>(scope)) == 1) {
    return scope;
  }

  return BlenderScope::UNKNOWN;
}

/* Determine the matching scope given some number of values. The dimensions parameter is used to
 * scale the scope_sizes and is meant to be used (not equal to 1) when processing a scalar array
 * for a possible remapping to some n-dimensionnal data type. */
static BlenderScope matching_scope(const ScopeSizeInfo scope_sizes,
                                   int dimensions,
                                   size_t num_values)
{
  BlenderScope scope = BlenderScope::UNKNOWN;

  if (scope_sizes.loop_scope_size != 0 &&
      static_cast<size_t>(scope_sizes.loop_scope_size * dimensions) == num_values) {
    scope |= BlenderScope::LOOPS;
  }

  if (scope_sizes.polygon_scope_size != 0 &&
      static_cast<size_t>(scope_sizes.polygon_scope_size * dimensions) == num_values) {
    scope |= BlenderScope::POLYGON;
  }

  if (scope_sizes.point_scope_size != 0 &&
      static_cast<size_t>(scope_sizes.point_scope_size * dimensions) == num_values) {
    scope |= BlenderScope::POINT;
  }

  return valid_scope_or_unknown(scope);
}

static CustomData *custom_data_for_scope(ScopeCustomDataPointers custom_data_pointers,
                                         BlenderScope bl_scope)
{
  if (bl_scope == BlenderScope::POINT) {
    return custom_data_pointers.point_custom_data;
  }

  if (bl_scope == BlenderScope::POLYGON) {
    return custom_data_pointers.polygon_custom_data;
  }

  if (bl_scope == BlenderScope::LOOPS) {
    return custom_data_pointers.loop_custom_data;
  }

  return nullptr;
}

#if 0
/* Useful for debugging. Turned off to quiet warnings. */
static std::ostream &operator<<(std::ostream &os, BlenderScope bl_scope)
{
  switch (bl_scope) {
    case BlenderScope::POINT: {
      os << "POINT";
      break;
    }
    case BlenderScope::LOOPS: {
      os << "LOOPS";
      break;
    }
    case BlenderScope::POLYGON: {
      os << "POLYGON";
      break;
    }
    default: {
      os << "UNKNOWN";
      break;
    }
  }
  return os;
}
#endif

static int size_for_scope(const ScopeSizeInfo scope_sizes, BlenderScope scope)
{
  if (scope == BlenderScope::POINT) {
    return scope_sizes.point_scope_size;
  }

  if (scope == BlenderScope::LOOPS) {
    return scope_sizes.loop_scope_size;
  }

  if (scope == BlenderScope::POLYGON) {
    return scope_sizes.polygon_scope_size;
  }

  return 0;
}

/* UVs can be defined per-loop (one value per vertex per face), or per-vertex (one value per
 * vertex). The first case is the most common, as this is the standard way of storing this data
 * given that some vertices might be on UV seams and have multiple possible UV coordinates; the
 * second case can happen when the mesh is split according to the UV islands, in which case storing
 * a single UV value per vertex allows to deduplicate data and thus to reduce the file size since
 * vertices are guaranteed to only have a single UV coordinate. */
static bool is_valid_uv_scope(BlenderScope scope)
{
  return scope == BlenderScope::LOOPS || scope == BlenderScope::POINT;
}

static bool is_valid_vertex_color_scope(BlenderScope scope)
{
  return scope == BlenderScope::LOOPS || scope == BlenderScope::POINT;
}

static bool is_valid_vertex_group_scope(BlenderScope scope)
{
  return scope == BlenderScope::POINT;
}

static bool is_valid_scope_for_layer(BlenderScope scope, CustomDataType custom_data_type)
{
  if (scope == BlenderScope::UNKNOWN) {
    return false;
  }

  if (custom_data_type == CD_MLOOPUV) {
    return is_valid_uv_scope(scope);
  }

  if (custom_data_type == CD_MCOL) {
    return is_valid_vertex_color_scope(scope);
  }

  if (custom_data_type == CD_ORCO) {
    return scope == BlenderScope::POINT;
  }

  return true;
}

/* Converts an Alembic scope to a Blender one. We need to be careful as some Alembic scopes depend
 * on the domain on which they appear. For example, kVaryingScope could mean that the data is
 * varying across the polygons or the vertices. To resolve this, we also use the expected number of
 * elements for each scopes, and the matching scope is returned based on this.
 * If we cannot resolve the scope, BlenderScope::UNKNOWN is returned.
 */
static BlenderScope to_blender_scope(Alembic::AbcGeom::GeometryScope abc_scope,
                                     size_t element_size,
                                     ScopeSizeInfo scope_sizes)
{
  switch (abc_scope) {
    case kConstantScope: {
      return matching_scope(scope_sizes, 1, element_size);
    }
    case kUniformScope: {
      /* This would mean one value for the whole object, but we don't support that yet. */
      return BlenderScope::UNKNOWN;
    }
    case kVertexScope: {
      /* This is pretty straightforward, just ensure that the sizes match. */
      if (static_cast<size_t>(scope_sizes.point_scope_size) == element_size) {
        return BlenderScope::POINT;
      }
      return BlenderScope::UNKNOWN;
    }
    case kFacevaryingScope: {
      /* Fist check that the loops size match, since this is the right domain. */
      if (static_cast<size_t>(scope_sizes.loop_scope_size) == element_size) {
        return BlenderScope::LOOPS;
      }
      /* If not, we may have some data that is face varying, but was written for each vertex
       * instead (e.g. vertex colors). */
      if (static_cast<size_t>(scope_sizes.point_scope_size) == element_size) {
        return BlenderScope::POINT;
      }
      return BlenderScope::UNKNOWN;
    }
    case kVaryingScope: {
      /* We have a varying field over some domain, which we need to determine. */
      return matching_scope(scope_sizes, 1, element_size);
    }
    case kUnknownScope: {
      return BlenderScope::UNKNOWN;
    }
  }

  return BlenderScope::UNKNOWN;
}

static AttributeDomain attribute_domain_from_blender_scope(BlenderScope scope)
{
  if (scope == BlenderScope::POINT) {
    return ATTR_DOMAIN_POINT;
  }

  if (scope == BlenderScope::LOOPS) {
    return ATTR_DOMAIN_CORNER;
  }

  if (scope == BlenderScope::POLYGON) {
    return ATTR_DOMAIN_FACE;
  }

  BLI_assert_msg(0, "Obtained an invalid scope when converting to an AttributeDomain");
  return ATTR_DOMAIN_NUM;
}

/** \} */

/* -------------------------------------------------------------------- */

/** \name value_type_converter
 *
 * Utilities to convert values from the types used in Alembic to the ones used in Blender. Besides
 * type conversions, vectors are also converted from Y-up to Z-up.
 * \{ */

template<typename T> struct value_type_converter {
  using blender_type = float;

  static blender_type convert_value(T value)
  {
    return static_cast<float>(value);
  }

  /* The following are used for attribute remapping. */

  static float2 map_to_float2(const T *value)
  {
    return {static_cast<float>(value[0]), static_cast<float>(value[1])};
  }

  static float3 map_to_float3(const T *value)
  {
    return {
        static_cast<float>(value[0]), static_cast<float>(value[1]), static_cast<float>(value[2])};
  }

  static ColorGeometry4f map_to_color_from_rgb(const T *ptr)
  {
    return ColorGeometry4f(
        static_cast<float>(ptr[0]), static_cast<float>(ptr[1]), static_cast<float>(ptr[2]), 1.0f);
  }

  static ColorGeometry4f map_to_color_from_rgba(const T *ptr)
  {
    return ColorGeometry4f(static_cast<float>(ptr[0]),
                           static_cast<float>(ptr[1]),
                           static_cast<float>(ptr[2]),
                           static_cast<float>(ptr[3]));
  }

  static MCol map_to_mcol_from_rgb(const T *ptr)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(static_cast<float>(ptr[0]));
    mcol.r = unit_float_to_uchar_clamp(static_cast<float>(ptr[1]));
    mcol.g = unit_float_to_uchar_clamp(static_cast<float>(ptr[2]));
    mcol.b = 255;
    return mcol;
  }

  static MCol map_to_mcol_from_rgba(const T *ptr)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(static_cast<float>(ptr[0]));
    mcol.r = unit_float_to_uchar_clamp(static_cast<float>(ptr[1]));
    mcol.g = unit_float_to_uchar_clamp(static_cast<float>(ptr[2]));
    mcol.b = unit_float_to_uchar_clamp(static_cast<float>(ptr[3]));
    return mcol;
  }
};

template<> struct value_type_converter<bool> {
  using blender_type = bool;

  static blender_type convert_value(bool value)
  {
    return value;
  }
};

template<> struct value_type_converter<int> {
  using blender_type = int;

  static blender_type convert_value(int value)
  {
    return value;
  }
};

template<> struct value_type_converter<Imath::V2f> {
  using blender_type = float2;

  static blender_type convert_value(Imath::V2f value)
  {
    return float2(value.x, value.y);
  }
};

template<> struct value_type_converter<Imath::V3f> {
  using blender_type = float3;

  static blender_type convert_value(Imath::V3f value)
  {
    float3 tmp;
    copy_yup_from_zup(tmp, value.getValue());
    return tmp;
  }
};

template<> struct value_type_converter<Imath::V3d> {
  using blender_type = float3;

  static blender_type convert_value(Imath::V3d value)
  {
    Imath::V3f to_v3f = value;
    float3 tmp;
    copy_yup_from_zup(tmp, to_v3f.getValue());
    return tmp;
  }
};

template<> struct value_type_converter<Imath::C3f> {
  using blender_type = ColorGeometry4f;

  static blender_type convert_value(Imath::C3f value)
  {
    return ColorGeometry4f(value.x, value.y, value.z, 1.0f);
  }

  /* Vertex colors are a special case since the code is meant to be generic, the color attributes
   * should also work for point clouds or curves, so by default we do not consider them as vertex
   * colors. */
  static MCol convert_mcol(Imath::C3f color)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(color[0]);
    mcol.r = unit_float_to_uchar_clamp(color[1]);
    mcol.g = unit_float_to_uchar_clamp(color[2]);
    mcol.b = 255;
    return mcol;
  }
};

template<> struct value_type_converter<Imath::C4f> {
  using blender_type = ColorGeometry4f;

  static blender_type convert_value(Imath::C4f value)
  {
    return ColorGeometry4f(value.r, value.g, value.b, value.a);
  }

  /* Vertex colors are a special case since the code is meant to be generic, the color attributes
   * should also work for point clouds or curves, so by default we do not consider them as vertex
   * colors. */
  static MCol convert_mcol(Imath::C4f color)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(color[0]);
    mcol.r = unit_float_to_uchar_clamp(color[1]);
    mcol.g = unit_float_to_uchar_clamp(color[2]);
    mcol.b = unit_float_to_uchar_clamp(color[3]);
    return mcol;
  }
};

/** \} */

/* -------------------------------------------------------------------- */

static size_t mcols_out_of_bounds_check(const size_t color_index,
                                        const size_t array_size,
                                        const std::string &iobject_full_name,
                                        const PropertyHeader &prop_header,
                                        bool &r_is_out_of_bounds,
                                        bool &r_bounds_warning_given)
{
  if (color_index < array_size) {
    return color_index;
  }

  if (!r_bounds_warning_given) {
    std::cerr << "Alembic: color index out of bounds "
                 "reading face colors for object "
              << iobject_full_name << ", property " << prop_header.getName() << std::endl;
    r_bounds_warning_given = true;
  }
  r_is_out_of_bounds = true;
  return 0;
}

template<typename TRAIT>
static bool should_read_attribute(const AttributeSelector &attr_sel,
                                  const ITypedGeomParam<TRAIT> &param,
                                  CustomDataType custom_data_type,
                                  ScopeSizeInfo scope_sizes,
                                  ISampleSelector sample_selector)
{
  typename ITypedGeomParam<TRAIT>::Sample sample;
  param.getIndexed(sample, sample_selector);

  if (!sample.valid()) {
    return false;
  }

  const TypedArraySample<TRAIT> &values = *sample.getVals();
  const BlenderScope bl_scope = to_blender_scope(param.getScope(), values.size(), scope_sizes);

  if (!is_valid_scope_for_layer(bl_scope, custom_data_type)) {
    return false;
  }

  return attr_sel.select_attribute(bl_scope, param.getName());
}

/* -------------------------------------------------------------------- */

/** \name AbcAttributeMapping
 *
 * Mirror of the mapping enumeration in DNA_cachefile_types.h. It is more specific and used as
 * some sort of token to indicate that the mapping is valid for the attribute.
 * \{ */

enum class AbcAttributeMapping {
  /* The mapping for the attribute is impossible. Either the number of values do not match, or the
   * resolved scope is not supported. */
  IMPOSSIBLE,
  /* It may be possible that no mapping is needed, so this will just load the attribute directly if
   * possible. */
  IDENTITY,
  /* The rest are the possible mappings. The color mapping are split between RGB and RGBA
   * (depending on the number of elements in the original array). */
  TO_UVS,
  TO_VERTEX_COLORS_FROM_RGB,
  TO_VERTEX_COLORS_FROM_RGBA,
  TO_VERTEX_GROUP,
  TO_FLOAT2,
  TO_FLOAT3,
  TO_COLOR_FROM_RGB,
  TO_COLOR_FROM_RGBA,
};

/* Try to resolve the mapping for an attribute from the number of individual elements in the array
 * sample and the mapping as desired by the user. Returns the AbcAttributeMapping, and the Blender
 * scope for the attribute. */
template<typename TRAIT>
static AbcAttributeMapping determine_abc_mapping_and_scope(
    const CacheAttributeMapping &mapping,
    const TypedArraySample<TRAIT> &array_sample,
    const ScopeSizeInfo scope_sizes,
    BlenderScope &r_scope_hint)
{
  static_assert(std::is_floating_point_v<typename TRAIT::value_type>,
                "Only floating point attributes can be remapped");

  const size_t num_elements = array_sample.size();
  BLI_assert_msg(num_elements != 0,
                 "The number of elements in the array is null, although this should have been "
                 "checked before.");

  r_scope_hint = BlenderScope::UNKNOWN;

  switch (mapping.mapping) {
    case CACHEFILE_ATTRIBUTE_MAP_NONE: {
      return AbcAttributeMapping::IDENTITY;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_UVS: {
      /* UVs have 2 values per element. */
      r_scope_hint = matching_scope(scope_sizes, 2, num_elements);

      if (!is_valid_uv_scope(r_scope_hint)) {
        return AbcAttributeMapping::IMPOSSIBLE;
      }

      return AbcAttributeMapping::TO_UVS;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_VERTEX_COLORS: {
      /* 3 values for RGB, 4 for RGBA */
      BlenderScope rgb_scope_hint = matching_scope(scope_sizes, 3, num_elements);
      BlenderScope rgba_scope_hint = matching_scope(scope_sizes, 4, num_elements);

      if (is_valid_vertex_color_scope(rgb_scope_hint) &&
          rgba_scope_hint == BlenderScope::UNKNOWN) {
        r_scope_hint = rgb_scope_hint;
        return AbcAttributeMapping::TO_VERTEX_COLORS_FROM_RGB;
      }

      if (rgb_scope_hint == BlenderScope::UNKNOWN &&
          is_valid_vertex_color_scope(rgba_scope_hint)) {
        r_scope_hint = rgba_scope_hint;
        return AbcAttributeMapping::TO_VERTEX_COLORS_FROM_RGBA;
      }

      /* Either both are unknown, or we matched two different scopes in which case we cannot
       * resolve the ambiguity. */
      r_scope_hint = BlenderScope::UNKNOWN;
      return AbcAttributeMapping::IMPOSSIBLE;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_WEIGHT_GROUPS: {
      r_scope_hint = matching_scope(scope_sizes, 1, num_elements);
      if (!is_valid_vertex_group_scope(r_scope_hint)) {
        return AbcAttributeMapping::IMPOSSIBLE;
      }
      return AbcAttributeMapping::TO_VERTEX_GROUP;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT2: {
      r_scope_hint = matching_scope(scope_sizes, 2, num_elements);
      return AbcAttributeMapping::TO_FLOAT2;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT3: {
      r_scope_hint = matching_scope(scope_sizes, 3, num_elements);
      return AbcAttributeMapping::TO_FLOAT3;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_COLOR: {
      /* 3 values for RGB, 4 for RGBA */
      BlenderScope rgb_scope_hint = matching_scope(scope_sizes, 3, num_elements);
      BlenderScope rgba_scope_hint = matching_scope(scope_sizes, 4, num_elements);

      if (rgb_scope_hint != BlenderScope::UNKNOWN && rgba_scope_hint == BlenderScope::UNKNOWN) {
        r_scope_hint = rgb_scope_hint;
        return AbcAttributeMapping::TO_COLOR_FROM_RGB;
      }

      if (rgb_scope_hint == BlenderScope::UNKNOWN && rgba_scope_hint != BlenderScope::UNKNOWN) {
        r_scope_hint = rgba_scope_hint;
        return AbcAttributeMapping::TO_COLOR_FROM_RGB;
      }

      /* Either both are unknown, or we matched two different scopes in which case we cannot
       * resolve the ambiguity. */
      r_scope_hint = BlenderScope::UNKNOWN;
      return AbcAttributeMapping::IMPOSSIBLE;
    }
  }

  return AbcAttributeMapping::IMPOSSIBLE;
}

/** \} */

/* -------------------------------------------------------------------- */

static bool can_add_custom_data_layer(const CustomData *data,
                                      CustomDataType type,
                                      int max_allowed_layers)
{
  if (data == nullptr) {
    return false;
  }

  return CustomData_number_of_layers(data, type) < max_allowed_layers;
}

/* Check if we do not already have reached the maximum allowed number of vertex colors layers. */
static bool can_add_vertex_color_layer(const CDStreamConfig &config)
{
  return can_add_custom_data_layer(
      config.custom_data_pointers.loop_custom_data, CD_MLOOPCOL, MAX_MCOL);
}

/* Check if we do not already have reached the maximum allowed number of UV layers. */
static bool can_add_uv_layer(const CDStreamConfig &config)
{
  return can_add_custom_data_layer(
      config.custom_data_pointers.loop_custom_data, CD_MLOOPUV, MAX_MTFACE);
}

/* For converting attributes with a loop scope, we need to convert the polygon winding order as
 * well, as viewed from Blender, Alembic orders vertices around a polygon in reverse.
 * The callback will called for each loop, and the index passed to it will be the index of the data
 * in the source_scope. If source_scope is POINT, the passed index will be that of the point of the
 * corresponding loop vertex. If it is LOOPS, it will be that of the loop.
 * For now, we do support POLYGON scope as a source_scope.
 */
template<typename BlenderType, typename Callback>
static void iterate_attribute_loop_scope(const CDStreamConfig &config,
                                         BlenderType *attribute_data,
                                         BlenderScope source_scope,
                                         Callback callback)
{
  if (source_scope != BlenderScope::LOOPS && source_scope != BlenderScope::POINT) {
    return;
  }

  Mesh *mesh = config.mesh;
  MPoly *mpolys = mesh->mpoly;
  MLoop *mloops = mesh->mloop;
  unsigned int loop_index, rev_loop_index;
  const bool index_per_loop = source_scope == BlenderScope::LOOPS;

  for (int i = 0; i < mesh->totpoly; i++) {
    MPoly &poly = mpolys[i];
    unsigned int rev_loop_offset = poly.loopstart + poly.totloop - 1;

    for (int f = 0; f < poly.totloop; f++) {
      rev_loop_index = rev_loop_offset - f;
      loop_index = index_per_loop ? poly.loopstart + f : mloops[rev_loop_index].v;
      attribute_data[rev_loop_index] = callback(loop_index);
    }
  }
}

/* Main entry point for creating a custom data layer from an Alembic attribute.
 *
 * The callback is responsible for converting filling the custom data layer, one value at a time.
 * Its signature should be : BlenderType(size_t). The parameter is the current index of the
 * iteration, and is dependant on the target_scope.
 *
 * The source_scope is the resolved scope for the Alembic data, while the target_scope is the scope
 * of the final Blender data. We split these as they can be different either due to remapping, or
 * because we have an attribute that is always on one scope in Blender, but can be expressed in
 * multiple scopes in Alembic (e.g. UV maps are loop scope in Blender, but loop or vertex in
 * Alembic).
 */
template<typename BlenderType, typename Callback>
static void create_layer_for_scope(const CDStreamConfig &config,
                                   BlenderScope source_scope,
                                   BlenderScope target_scope,
                                   CustomDataType cd_type,
                                   const std::string &name,
                                   Callback callback)
{
  BLI_assert(source_scope != BlenderScope::UNKNOWN);
  BLI_assert(target_scope != BlenderScope::UNKNOWN);

  const ScopeCustomDataPointers &custom_data_pointers = config.custom_data_pointers;
  CustomData *custom_data = custom_data_for_scope(custom_data_pointers, target_scope);
  if (!custom_data) {
    return;
  }

  const int layer_size = size_for_scope(config.scope_sizes, target_scope);

  const AttributeDomain domain = attribute_domain_from_blender_scope(target_scope);
  CustomDataLayer *layer = BKE_id_attribute_new(config.id, name.c_str(), cd_type, domain, nullptr);

  BlenderType *layer_data = static_cast<BlenderType *>(layer->data);

  if (target_scope == BlenderScope::LOOPS) {
    iterate_attribute_loop_scope(config, layer_data, source_scope, callback);
    return;
  }

  /* POINT and POLYGON scopes can be simply iterated. */
  for (size_t i = 0; i < static_cast<size_t>(layer_size); i++) {
    *layer_data++ = callback(i);
  }
}

/* Wrapper around create_layer_for_scope with similar source and target scopes. */
template<typename BlenderType, typename Callback>
static void create_layer_for_scope(const CDStreamConfig &config,
                                   BlenderScope bl_scope,
                                   CustomDataType cd_type,
                                   const std::string &name,
                                   Callback callback)
{
  return create_layer_for_scope<BlenderType>(config, bl_scope, bl_scope, cd_type, name, callback);
}

/* Wrapper around create_layer_for_scope with the loop scope being the target scope. */
template<typename BlenderType, typename Callback>
static void create_loop_layer_for_scope(const CDStreamConfig &config,
                                        BlenderScope source_scope,
                                        CustomDataType cd_type,
                                        const std::string &name,
                                        Callback callback)
{
  return create_layer_for_scope<BlenderType>(
      config, source_scope, BlenderScope::LOOPS, cd_type, name, callback);
}

/* Wrapper around create_layer_for_scope with a default (generic) callback. */
template<typename TRAIT>
static void create_layer_for_scope_generic(const CDStreamConfig &config,
                                           CustomDataType cd_type,
                                           const std::string &name,
                                           const TypedArraySample<TRAIT> &values,
                                           BlenderScope bl_scope)
{
  using abc_type = typename TRAIT::value_type;
  using blender_type = typename value_type_converter<abc_type>::blender_type;
  create_layer_for_scope<blender_type>(config, bl_scope, cd_type, name, [&](size_t i) {
    return value_type_converter<abc_type>::convert_value(values[i]);
  });
}

/* Special function for velocities since we overwride the name. */
template<typename TRAIT>
static void process_velocity_attribute(const CDStreamConfig &config,
                                       const ITypedGeomParam<TRAIT> &param,
                                       const ISampleSelector iss,
                                       const float velocity_scale)
{
  if (!config.id || velocity_scale == 0.0f) {
    return;
  }

  if (!param.valid()) {
    return;
  }

  typename ITypedGeomParam<TRAIT>::Sample sample;
  param.getIndexed(sample, iss);

  if (!sample.valid()) {
    return;
  }

  const TypedArraySample<TRAIT> &values = *sample.getVals();

  BlenderScope scope = to_blender_scope(param.getScope(), values.size(), config.scope_sizes);

  if (scope != BlenderScope::POINT) {
    return;
  }

  using abc_type = typename TRAIT::value_type;
  create_layer_for_scope<float3>(config, scope, CD_PROP_FLOAT3, "velocity", [&](size_t i) {
    return value_type_converter<abc_type>::convert_value(values[i]) * velocity_scale;
  });
}

template<typename TRAIT>
static void process_typed_attribute(const CDStreamConfig &config,
                                    CustomDataType cd_type,
                                    const ITypedGeomParam<TRAIT> &param,
                                    ISampleSelector iss)
{
  typename ITypedGeomParam<TRAIT>::Sample sample;
  param.getIndexed(sample, iss);

  if (!sample.valid()) {
    return;
  }

  const ScopeSizeInfo &scope_sizes = config.scope_sizes;
  const AttributeSelector &attr_sel = *config.attr_selector;

  if (!should_read_attribute(attr_sel, param, cd_type, scope_sizes, iss)) {
    return;
  }

  const TypedArraySample<TRAIT> &values = *sample.getVals();
  BlenderScope bl_scope = to_blender_scope(param.getScope(), values.size(), scope_sizes);
  create_layer_for_scope_generic(config, cd_type, param.getName(), values, bl_scope);
}

template<typename TRAIT>
static void process_typed_attribute_with_mapping(const CDStreamConfig &config,
                                                 const CacheAttributeMapping *mapping,
                                                 CustomDataType cd_type,
                                                 const ITypedGeomParam<TRAIT> &param,
                                                 ISampleSelector iss)
{
  typename ITypedGeomParam<TRAIT>::Sample sample;
  param.getIndexed(sample, iss);

  if (!sample.valid()) {
    return;
  }

  const ScopeSizeInfo &scope_sizes = config.scope_sizes;
  const AttributeSelector &attr_sel = *config.attr_selector;

  const TypedArraySample<TRAIT> &values = *sample.getVals();

  if (!mapping) {
    if (!should_read_attribute(attr_sel, param, cd_type, scope_sizes, iss)) {
      return;
    }

    BlenderScope bl_scope = to_blender_scope(param.getScope(), values.size(), scope_sizes);
    create_layer_for_scope_generic(config, cd_type, param.getName(), values, bl_scope);
    return;
  }

  BlenderScope bl_scope;
  AbcAttributeMapping abc_mapping = determine_abc_mapping_and_scope(
      *mapping, values, scope_sizes, bl_scope);

  if (bl_scope == BlenderScope::UNKNOWN) {
    return;
  }

  /* TODO(kevindietrich): check if selected */

  /* TODO(kevindietrich): how to verify that the scope from the mapping matches the one from the
   * archive? */

  using abc_type = typename TRAIT::value_type;
  const abc_type *input_data = static_cast<const abc_type *>(values.get());

  switch (abc_mapping) {
    case AbcAttributeMapping::IMPOSSIBLE: {
      return;
    }
    case AbcAttributeMapping::IDENTITY: {
      bl_scope = to_blender_scope(param.getScope(), values.size(), scope_sizes);
      create_layer_for_scope_generic(config, cd_type, param.getName(), values, bl_scope);
      return;
    }
    case AbcAttributeMapping::TO_UVS: {
      if (!can_add_uv_layer(config)) {
        return;
      }

      create_loop_layer_for_scope<float2>(
          config, bl_scope, CD_MLOOPUV, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_float2(&input_data[i * 2]);
          });

      return;
    }
    case AbcAttributeMapping::TO_VERTEX_COLORS_FROM_RGB: {
      if (!can_add_vertex_color_layer(config)) {
        return;
      }

      create_loop_layer_for_scope<MCol>(
          config, bl_scope, CD_MLOOPCOL, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_mcol_from_rgb(&input_data[i * 3]);
          });
      return;
    }
    case AbcAttributeMapping::TO_VERTEX_COLORS_FROM_RGBA: {
      if (!can_add_vertex_color_layer(config)) {
        return;
      }

      create_loop_layer_for_scope<MCol>(
          config, bl_scope, CD_MLOOPCOL, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_mcol_from_rgba(&input_data[i * 4]);
          });
      return;
    }
    case AbcAttributeMapping::TO_VERTEX_GROUP: {
      if (config.mesh) {
        Mesh *mesh = config.mesh;
        MVert *mvert = config.mvert;
        for (int i = 0; i < mesh->totvert; i++) {
          mvert[i].bweight = unit_float_to_uchar_clamp(
              value_type_converter<abc_type>::convert_value(input_data[i]));
        }
      }
      return;
    }
    case AbcAttributeMapping::TO_FLOAT2: {
      create_layer_for_scope<float2>(
          config, bl_scope, CD_PROP_FLOAT2, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_float2(&input_data[i * 2]);
          });
      return;
    }
    case AbcAttributeMapping::TO_FLOAT3: {
      create_layer_for_scope<float3>(
          config, bl_scope, CD_PROP_FLOAT3, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_float3(&input_data[i * 3]);
          });
      return;
    }
    case AbcAttributeMapping::TO_COLOR_FROM_RGB: {
      create_layer_for_scope<ColorGeometry4f>(
          config, bl_scope, CD_PROP_COLOR, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_color_from_rgb(&input_data[i * 3]);
          });
      return;
    }
    case AbcAttributeMapping::TO_COLOR_FROM_RGBA: {
      create_layer_for_scope<ColorGeometry4f>(
          config, bl_scope, CD_PROP_COLOR, param.getName(), [&](size_t i) {
            return value_type_converter<abc_type>::map_to_color_from_rgba(&input_data[i * 4]);
          });
      return;
    }
  }
}

static void read_mesh_uvs(const CDStreamConfig &config,
                          const IV2fGeomParam &uv_param,
                          ISampleSelector sample_sel)
{
  BLI_assert(config.mesh);

  if (!uv_param.valid() || !uv_param.isIndexed()) {
    return;
  }

  IV2fGeomParam::Sample sample;
  uv_param.getIndexed(sample, sample_sel);

  const Alembic::AbcGeom::V2fArraySamplePtr &uvs = sample.getVals();
  const Alembic::AbcGeom::UInt32ArraySamplePtr &indices = sample.getIndices();

  const BlenderScope bl_scope = to_blender_scope(
      uv_param.getScope(), indices->size(), config.scope_sizes);

  if (!is_valid_uv_scope(bl_scope)) {
    return;
  }

  /* According to the convention, primary UVs should have had their name set using
   * Alembic::Abc::SetSourceName. If there is no such name, use the name defined for the uv_param.
   */
  std::string name = Alembic::Abc::GetSourceName(uv_param.getMetaData());
  if (name.empty()) {
    name = uv_param.getName();
  }

  create_loop_layer_for_scope<MLoopUV>(config, bl_scope, CD_MLOOPUV, name, [&](size_t loop_index) {
    size_t uv_index = (*indices)[loop_index];
    const Imath::V2f &uv = (*uvs)[uv_index];

    MLoopUV result;
    result.uv[0] = uv[0];
    result.uv[1] = uv[1];
    return result;
  });
}

template<typename TRAIT>
static void read_mesh_colors_generic(const CDStreamConfig &config,
                                     const PropertyHeader &prop_header,
                                     const ITypedGeomParam<TRAIT> &color_param,
                                     ISampleSelector iss)
{
  typename ITypedGeomParam<TRAIT>::Sample sample;
  color_param.getIndexed(sample, iss);

  if (!sample.valid()) {
    return;
  }

  const TypedArraySample<TRAIT> &values = *sample.getVals();
  const Alembic::Abc::UInt32ArraySamplePtr &indices = sample.getIndices();

  size_t num_values = indices->size();
  if (num_values == 0) {
    num_values = values.size();
  }

  const BlenderScope bl_scope = to_blender_scope(
      color_param.getScope(), num_values, config.scope_sizes);

  if (!is_valid_vertex_color_scope(bl_scope)) {
    return;
  }

  const bool is_facevarying = bl_scope == BlenderScope::LOOPS;

  /* Read the vertex colors */
  bool bounds_warning_given = false;

  /* The colors can go through two layers of indexing. Often the 'indices'
   * array doesn't do anything (i.e. indices[n] = n), but when it does, it's
   * important. Blender 2.79 writes indices incorrectly (see T53745), which
   * is why we have to check for indices->size() > 0 */
  bool use_dual_indexing = is_facevarying && indices->size() > 0;

  create_loop_layer_for_scope<MCol>(
      config, bl_scope, CD_MLOOPCOL, prop_header.getName(), [&](size_t loop_index) {
        size_t color_index = loop_index;
        if (use_dual_indexing) {
          color_index = (*indices)[color_index];
        }

        bool is_mcols_out_of_bounds = false;
        color_index = mcols_out_of_bounds_check(color_index,
                                                values.size(),
                                                config.iobject_full_name,
                                                prop_header,
                                                is_mcols_out_of_bounds,
                                                bounds_warning_given);

        if (is_mcols_out_of_bounds) {
          return MCol{};
        }

        using abc_type = typename TRAIT::value_type;
        return value_type_converter<abc_type>::convert_mcol(values[color_index]);
      });
}

static void read_mesh_vertex_colors_c3f(const CDStreamConfig &config,
                                        const ICompoundProperty &prop,
                                        const PropertyHeader &prop_header,
                                        ISampleSelector sample_sel)
{
  IC3fGeomParam color_param(prop, prop_header.getName());

  if (!color_param.valid()) {
    return;
  }

  BLI_assert(STREQ("rgb", color_param.getInterpretation()));
  read_mesh_colors_generic(config, prop_header, color_param, sample_sel);
}

static void read_mesh_vertex_colors_c4f(const CDStreamConfig &config,
                                        const ICompoundProperty &prop,
                                        const PropertyHeader &prop_header,
                                        ISampleSelector sample_sel)
{
  IC4fGeomParam color_param(prop, prop_header.getName());

  if (!color_param.valid()) {
    return;
  }

  BLI_assert(STREQ("rgba", color_param.getInterpretation()));
  read_mesh_colors_generic(config, prop_header, color_param, sample_sel);
}

/* This structure holds data for an attribute found on the Alembic object. */
struct ParsedAttributeDesc {
  ICompoundProperty parent;
  const PropertyHeader &prop_header;
  const CacheAttributeMapping *mapping;
};

/* Extract supported attributes from the ICompoundProperty, and associate them with any mapping
 * with a matching name. */
static Vector<ParsedAttributeDesc> parse_attributes(const AttributeSelector *attr_sel,
                                                    const ICompoundProperty &arb_geom_params)
{
  Vector<ParsedAttributeDesc> result;
  if (!arb_geom_params.valid()) {
    return result;
  }

  for (size_t i = 0; i < arb_geom_params.getNumProperties(); ++i) {
    const PropertyHeader &prop = arb_geom_params.getPropertyHeader(i);

    /* TODO(kevindietrich): support scalar properties. */
    if (prop.isScalar()) {
      continue;
    }

    const CacheAttributeMapping *mapping = attr_sel ? attr_sel->get_mapping(prop.getName()) :
                                                      nullptr;
    result.append({arb_geom_params, prop, mapping});
  }

  return result;
}

void read_arbitrary_attributes(const CDStreamConfig &config,
                               const ICompoundProperty &arb_geom_params,
                               const IV2fGeomParam &primary_uvs,
                               const ISampleSelector &sample_sel,
                               float velocity_scale)
{

  /* The attribute selector may be null when run through the import operator. */
  if (!config.attr_selector) {
    return;
  }

  const AttributeSelector &attr_sel = *config.attr_selector;

  Vector<ParsedAttributeDesc> attributes = parse_attributes(config.attr_selector, arb_geom_params);

  if (primary_uvs.valid() && attr_sel.uvs_requested()) {
    read_mesh_uvs(config, primary_uvs, sample_sel);
  }

  for (ParsedAttributeDesc desc : attributes) {
    const PropertyHeader &prop = desc.prop_header;

    if (IFloatGeomParam::matches(prop)) {
      IFloatGeomParam param = IFloatGeomParam(desc.parent, prop.getName());
      process_typed_attribute_with_mapping(config, desc.mapping, CD_PROP_FLOAT, param, sample_sel);
    }
    else if (IDoubleGeomParam::matches(prop)) {
      IDoubleGeomParam param = IDoubleGeomParam(desc.parent, prop.getName());
      process_typed_attribute_with_mapping(config, desc.mapping, CD_PROP_FLOAT, param, sample_sel);
    }
    else if (IBoolGeomParam::matches(prop)) {
      IBoolGeomParam param = IBoolGeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_BOOL, param, sample_sel);
    }
    else if (IInt32GeomParam::matches(prop)) {
      IInt32GeomParam param = IInt32GeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_INT32, param, sample_sel);
    }
    else if (IV2fGeomParam::matches(prop)) {
      if (Alembic::AbcGeom::isUV(prop)) {
        if (!attr_sel.uvs_requested()) {
          continue;
        }

        if (config.mesh) {
          if (!can_add_uv_layer(config)) {
            continue;
          }

          IV2fGeomParam param = IV2fGeomParam(desc.parent, prop.getName());
          read_mesh_uvs(config, param, sample_sel);
        }
      }
      else {
        IV2fGeomParam param = IV2fGeomParam(desc.parent, prop.getName());
        process_typed_attribute(config, CD_PROP_FLOAT2, param, sample_sel);
      }
    }
    else if (IN3fGeomParam::matches(prop)) {
      IP3fGeomParam param = IP3fGeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
    }
    else if (IN3dGeomParam::matches(prop)) {
      IP3dGeomParam param = IP3dGeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
    }
    else if (IP3fGeomParam::matches(prop)) {
      IP3fGeomParam param = IP3fGeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
    }
    else if (IP3dGeomParam::matches(prop)) {
      IP3dGeomParam param = IP3dGeomParam(desc.parent, prop.getName());
      process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
    }
    else if (IV3fGeomParam::matches(prop)) {
      IV3fGeomParam param = IV3fGeomParam(desc.parent, prop.getName());
      if (prop.getName() == propNameOriginalCoordinates) {
        if (attr_sel.original_coordinates_requested()) {
          process_typed_attribute(config, CD_ORCO, param, sample_sel);
        }
      }
      else if (prop.getName() == attr_sel.velocity_name()) {
        process_velocity_attribute(config, param, sample_sel, velocity_scale);
      }
      else {
        process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
      }
    }
    else if (IV3dGeomParam::matches(prop)) {
      IV3dGeomParam param = IV3dGeomParam(desc.parent, prop.getName());
      /* Some softaware may write ORCOs as doubles. */
      if (prop.getName() == propNameOriginalCoordinates) {
        if (attr_sel.original_coordinates_requested()) {
          process_typed_attribute(config, CD_ORCO, param, sample_sel);
        }
      }
      /* Some softaware may write velocity as doubles. */
      else if (prop.getName() == attr_sel.velocity_name()) {
        process_velocity_attribute(config, param, sample_sel, velocity_scale);
      }
      else {
        process_typed_attribute(config, CD_PROP_FLOAT3, param, sample_sel);
      }
    }
    else if (IC3fGeomParam::matches(prop)) {
      if (config.mesh) {
        if (!attr_sel.vertex_colors_requested()) {
          continue;
        }

        if (!can_add_vertex_color_layer(config)) {
          continue;
        }

        read_mesh_vertex_colors_c3f(config, desc.parent, prop, sample_sel);
      }
      else {
        IC3fGeomParam param = IC3fGeomParam(desc.parent, prop.getName());
        process_typed_attribute(config, CD_PROP_COLOR, param, sample_sel);
      }
    }
    else if (IC4fGeomParam::matches(prop)) {
      if (config.mesh) {
        if (!attr_sel.vertex_colors_requested()) {
          continue;
        }

        if (!can_add_vertex_color_layer(config)) {
          continue;
        }

        read_mesh_vertex_colors_c4f(config, desc.parent, prop, sample_sel);
      }
      else {
        IC4fGeomParam param = IC4fGeomParam(desc.parent, prop.getName());
        process_typed_attribute(config, CD_PROP_COLOR, param, sample_sel);
      }
    }
  }
}

bool has_animated_attributes(const ICompoundProperty &arb_geom_params)
{
  /* Since this function is mainly called during data creation when importing an archive, we do not
   * have an attribute selector to pass here. Also most of its settings (what to read, velocity
   * name, etc.) would not be initialized anyway. */
  Vector<ParsedAttributeDesc> attributes = parse_attributes(nullptr, arb_geom_params);

  for (ParsedAttributeDesc desc : attributes) {
    const PropertyHeader &prop = desc.prop_header;

    if (IFloatGeomParam::matches(prop)) {
      IFloatGeomParam param = IFloatGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IDoubleGeomParam::matches(prop)) {
      IDoubleGeomParam param = IDoubleGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IBoolGeomParam::matches(prop)) {
      IBoolGeomParam param = IBoolGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IInt32GeomParam::matches(prop)) {
      IInt32GeomParam param = IInt32GeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IV2fGeomParam::matches(prop)) {
      IV2fGeomParam param = IV2fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IN3fGeomParam::matches(prop)) {
      IP3fGeomParam param = IP3fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IN3dGeomParam::matches(prop)) {
      IP3dGeomParam param = IP3dGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IP3fGeomParam::matches(prop)) {
      IP3fGeomParam param = IP3fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IP3dGeomParam::matches(prop)) {
      IP3dGeomParam param = IP3dGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IV3fGeomParam::matches(prop)) {
      IV3fGeomParam param = IV3fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IV3dGeomParam::matches(prop)) {
      IV3dGeomParam param = IV3dGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IC3fGeomParam::matches(prop)) {
      IC3fGeomParam param = IC3fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
    else if (IC4fGeomParam::matches(prop)) {
      IC4fGeomParam param = IC4fGeomParam(desc.parent, prop.getName());
      if (param.valid() && !param.isConstant()) {
        return true;
      }
    }
  }

  return false;
}

void AttributeSelector::set_read_flags(int flags)
{
  read_flags = flags;
}

const CacheAttributeMapping *AttributeSelector::get_mapping(const std::string &attr_name) const
{
  for (const CacheAttributeMapping *mapping : mappings) {
    if (attr_name == mapping->name) {
      return mapping;
    }
  }

  return nullptr;
}

const std::string &AttributeSelector::velocity_name() const
{
  return velocity_attribute;
}

void AttributeSelector::set_velocity_attribute(const char *name)
{
  velocity_attribute = name;
}

bool AttributeSelector::uvs_requested() const
{
  return (read_flags & MOD_MESHSEQ_READ_UV) != 0;
}

bool AttributeSelector::vertex_colors_requested() const
{
  return (read_flags & MOD_MESHSEQ_READ_COLOR) != 0;
}

bool AttributeSelector::original_coordinates_requested() const
{
  /* There is no flag for the ORCO layer, we load it if the vertices are requested. */
  return (read_flags & MOD_MESHSEQ_READ_VERT) != 0;
}

bool AttributeSelector::select_attribute(const BlenderScope bl_scope,
                                         const std::string &attr_name) const
{
  if (bl_scope == BlenderScope::UNKNOWN) {
    return false;
  }

  /* Empty names are invalid. Those beginning with a '.' are special and correspond to data
   * accessible through specific APIs (e.g. the vertices are named ".P") */
  if (attr_name.empty() || attr_name[0] == '.') {
    return false;
  }

  /* This is loaded separately. */
  if (attr_name == velocity_attribute) {
    return false;
  }

  return true;
}

}  // namespace blender::io::alembic

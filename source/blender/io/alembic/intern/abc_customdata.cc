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
#include <optional>
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

#include "BLT_translation.h"

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
  param.setTimeSampling(config.timesample_index);

  config.abc_uv_maps[uv_map_name] = param;
}

static void get_cols(const CDStreamConfig &config,
                     std::vector<Imath::C4f> &buffer,
                     std::vector<uint32_t> &uvidx,
                     void *cd_data)
{
  const float cscale = 1.0f / 255.0f;
  MPoly *polys = config.mpoly;
  MLoop *mloops = config.mloop;
  MCol *cfaces = static_cast<MCol *>(cd_data);

  buffer.reserve(config.totvert);
  uvidx.reserve(config.totvert);

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
      uvidx.push_back(buffer.size() - 1);
    }
  }
}

/* Convention to write Vertex Colors:
 * - C3fGeomParam/C4fGeomParam on the arbGeomParam
 * - set scope as vertex varying
 */
static void write_mcol(const OCompoundProperty &prop,
                       CDStreamConfig &config,
                       void *data,
                       const char *name)
{
  std::vector<uint32_t> indices;
  std::vector<Imath::C4f> buffer;

  get_cols(config, buffer, indices, data);

  if (indices.empty() || buffer.empty()) {
    return;
  }

  std::string vcol_name(name);
  OC4fGeomParam param = config.abc_vertex_colors[vcol_name];

  if (!param.valid()) {
    param = OC4fGeomParam(prop, name, true, kFacevaryingScope, 1);
  }

  OC4fGeomParam::Sample sample(C4fArraySample(&buffer.front(), buffer.size()),
                               UInt32ArraySample(&indices.front(), indices.size()),
                               kVertexScope);

  param.set(sample);
  param.setTimeSampling(config.timesample_index);

  config.abc_vertex_colors[vcol_name] = param;
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
                                   size_t num_values,
                                   char requested_domain)
{
  if (requested_domain != CACHEFILE_ATTR_MAP_DOMAIN_AUTO) {
    if (requested_domain == CACHEFILE_ATTR_MAP_DOMAIN_POINT) {
      if (static_cast<size_t>(scope_sizes.point_scope_size * dimensions) == num_values) {
        return BlenderScope::POINT;
      }

      return BlenderScope::UNKNOWN;
    }

    if (requested_domain == CACHEFILE_ATTR_MAP_DOMAIN_FACE_CORNER) {
      if (static_cast<size_t>(scope_sizes.loop_scope_size * dimensions) == num_values) {
        return BlenderScope::LOOPS;
      }

      return BlenderScope::UNKNOWN;
    }

    if (requested_domain == CACHEFILE_ATTR_MAP_DOMAIN_FACE) {
      if (static_cast<size_t>(scope_sizes.polygon_scope_size * dimensions) == num_values) {
        return BlenderScope::POLYGON;
      }

      return BlenderScope::UNKNOWN;
    }

    return BlenderScope::UNKNOWN;
  }

  BlenderScope scope = BlenderScope::UNKNOWN;

  if (static_cast<size_t>(scope_sizes.loop_scope_size * dimensions) == num_values) {
    scope |= BlenderScope::LOOPS;
  }

  if (static_cast<size_t>(scope_sizes.polygon_scope_size * dimensions) == num_values) {
    scope |= BlenderScope::POLYGON;
  }

  if (static_cast<size_t>(scope_sizes.point_scope_size * dimensions) == num_values) {
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
      return matching_scope(scope_sizes, 1, element_size, CACHEFILE_ATTR_MAP_DOMAIN_AUTO);
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
      return matching_scope(scope_sizes, 1, element_size, CACHEFILE_ATTR_MAP_DOMAIN_AUTO);
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
  static bool map_to_bool(const T *value)
  {
    return static_cast<bool>(value[0]);
  }

  static int32_t map_to_int32(const T *value)
  {
    return static_cast<int32_t>(value[0]);
  }

  static float map_to_float(const T *value)
  {
    return static_cast<float>(value[0]);
  }

  static float2 map_to_float2(const T *value)
  {
    return {static_cast<float>(value[0]), static_cast<float>(value[1])};
  }

  static float3 map_to_float3(const T *value)
  {
    float3 result{
        static_cast<float>(value[0]), static_cast<float>(value[1]), static_cast<float>(value[2])};
    copy_zup_from_yup(result, result);
    return result;
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

template<> struct value_type_converter<Imath::V3f> {
  static float3 convert_value(const Imath::V3f &v)
  {
    return v.getValue();
  }
};

template<> struct value_type_converter<Imath::V3d> {
  static float3 convert_value(const Imath::V3d &v)
  {
    const double *ptr = v.getValue();
    return float3(
        static_cast<float>(ptr[0]), static_cast<float>(ptr[1]), static_cast<float>(ptr[2]));
  }
};

template<> struct value_type_converter<Imath::C3f> {
  static MCol convert_value(const Imath::C3f &v)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(static_cast<float>(v[0]));
    mcol.r = unit_float_to_uchar_clamp(static_cast<float>(v[1]));
    mcol.g = unit_float_to_uchar_clamp(static_cast<float>(v[2]));
    mcol.b = 255;
    return mcol;
  }
};

template<> struct value_type_converter<Imath::C4f> {
  static MCol convert_value(const Imath::C4f &v)
  {
    MCol mcol;
    mcol.a = unit_float_to_uchar_clamp(static_cast<float>(v[0]));
    mcol.r = unit_float_to_uchar_clamp(static_cast<float>(v[1]));
    mcol.g = unit_float_to_uchar_clamp(static_cast<float>(v[2]));
    mcol.b = unit_float_to_uchar_clamp(static_cast<float>(v[3]));
    return mcol;
  }
};

template<typename T> struct to_scalar_type {
  using type = T;
};

template<> struct to_scalar_type<Imath::V2f> {
  using type = float;
};

template<> struct to_scalar_type<Imath::V3f> {
  using type = float;
};

template<> struct to_scalar_type<Imath::V3d> {
  using type = double;
};

template<> struct to_scalar_type<Imath::C3f> {
  using type = float;
};

template<> struct to_scalar_type<Imath::C4f> {
  using type = float;
};

/** \} */

/* -------------------------------------------------------------------- */

static size_t mcols_out_of_bounds_check(const size_t color_index,
                                        const size_t array_size,
                                        const std::string &iobject_full_name,
                                        const std::string &prop_name,
                                        bool &r_is_out_of_bounds,
                                        bool &r_bounds_warning_given)
{
  if (color_index < array_size) {
    return color_index;
  }

  if (!r_bounds_warning_given) {
    std::cerr << "Alembic: color index out of bounds "
                 "reading face colors for object "
              << iobject_full_name << ", property " << prop_name << std::endl;
    r_bounds_warning_given = true;
  }
  r_is_out_of_bounds = true;
  return 0;
}

/* -------------------------------------------------------------------- */

/** \name AbcAttributeMapping
 *
 * Mirror of the mapping enumeration in DNA_cachefile_types.h. It is more specific and used as
 * some sort of token to indicate that the mapping is valid for the attribute.
 * \{ */

struct AbcAttributeMapping {
  CustomDataType type;
  BlenderScope scope;
  /* This is to differentiate between RGB and RGBA colors. */
  bool is_rgba = false;
};

/* Try to resolve the mapping for an attribute from the number of individual elements in the array
 * sample and the mapping as desired by the user. */
static std::optional<AbcAttributeMapping> final_mapping_from_cache_mapping(
    const CacheAttributeMapping &mapping,
    const ScopeSizeInfo scope_sizes,
    AbcAttributeMapping original_mapping,
    const uint num_elements)
{
  BLI_assert_msg(num_elements != 0,
                 "The number of elements in the array is null, although this should have been "
                 "checked before.");

  switch (mapping.mapping) {
    case CACHEFILE_ATTRIBUTE_MAP_NONE: {
      if (mapping.domain == CACHEFILE_ATTR_MAP_DOMAIN_POINT) {
        original_mapping.scope = BlenderScope::POINT;
      }
      if (mapping.domain == CACHEFILE_ATTR_MAP_DOMAIN_FACE_CORNER) {
        original_mapping.scope = BlenderScope::LOOPS;
      }
      return original_mapping;
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_UVS: {
      /* UVs have 2 values per element. */
      const BlenderScope scope_hint = matching_scope(scope_sizes, 2, num_elements, mapping.domain);
      if (!is_valid_uv_scope(scope_hint)) {
        return {};
      }
      return AbcAttributeMapping{CD_MLOOPUV, scope_hint};
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_VERTEX_COLORS: {
      /* 3 values for RGB, 4 for RGBA */
      BlenderScope rgb_scope_hint = matching_scope(scope_sizes, 3, num_elements, mapping.domain);
      BlenderScope rgba_scope_hint = matching_scope(scope_sizes, 4, num_elements, mapping.domain);

      if (is_valid_vertex_color_scope(rgb_scope_hint) &&
          rgba_scope_hint == BlenderScope::UNKNOWN) {
        return AbcAttributeMapping{CD_MCOL, rgb_scope_hint, false};
      }

      if (rgb_scope_hint == BlenderScope::UNKNOWN &&
          is_valid_vertex_color_scope(rgba_scope_hint)) {
        return AbcAttributeMapping{CD_MCOL, rgba_scope_hint, true};
      }

      /* Either both are unknown, or we matched two different scopes in which case we cannot
       * resolve the ambiguity. */
      return {};
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_WEIGHT_GROUPS: {
      BlenderScope r_scope_hint = matching_scope(scope_sizes, 1, num_elements, mapping.domain);
      if (!is_valid_vertex_group_scope(r_scope_hint)) {
        return {};
      }
      return AbcAttributeMapping{CD_BWEIGHT, r_scope_hint};
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT2: {
      BlenderScope r_scope_hint = matching_scope(scope_sizes, 2, num_elements, mapping.domain);
      if (r_scope_hint == BlenderScope::UNKNOWN) {
        return {};
      }
      return AbcAttributeMapping{CD_PROP_FLOAT2, r_scope_hint};
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT3: {
      BlenderScope r_scope_hint = matching_scope(scope_sizes, 3, num_elements, mapping.domain);
      if (r_scope_hint == BlenderScope::UNKNOWN) {
        return {};
      }
      return AbcAttributeMapping{CD_PROP_FLOAT3, r_scope_hint};
    }
    case CACHEFILE_ATTRIBUTE_MAP_TO_COLOR: {
      /* 3 values for RGB, 4 for RGBA */
      BlenderScope rgb_scope_hint = matching_scope(scope_sizes, 3, num_elements, mapping.domain);
      BlenderScope rgba_scope_hint = matching_scope(scope_sizes, 4, num_elements, mapping.domain);

      if (rgb_scope_hint != BlenderScope::UNKNOWN && rgba_scope_hint == BlenderScope::UNKNOWN) {
        return AbcAttributeMapping{CD_PROP_COLOR, rgb_scope_hint, false};
      }

      if (rgb_scope_hint == BlenderScope::UNKNOWN && rgba_scope_hint != BlenderScope::UNKNOWN) {
        return AbcAttributeMapping{CD_PROP_COLOR, rgba_scope_hint, true};
      }

      /* Either both are unknown, or we matched two different scopes in which case we cannot
       * resolve the ambiguity. */
      return {};
    }
  }

  return {};
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
  CustomDataLayer *layer = BKE_id_attribute_ensure(
      config.id, name.c_str(), cd_type, domain, nullptr);

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

enum class AbcAttributeReadError {
  /* Default value for success. */
  READ_SUCCESS,
  /* We do not support this attribute type. */
  UNSUPPORTED_TYPE,
  /* The attribute is invalid (e.g. corrupt file or data). */
  INVALID_ATTRIBUTE,
  /* We cannot determine the scope for the attribute, either from mismatching element size, or
   * ambiguous scope used in the Alembic archive. */
  SCOPE_RESOLUTION_FAILED,
  /* Scope resolution succeeded, but the scope is not valid for the data. */
  INVALID_SCOPE,
  /* The mapping selected by the user is not possible. */
  MAPPING_IMPOSSIBLE,
  /* The limit of a attribute for the CustomData layer is reached (this should only concern UVs and
   * vertex colors). */
  TOO_MANY_ATTRIBUTES,
};

static CustomDataType custom_data_type_for_pod(PlainOldDataType pod_type, uint extent)
{
  switch (pod_type) {
    case kBooleanPOD: {
      return CD_PROP_BOOL;
    }
    case kUint8POD:
    case kInt8POD:
    case kUint16POD:
    case kInt16POD:
    case kUint32POD:
    case kInt32POD: {
      return CD_PROP_INT32;
    }
    case kFloat32POD:
    case kFloat64POD: {
      if (extent == 2) {
        return CD_PROP_FLOAT2;
      }

      if (extent == 3) {
        return CD_PROP_FLOAT3;
      }

      return CD_PROP_FLOAT;
    }
    /* These are unsupported for now. */
    case kFloat16POD:
    case kUint64POD:
    case kInt64POD:
    case kStringPOD:
    case kWstringPOD:
    default: {
      /* Use this as invalid value. */
      return CD_AUTO_FROM_NAME;
    }
  }
}

template<typename TRAIT>
static CustomDataType get_default_custom_data_type(const CDStreamConfig &config,
                                                   const ITypedGeomParam<TRAIT> &param,
                                                   bool &r_is_rgba)
{
  if (std::is_same_v<ITypedGeomParam<TRAIT>, IC3fGeomParam>) {
    if (!can_add_vertex_color_layer(config)) {
      /* CD_MCOL is full, fallback to generic colors. */
      return CD_PROP_COLOR;
    }

    return CD_MCOL;
  }

  if (std::is_same_v<ITypedGeomParam<TRAIT>, IC4fGeomParam>) {
    r_is_rgba = true;
    if (!can_add_vertex_color_layer(config)) {
      /* CD_MCOL is full, fallback to generic colors. */
      return CD_PROP_COLOR;
    }

    return CD_MCOL;
  }

  if (std::is_same_v<ITypedGeomParam<TRAIT>, IV2fGeomParam>) {
    if (Alembic::AbcGeom::isUV(param.getHeader())) {
      if (can_add_uv_layer(config)) {
        return CD_MLOOPUV;
      }
    }

    /* If it is not a UV map or if CD_MLOOPUV is full, fallback to generic 2D vectors. */
    return CD_PROP_FLOAT2;
  }

  /* Ensure UVs are also detected if written as doubles. */
  if (std::is_same_v<ITypedGeomParam<TRAIT>, IV2dGeomParam>) {
    if (Alembic::AbcGeom::isUV(param.getHeader())) {
      if (can_add_uv_layer(config)) {
        return CD_MLOOPUV;
      }
    }

    /* If it is not a UV map or if CD_MLOOPUV is full, fallback to generic 2D vectors. */
    return CD_PROP_FLOAT2;
  }

  if (param.getName() == propNameOriginalCoordinates) {
    return CD_ORCO;
  }

  const DataType &data_type = param.getDataType();
  return custom_data_type_for_pod(data_type.getPod(), data_type.getExtent());
}

template<typename TRAIT>
static std::optional<AbcAttributeMapping> determine_attribute_mapping(
    const CDStreamConfig &config,
    const ITypedGeomParam<TRAIT> &param,
    size_t num_values,
    const CacheAttributeMapping *desired_mapping)
{
  const BlenderScope bl_scope = to_blender_scope(param.getScope(), num_values, config.scope_sizes);
  AbcAttributeMapping default_mapping;
  default_mapping.scope = bl_scope;
  default_mapping.type = get_default_custom_data_type(config, param, default_mapping.is_rgba);

  if (desired_mapping) {
    auto opt_final_mapping = final_mapping_from_cache_mapping(*desired_mapping,
                                                              config.scope_sizes,
                                                              default_mapping,
                                                              num_values *
                                                                  param.getDataType().getExtent());

    if (opt_final_mapping.has_value()) {
      /* Verify that the scope is valid, it may be that we cannot apply the desired mapping, or
       * that the data matches the default mapping, but that is also invalid. */
      AbcAttributeMapping final_mapping = *opt_final_mapping;
      if (final_mapping.scope != BlenderScope::UNKNOWN) {
        return opt_final_mapping;
      }
    }
  }

  if (!is_valid_scope_for_layer(default_mapping.scope, default_mapping.type)) {
    return {};
  }

  return default_mapping;
}

template<typename TRAIT>
static AbcAttributeReadError process_typed_attribute(const CDStreamConfig &config,
                                                     const CacheAttributeMapping *mapping,
                                                     const ITypedGeomParam<TRAIT> &param,
                                                     ISampleSelector iss)
{
  typename ITypedGeomParam<TRAIT>::Sample sample;
  param.getIndexed(sample, iss);

  if (!sample.valid()) {
    return AbcAttributeReadError::INVALID_ATTRIBUTE;
  }

  const AttributeSelector &attr_sel = *config.attr_selector;

  const TypedArraySample<TRAIT> &values = *sample.getVals();
  const UInt32ArraySamplePtr indices = sample.getIndices();
  const size_t num_values = indices->size() > 0 ? indices->size() : values.size();

  std::optional<AbcAttributeMapping> opt_abc_mapping = determine_attribute_mapping(
      config, param, num_values, mapping);

  if (!opt_abc_mapping.has_value()) {
    return AbcAttributeReadError::MAPPING_IMPOSSIBLE;
  }

  AbcAttributeMapping abc_mapping = *opt_abc_mapping;
  BlenderScope bl_scope = abc_mapping.scope;

  if (!attr_sel.select_attribute(param.getName())) {
    return AbcAttributeReadError::READ_SUCCESS;
  }

  using abc_type = typename TRAIT::value_type;
  using abc_scalar_type = typename to_scalar_type<abc_type>::type;

  const abc_scalar_type *input_data = reinterpret_cast<const abc_scalar_type *>(values.get());

  switch (abc_mapping.type) {
    default: {
      return AbcAttributeReadError::MAPPING_IMPOSSIBLE;
    }
    case CD_PROP_BOOL: {
      create_layer_for_scope<bool>(config, bl_scope, CD_PROP_BOOL, param.getName(), [&](size_t i) {
        return value_type_converter<abc_scalar_type>::map_to_bool(&input_data[i]);
      });
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_PROP_INT32: {
      create_layer_for_scope<int32_t>(
          config, bl_scope, CD_PROP_INT32, param.getName(), [&](size_t i) {
            return value_type_converter<abc_scalar_type>::map_to_int32(&input_data[i]);
          });
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_PROP_FLOAT: {
      create_layer_for_scope<float>(
          config, bl_scope, CD_PROP_FLOAT, param.getName(), [&](size_t i) {
            return value_type_converter<abc_scalar_type>::map_to_float(&input_data[i]);
          });
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_PROP_FLOAT2: {
      create_layer_for_scope<float2>(
          config, bl_scope, CD_PROP_FLOAT2, param.getName(), [&](size_t i) {
            return value_type_converter<abc_scalar_type>::map_to_float2(&input_data[i * 2]);
          });
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_ORCO:
    case CD_PROP_FLOAT3: {
      std::string param_name = param.getName();
      if (param.getName() == attr_sel.velocity_name()) {
        param_name = "velocity";

        /* Check that we indeed have an attribute on the points. */
        if (bl_scope != BlenderScope::POINT) {
          return AbcAttributeReadError::INVALID_SCOPE;
        }
      }

      create_layer_for_scope<float3>(
          config, bl_scope, abc_mapping.type, param_name, [&](size_t i) {
            return value_type_converter<abc_scalar_type>::map_to_float3(&input_data[i * 3]);
          });
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_PROP_COLOR: {
      if (abc_mapping.is_rgba) {
        create_layer_for_scope<ColorGeometry4f>(
            config, bl_scope, CD_PROP_COLOR, param.getName(), [&](size_t i) {
              return value_type_converter<abc_scalar_type>::map_to_color_from_rgba(
                  &input_data[i * 4]);
            });
      }
      else {
        create_layer_for_scope<ColorGeometry4f>(
            config, bl_scope, CD_PROP_COLOR, param.getName(), [&](size_t i) {
              return value_type_converter<abc_scalar_type>::map_to_color_from_rgb(
                  &input_data[i * 3]);
            });
      }
      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_MLOOPUV: {
      if (!can_add_uv_layer(config)) {
        return AbcAttributeReadError::TOO_MANY_ATTRIBUTES;
      }

      create_loop_layer_for_scope<float2>(
          config, bl_scope, CD_MLOOPUV, param.getName(), [&](size_t i) {
            i = indices->size() ? (*indices)[i] : i;
            return value_type_converter<abc_scalar_type>::map_to_float2(&input_data[i * 2]);
          });

      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_MCOL: {
      if (!can_add_vertex_color_layer(config)) {
        return AbcAttributeReadError::TOO_MANY_ATTRIBUTES;
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
          config, bl_scope, CD_MLOOPCOL, param.getName(), [&](size_t loop_index) {
            size_t color_index = loop_index;
            if (use_dual_indexing) {
              color_index = (*indices)[color_index];
            }

            bool is_mcols_out_of_bounds = false;
            color_index = mcols_out_of_bounds_check(color_index,
                                                    values.size(),
                                                    config.iobject_full_name,
                                                    param.getName(),
                                                    is_mcols_out_of_bounds,
                                                    bounds_warning_given);

            if (is_mcols_out_of_bounds) {
              return MCol{};
            }

            return value_type_converter<abc_scalar_type>::map_to_mcol_from_rgb(
                &input_data[loop_index * (3 + abc_mapping.is_rgba)]);
          });

      return AbcAttributeReadError::READ_SUCCESS;
    }
    case CD_BWEIGHT: {
      if (config.mesh) {
        Mesh *mesh = config.mesh;
        MVert *mvert = config.mvert;
        for (int i = 0; i < mesh->totvert; i++) {
          mvert[i].bweight = unit_float_to_uchar_clamp(
              value_type_converter<abc_scalar_type>::map_to_float(&input_data[i]));
        }
      }
      return AbcAttributeReadError::READ_SUCCESS;
    }
  }
  return AbcAttributeReadError::READ_SUCCESS;
}

static AbcAttributeReadError read_mesh_uvs(const CDStreamConfig &config,
                                           const IV2fGeomParam &uv_param,
                                           ISampleSelector sample_sel)
{
  BLI_assert(config.mesh);

  if (!uv_param.valid() || !uv_param.isIndexed()) {
    return AbcAttributeReadError::INVALID_ATTRIBUTE;
  }

  IV2fGeomParam::Sample sample;
  uv_param.getIndexed(sample, sample_sel);

  const Alembic::AbcGeom::V2fArraySamplePtr &uvs = sample.getVals();
  const Alembic::AbcGeom::UInt32ArraySamplePtr &indices = sample.getIndices();

  const BlenderScope bl_scope = to_blender_scope(
      uv_param.getScope(), indices->size(), config.scope_sizes);

  if (!is_valid_uv_scope(bl_scope)) {
    return AbcAttributeReadError::INVALID_SCOPE;
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

  return AbcAttributeReadError::READ_SUCCESS;
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

/* This function should be used for any attribute processing to ensure that all supported attribute
 * types are handled.
 *
 * Unsupported attribute types are:
 * - int64 and uint64, since we may have data loss as Blender only supports int32
 * - half floating points (scalar, vec2, vec3, color)
 * - matrices (3x3, 4x4)
 * - string
 * - quaternions
 * - 2d normals
 * - 2d & 3d bounding boxes
 */
template<typename OpType>
static auto abc_attribute_type_operation(const ICompoundProperty &parent_prop,
                                         const PropertyHeader &prop,
                                         OpType &&op)
{
  if (IFloatGeomParam::matches(prop)) {
    IFloatGeomParam param = IFloatGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IDoubleGeomParam::matches(prop)) {
    IDoubleGeomParam param = IDoubleGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IBoolGeomParam::matches(prop)) {
    IBoolGeomParam param = IBoolGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (ICharGeomParam::matches(prop)) {
    ICharGeomParam param = ICharGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IInt16GeomParam::matches(prop)) {
    IInt16GeomParam param = IInt16GeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IInt32GeomParam::matches(prop)) {
    IInt32GeomParam param = IInt32GeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IUcharGeomParam::matches(prop)) {
    IUcharGeomParam param = IUcharGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IUInt16GeomParam::matches(prop)) {
    IUInt16GeomParam param = IUInt16GeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IUInt32GeomParam::matches(prop)) {
    IUInt32GeomParam param = IUInt32GeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IV2fGeomParam::matches(prop)) {
    IV2fGeomParam param = IV2fGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IN3fGeomParam::matches(prop)) {
    IN3fGeomParam param = IN3fGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IN3dGeomParam::matches(prop)) {
    IN3dGeomParam param = IN3dGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IP3fGeomParam::matches(prop)) {
    IP3fGeomParam param = IP3fGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IP3dGeomParam::matches(prop)) {
    IP3dGeomParam param = IP3dGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IV3fGeomParam::matches(prop)) {
    IV3fGeomParam param = IV3fGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IV3dGeomParam::matches(prop)) {
    IV3dGeomParam param = IV3dGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IC3fGeomParam::matches(prop)) {
    IC3fGeomParam param = IC3fGeomParam(parent_prop, prop.getName());
    return op(param);
  }
  if (IC4fGeomParam::matches(prop)) {
    IC4fGeomParam param = IC4fGeomParam(parent_prop, prop.getName());
    return op(param);
  }

  return op.default_handler_for_unsupported_attribute_type(prop.getName());
}

struct AttributeReadOperator {
  const CDStreamConfig &config;
  const ISampleSelector &sample_sel;
  const AttributeSelector &attr_sel;
  float velocity_scale;

  ParsedAttributeDesc *desc;

  bool has_error = false;

  void handle_error(AbcAttributeReadError error, const std::string &attribute_name)
  {
    has_error |= error != AbcAttributeReadError::READ_SUCCESS;

    switch (error) {
      case AbcAttributeReadError::READ_SUCCESS: {
        return;
      }
      case AbcAttributeReadError::TOO_MANY_ATTRIBUTES: {
        std::cerr << "Cannot read attribute \"" << attribute_name
                  << "\", too many attributes of the same type have been read!\n";
        return;
      }
      case AbcAttributeReadError::INVALID_ATTRIBUTE: {
        std::cerr << "Cannot read attribute \"" << attribute_name << "\" as it is invalid!\n";
        return;
      }
      case AbcAttributeReadError::MAPPING_IMPOSSIBLE: {
        std::cerr << "Cannot read attribute \"" << attribute_name
                  << "\" as the mapping is impossible!\n";
        return;
      }
      case AbcAttributeReadError::SCOPE_RESOLUTION_FAILED: {
        std::cerr << "Cannot read attribute \"" << attribute_name
                  << "\" as the scope is undeterminable!\n";
        return;
      }
      case AbcAttributeReadError::INVALID_SCOPE: {
        std::cerr << "Cannot read attribute \"" << attribute_name
                  << "\" as the scope is invalid for the specific data type!\n";
        return;
      }
      case AbcAttributeReadError::UNSUPPORTED_TYPE: {
        std::cerr << "Cannot read attribute \"" << attribute_name << "\""
                  << " as the data type is not supported!\n";
        return;
      }
    }
  }

  void default_handler_for_unsupported_attribute_type(const std::string &param_name)
  {
    handle_error(AbcAttributeReadError::UNSUPPORTED_TYPE, param_name);
  }

  template<typename GeomParamType> void operator()(const GeomParamType &param)
  {
    AbcAttributeReadError error = process_typed_attribute(
        config, desc->mapping, param, sample_sel);
    handle_error(error, desc->prop_header.getName());
  }
};

void read_arbitrary_attributes(const CDStreamConfig &config,
                               const ICompoundProperty &schema,
                               const IV2fGeomParam &primary_uvs,
                               const ISampleSelector &sample_sel,
                               float velocity_scale)
{

  /* The attribute selector may be null when run through the import operator. */
  if (!config.attr_selector) {
    return;
  }

  /* Manually extract the arbitrary geometry parameters. We do it this way to avoid complicating
   * the code when dealing with schemas and default velocities which are not accessible via an
   * IGeomParam as we would like for the sake of genericity, but as an IArrayProperty. */
  const ICompoundProperty &arb_geom_params = ICompoundProperty(schema, ".arbGeomParams");
  if (!arb_geom_params.valid()) {
    return;
  }

  const AttributeSelector &attr_sel = *config.attr_selector;

  Vector<ParsedAttributeDesc> attributes = parse_attributes(config.attr_selector, arb_geom_params);

  /* At this point the velocities attribute should have the default, standard, attribute velocity
   * name in Alembic. So we also expect any remapping to also use this name. If there is no
   * attribute with the standard name, then either there are no velocities, or it has a different
   * name which should have been set in the UI (both for the selection and the remapping). */
  const PropertyHeader *velocity_prop_header = schema.getPropertyHeader(".velocities");
  if (velocity_prop_header) {
    attributes.append({schema, *velocity_prop_header, attr_sel.get_mapping(".velocities")});
  }

  if (primary_uvs.valid() && attr_sel.uvs_requested()) {
    read_mesh_uvs(config, primary_uvs, sample_sel);
  }

  AttributeReadOperator op{config, sample_sel, attr_sel, velocity_scale, nullptr};

  for (ParsedAttributeDesc desc : attributes) {
    op.desc = &desc;
    abc_attribute_type_operation(desc.parent, desc.prop_header, op);
  }

  if (op.has_error && config.modifier_error_message) {
    *config.modifier_error_message = N_(
        "Errors while trying to read attributes, see console for details...");
  }
}

struct AnimatedAttributeOperator {
  bool default_handler_for_unsupported_attribute_type(const std::string & /*param_name*/)
  {
    return false;
  }

  template<typename ParamType> bool operator()(const ParamType &param)
  {
    return param.valid() && !param.isConstant();
  }
};

bool has_animated_attributes(const ICompoundProperty &arb_geom_params)
{
  /* Since this function is mainly called during data creation when importing an archive, we do not
   * have an attribute selector to pass here. Also most of its settings (what to read, velocity
   * name, etc.) would not be initialized anyway. */
  Vector<ParsedAttributeDesc> attributes = parse_attributes(nullptr, arb_geom_params);

  AnimatedAttributeOperator animated_param_op;
  for (ParsedAttributeDesc desc : attributes) {
    const PropertyHeader &prop = desc.prop_header;
    if (abc_attribute_type_operation(desc.parent, prop, animated_param_op)) {
      return true;
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

bool AttributeSelector::select_attribute(const std::string &attr_name) const
{
  /* Empty names are invalid. Those beginning with a '.' are special and correspond to data
   * accessible through specific APIs (e.g. the vertices are named ".P") */
  if (attr_name.empty() || (attr_name[0] == '.' && attr_name != velocity_attribute)) {
    return false;
  }

  return true;
}

}  // namespace blender::io::alembic

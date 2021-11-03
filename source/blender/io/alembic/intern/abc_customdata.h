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
#pragma once

/** \file
 * \ingroup balembic
 */

#include <Alembic/Abc/All.h>
#include <Alembic/AbcGeom/All.h>

#include <map>

#include "BLI_listbase_wrapper.hh"

struct CacheAttributeMapping;
struct CustomData;
struct ID;
struct MLoop;
struct MLoopUV;
struct MPoly;
struct MVert;
struct Mesh;

using Alembic::Abc::ICompoundProperty;
using Alembic::Abc::OCompoundProperty;
namespace blender::io::alembic {

class AttributeSelector;
enum class BlenderScope : uint32_t;

struct UVSample {
  std::vector<Imath::V2f> uvs;
  std::vector<uint32_t> indices;
};

struct ScopeSizeInfo {
  int point_scope_size;
  int polygon_scope_size;
  int loop_scope_size;
};

struct ScopeCustomDataPointers {
  CustomData *point_custom_data;
  CustomData *polygon_custom_data;
  CustomData *loop_custom_data;
};

struct CDStreamConfig {
  MLoop *mloop;
  int totloop;

  MPoly *mpoly;
  int totpoly;

  MVert *mvert;
  int totvert;

  MLoopUV *mloopuv;

  CustomData *loopdata;

  bool pack_uvs;

  /* NOTE: the mesh is mostly used for iterating over loops for loop attributes (UVs, MCol, etc.).
   * It would be nice to remove it, in favor of a more generic way to iterate valid attribute
   * indices.
   */
  Mesh *mesh;
  ID *id;

  float weight;
  float time;
  bool use_vertex_interpolation;
  Alembic::AbcGeom::index_t index;
  Alembic::AbcGeom::index_t ceil_index;

  const char **modifier_error_message;

  ScopeCustomDataPointers custom_data_pointers;
  ScopeSizeInfo scope_sizes;

  /* For error reporting when reading vertex colors. */
  std::string iobject_full_name;

  const AttributeSelector *attr_selector;

  /* Alembic needs Blender to keep references to C++ objects (the destructors finalize the writing
   * to ABC). The following fields are all used to keep these references. */

  /* Mapping from UV map name to its ABC property, for the 2nd and subsequent UV maps; the primary
   * UV map is kept alive by the Alembic mesh sample itself. */
  std::map<std::string, Alembic::AbcGeom::OV2fGeomParam> abc_uv_maps;

  /* ORCO coordinates, aka Generated Coordinates. */
  Alembic::AbcGeom::OV3fGeomParam abc_orco;

  CDStreamConfig()
      : mloop(NULL),
        totloop(0),
        mpoly(NULL),
        totpoly(0),
        totvert(0),
        pack_uvs(false),
        mesh(NULL),
        weight(0.0f),
        time(0.0f),
        index(0),
        ceil_index(0),
        modifier_error_message(NULL),
        attr_selector(nullptr)
  {
  }
};

/* Get the UVs for the main UV property on a OSchema.
 * Returns the name of the UV layer.
 *
 * For now the active layer is used, maybe needs a better way to choose this. */
const char *get_uv_sample(UVSample &sample, const CDStreamConfig &config, CustomData *data);

void write_generated_coordinates(const OCompoundProperty &prop, CDStreamConfig &config);

void write_custom_data(const OCompoundProperty &prop,
                       CDStreamConfig &config,
                       CustomData *data,
                       int data_type);

class AttributeSelector {
  /* Name of the velocity attribute, it is ignored since we deal with separately. */
  std::string velocity_attribute = "";

  int read_flags = 0;

  ListBaseWrapper<const CacheAttributeMapping> mappings;

 public:
  AttributeSelector(ListBase *mappings_) : mappings(mappings_)
  {
  }

  void set_velocity_attribute(const char *name);

  void set_read_flags(int flags);

  const CacheAttributeMapping *get_mapping(const std::string &attr_name) const;

  const std::string &velocity_name() const;

  bool uvs_requested() const;

  bool vertex_colors_requested() const;

  bool original_coordinates_requested() const;

  bool select_attribute(const std::string &attr_name) const;
};

void read_arbitrary_attributes(const CDStreamConfig &config,
                               const ICompoundProperty &arb_geom_params,
                               const Alembic::AbcGeom::v12::IV2fGeomParam &primary_uvs,
                               const Alembic::Abc::ISampleSelector &sample_sel,
                               float velocity_scale);

bool has_animated_attributes(const ICompoundProperty &arb_geom_params);

}  // namespace blender::io::alembic

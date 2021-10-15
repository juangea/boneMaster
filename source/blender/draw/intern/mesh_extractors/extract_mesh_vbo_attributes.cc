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
 * The Original Code is Copyright (C) 2021 by Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup draw
 */

#include "MEM_guardedalloc.h"

#include "BLI_float2.hh"
#include "BLI_float3.hh"
#include "BLI_float4.hh"
#include "BLI_string.h"

#include "extract_mesh.h"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Attributes
 * \{ */

/* Utility to convert from the type used in the attributes to the types for the VBO.
 * This is mostly used to promote integers and booleans to floats, as other types (float, float2,
 * etc.) directly map to avalaible GPU type. Integers are still converted as attributes are vec4 in
 * the shader.
 */
template<typename AttributeType, typename VBOType> struct attribute_type_converter {
  static VBOType convert_value(AttributeType value)
  {
    if constexpr (std::is_same_v<AttributeType, VBOType>) {
      return value;
    }

    /* This should only concern int and bool which are converted to floats. */
    return static_cast<VBOType>(value);
  }
};

/* Return the number of component for the attribute's value type, or 0 if is it unsupported. */
static uint gpu_component_size_for_attribute_type(CustomDataType type)
{
  switch (type) {
    case CD_PROP_BOOL:
    case CD_PROP_INT32:
    case CD_PROP_FLOAT: {
      return 1;
    }
    case CD_PROP_FLOAT2: {
      return 2;
    }
    case CD_PROP_FLOAT3: {
      return 3;
    }
    case CD_PROP_COLOR: {
      return 4;
    }
    default: {
      return 0;
    }
  }
}

static void init_format_for_attr(GPUVertFormat &format,
                                 CustomData *custom_data,
                                 CustomDataType type,
                                 uint32_t layers_used)
{
  GPUVertCompType comp_type = GPU_COMP_F32;
  const uint comp_size = gpu_component_size_for_attribute_type(type);
  /* We should not be here if the attribute type is not supported. */
  BLI_assert(comp_size != 0);

  for (int i = 0; i < 8; i++) {
    if (layers_used & (1 << i)) {
      char attr_name[32], attr_safe_name[GPU_MAX_SAFE_ATTR_NAME];
      const char *layer_name = CustomData_get_layer_name(custom_data, type, i);
      GPU_vertformat_safe_attr_name(layer_name, attr_safe_name, GPU_MAX_SAFE_ATTR_NAME);
      /* Attributes use auto-name. */
      BLI_snprintf(attr_name, sizeof(attr_name), "a%s", attr_safe_name);

      GPU_vertformat_attr_add(&format, attr_name, comp_type, comp_size, GPU_FETCH_FLOAT);
      GPU_vertformat_alias_add(&format, attr_name);
    }
  }
}

static void init_vbo_for_attribute(GPUVertBuf *vbo,
                                   CustomData *custom_data,
                                   CustomDataType type,
                                   uint32_t layers_used,
                                   int vert_len)
{
  GPUVertFormat format = {0};
  GPU_vertformat_deinterleave(&format);

  init_format_for_attr(format, custom_data, type, layers_used);

  GPU_vertbuf_init_with_format(vbo, &format);
  GPU_vertbuf_data_alloc(vbo, static_cast<uint32_t>(vert_len));
}

template<typename AttributeType, typename VBOType>
static void fill_vertbuf_with_attribute(const MeshRenderData *mr,
                                        GPUVertBuf *vbo,
                                        CustomData *custom_data,
                                        CustomDataType cd_type,
                                        MLoop *loops,
                                        int layer_index)
{
  VBOType *vbo_data = static_cast<VBOType *>(GPU_vertbuf_get_data(vbo));
  const AttributeType *attr_data = static_cast<AttributeType *>(
      CustomData_get_layer_n(custom_data, cd_type, layer_index));

  using converter = attribute_type_converter<AttributeType, VBOType>;

  // TODO(kevindietrich) : domains
  for (int ml_index = 0; ml_index < mr->loop_len; ml_index++, vbo_data++, loops++) {
    *vbo_data = converter::convert_value(attr_data[loops->v]);
  }
}

template<typename AttributeType, typename VBOType>
static void fill_vertbuf_with_attribute_bm(const MeshRenderData *mr,
                                           GPUVertBuf *vbo,
                                           CustomData *custom_data,
                                           CustomDataType cd_type,
                                           int layer_index)
{
  VBOType *vbo_data = static_cast<VBOType *>(GPU_vertbuf_get_data(vbo));
  int cd_ofs = CustomData_get_n_offset(custom_data, cd_type, layer_index);

  using converter = attribute_type_converter<AttributeType, VBOType>;

  // TODO(kevindietrich) : domains
  BMIter f_iter;
  BMFace *efa;
  BM_ITER_MESH (efa, &f_iter, mr->bm, BM_FACES_OF_MESH) {
    BMLoop *l_iter, *l_first;
    l_iter = l_first = BM_FACE_FIRST_LOOP(efa);
    do {
      const AttributeType *attr_data = (const AttributeType *)BM_ELEM_CD_GET_VOID_P(l_iter->v,
                                                                                    cd_ofs);
      *vbo_data = converter::convert_value(*attr_data);
      vbo_data++;
    } while ((l_iter = l_iter->next) != l_first);
  }
}

template<typename AttributeType, typename VBOType = AttributeType>
static void extract_attr_generic(const MeshRenderData *mr,
                                 GPUVertBuf *vbo,
                                 CustomData *custom_data,
                                 CustomDataType cd_type,
                                 uint32_t layers_used)
{
  CustomData *cd_ldata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->ldata : &mr->me->ldata;
  init_vbo_for_attribute(vbo, custom_data, cd_type, layers_used, mr->loop_len);

  for (int i = 0; i < MAX_MCOL; i++) {
    if (layers_used & (1 << i)) {
      if (mr->extract_type == MR_EXTRACT_BMESH) {
        fill_vertbuf_with_attribute_bm<AttributeType, VBOType>(mr, vbo, custom_data, cd_type, i);
      }
      else {
        MLoop *loops = static_cast<MLoop *>(CustomData_get_layer(cd_ldata, CD_MLOOP));
        fill_vertbuf_with_attribute<AttributeType, VBOType>(
            mr, vbo, custom_data, cd_type, loops, i);
      }
    }
  }
}

static void extract_attr_bool_init(const MeshRenderData *mr,
                                   struct MeshBatchCache *cache,
                                   void *buf,
                                   void *UNUSED(tls_data))
{
  CustomData *cd_vdata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
  uint32_t layers_used = cache->cd_used.attr_float;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<bool, float>(mr, vbo, cd_vdata, CD_PROP_FLOAT, layers_used);
}

static void extract_attr_int32_init(const MeshRenderData *mr,
                                    struct MeshBatchCache *cache,
                                    void *buf,
                                    void *UNUSED(tls_data))
{
  CustomData *cd_vdata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
  uint32_t layers_used = cache->cd_used.attr_float;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<int32_t, float>(mr, vbo, cd_vdata, CD_PROP_FLOAT, layers_used);
}

static void extract_attr_float_init(const MeshRenderData *mr,
                                    struct MeshBatchCache *cache,
                                    void *buf,
                                    void *UNUSED(tls_data))
{
  CustomData *cd_vdata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
  uint32_t layers_used = cache->cd_used.attr_float;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float>(mr, vbo, cd_vdata, CD_PROP_FLOAT, layers_used);
}

static void extract_attr_float2_init(const MeshRenderData *mr,
                                     struct MeshBatchCache *cache,
                                     void *buf,
                                     void *UNUSED(tls_data))
{
  CustomData *cd_vdata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
  uint32_t layers_used = cache->cd_used.attr_float2;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float2>(mr, vbo, cd_vdata, CD_PROP_FLOAT2, layers_used);
}

static void extract_attr_float3_init(const MeshRenderData *mr,
                                     struct MeshBatchCache *cache,
                                     void *buf,
                                     void *UNUSED(tls_data))
{
  CustomData *cd_vdata = (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
  uint32_t layers_used = cache->cd_used.attr_float3;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float3>(mr, vbo, cd_vdata, CD_PROP_FLOAT3, layers_used);
}

constexpr MeshExtract create_extractor_attr_bool()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_bool_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_float);
  return extractor;
}

constexpr MeshExtract create_extractor_attr_int32()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_int32_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_float);
  return extractor;
}

constexpr MeshExtract create_extractor_attr_float()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_float_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_float);
  return extractor;
}

constexpr MeshExtract create_extractor_attr_float2()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_float2_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_float2);
  return extractor;
}

constexpr MeshExtract create_extractor_attr_float3()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_float3_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_float3);
  return extractor;
}

/** \} */

}  // namespace blender::draw

extern "C" {
const MeshExtract extract_attr_bool = blender::draw::create_extractor_attr_bool();
const MeshExtract extract_attr_int32 = blender::draw::create_extractor_attr_int32();
const MeshExtract extract_attr_float = blender::draw::create_extractor_attr_float();
const MeshExtract extract_attr_float2 = blender::draw::create_extractor_attr_float2();
const MeshExtract extract_attr_float3 = blender::draw::create_extractor_attr_float3();
}

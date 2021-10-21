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

#include <functional>

#include "BLI_float2.hh"
#include "BLI_float3.hh"
#include "BLI_float4.hh"
#include "BLI_string.h"

#include "BKE_attribute.h"

#include "extract_mesh.h"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Attributes
 * \{ */

static CustomData *get_custom_data_for_domain(const MeshRenderData *mr, AttributeDomain domain)
{
  switch (domain) {
    default: {
      return nullptr;
    }
    case ATTR_DOMAIN_POINT: {
      return (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->vdata : &mr->me->vdata;
    }
    case ATTR_DOMAIN_CORNER: {
      return (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->ldata : &mr->me->ldata;
    }
    case ATTR_DOMAIN_FACE: {
      return (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->pdata : &mr->me->pdata;
    }
    case ATTR_DOMAIN_EDGE: {
      return (mr->extract_type == MR_EXTRACT_BMESH) ? &mr->bm->edata : &mr->me->edata;
    }
  }
}

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

struct gpuMeshCol {
  ushort r, g, b, a;
};

template<> struct attribute_type_converter<MPropCol, gpuMeshCol> {
  static gpuMeshCol convert_value(MPropCol value)
  {
    gpuMeshCol result;
    result.r = unit_float_to_ushort_clamp(value.color[0]);
    result.g = unit_float_to_ushort_clamp(value.color[1]);
    result.b = unit_float_to_ushort_clamp(value.color[2]);
    result.a = unit_float_to_ushort_clamp(value.color[3]);
    return result;
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

static void init_format_for_attr(const MeshRenderData *mr,
                                 GPUVertFormat &format,
                                 CustomDataType type,
                                 const DRW_AttributeRequestsList *requests)
{
  GPUVertCompType comp_type =
      GPU_COMP_F32;  // type == CD_PROP_COLOR ? GPU_COMP_U16 : GPU_COMP_F32;
  const uint comp_size = gpu_component_size_for_attribute_type(type);
  /* We should not be here if the attribute type is not supported. */
  BLI_assert(comp_size != 0);

  for (int i = 0; i < requests->num_requests; i++) {
    const DRW_AttributeRequest req = requests->requests[i];
    const AttributeDomain domain = static_cast<AttributeDomain>(req.domain);
    const CustomData *custom_data = get_custom_data_for_domain(mr, domain);
    char attr_name[32], attr_safe_name[GPU_MAX_SAFE_ATTR_NAME];
    const char *layer_name = CustomData_get_layer_name(custom_data, type, req.layer_index);
    GPU_vertformat_safe_attr_name(layer_name, attr_safe_name, GPU_MAX_SAFE_ATTR_NAME);
    /* Attributes use auto-name. */
    BLI_snprintf(attr_name, sizeof(attr_name), "a%s", attr_safe_name);

    GPU_vertformat_attr_add(&format, attr_name, comp_type, comp_size, GPU_FETCH_FLOAT);
    GPU_vertformat_alias_add(&format, attr_name);
  }
}

static void init_vbo_for_attribute(const MeshRenderData *mr,
                                   GPUVertBuf *vbo,
                                   CustomDataType type,
                                   const DRW_AttributeRequestsList *requests,
                                   int vert_len)
{
  GPUVertFormat format = {0};
  GPU_vertformat_deinterleave(&format);
  init_format_for_attr(mr, format, type, requests);
  GPU_vertbuf_init_with_format(vbo, &format);
  GPU_vertbuf_data_alloc(vbo, static_cast<uint32_t>(vert_len));
}

static void init_vbo_for_attribute(const MeshRenderData *mr,
                                   GPUVertBuf *vbo,
                                   const DRW_AttributeRequest *request)
{
  const DRW_AttributeRequest req = *request;
  GPUVertCompType comp_type =
      GPU_COMP_F32;  // type == CD_PROP_COLOR ? GPU_COMP_U16 : GPU_COMP_F32;
  const uint comp_size = gpu_component_size_for_attribute_type(
      static_cast<CustomDataType>(req.cd_type));
  /* We should not be here if the attribute type is not supported. */
  BLI_assert(comp_size != 0);

  GPUVertFormat format = {0};
  GPU_vertformat_deinterleave(&format);

  const AttributeDomain domain = static_cast<AttributeDomain>(req.domain);
  const CustomData *custom_data = get_custom_data_for_domain(mr, domain);
  char attr_name[32], attr_safe_name[GPU_MAX_SAFE_ATTR_NAME];
  const char *layer_name = CustomData_get_layer_name(custom_data, req.cd_type, req.layer_index);
  GPU_vertformat_safe_attr_name(layer_name, attr_safe_name, GPU_MAX_SAFE_ATTR_NAME);
  /* Attributes use auto-name. */
  BLI_snprintf(attr_name, sizeof(attr_name), "a%s", attr_safe_name);

  GPU_vertformat_attr_add(&format, attr_name, comp_type, comp_size, GPU_FETCH_FLOAT);
  GPU_vertformat_alias_add(&format, attr_name);

  GPU_vertbuf_init_with_format(vbo, &format);
  GPU_vertbuf_data_alloc(vbo, static_cast<uint32_t>(mr->loop_len));
}

template<typename AttributeType, typename VBOType>
static void fill_vertbuf_with_attribute(const MeshRenderData *mr,
                                        VBOType *&vbo_data,
                                        CustomDataType cd_type,
                                        const DRW_AttributeRequest req)
{
  const AttributeDomain domain = static_cast<AttributeDomain>(req.domain);
  const CustomData *custom_data = get_custom_data_for_domain(mr, domain);
  BLI_assert(custom_data);
  const int layer_index = req.layer_index;

  const MPoly *mpoly = mr->mpoly;
  const MLoop *mloop = mr->mloop;

  const AttributeType *attr_data = static_cast<AttributeType *>(
      CustomData_get_layer_n(custom_data, cd_type, layer_index));

  using converter = attribute_type_converter<AttributeType, VBOType>;

  switch (domain) {
    default: {
      BLI_assert(false);
      break;
    }
    case ATTR_DOMAIN_POINT: {
      for (int ml_index = 0; ml_index < mr->loop_len; ml_index++, vbo_data++, mloop++) {
        *vbo_data = converter::convert_value(attr_data[mloop->v]);
      }
      break;
    }
    case ATTR_DOMAIN_CORNER: {
      for (int ml_index = 0; ml_index < mr->loop_len; ml_index++, vbo_data++) {
        *vbo_data = converter::convert_value(attr_data[ml_index]);
      }
      break;
    }
    case ATTR_DOMAIN_EDGE: {
      for (int ml_index = 0; ml_index < mr->loop_len; ml_index++, vbo_data++, mloop++) {
        *vbo_data = converter::convert_value(attr_data[mloop->e]);
      }
      break;
    }
    case ATTR_DOMAIN_FACE: {
      for (int mp_index = 0; mp_index < mr->poly_len; mp_index++) {
        const MPoly &poly = mpoly[mp_index];
        const VBOType value = converter::convert_value(attr_data[mp_index]);
        for (int l = 0; l < poly.totloop; l++) {
          *vbo_data++ = value;
        }
      }
      break;
    }
  }
}

template<typename AttributeType, typename VBOType>
static void fill_vertbuf_with_attribute_bm(const MeshRenderData *mr,
                                           VBOType *&vbo_data,
                                           CustomDataType cd_type,
                                           const DRW_AttributeRequest req)
{
  const AttributeDomain domain = static_cast<AttributeDomain>(req.domain);
  const CustomData *custom_data = get_custom_data_for_domain(mr, domain);
  BLI_assert(custom_data);
  const int layer_index = req.layer_index;

  int cd_ofs = CustomData_get_n_offset(custom_data, cd_type, layer_index);

  using converter = attribute_type_converter<AttributeType, VBOType>;

  BMIter f_iter;
  BMFace *efa;
  BM_ITER_MESH (efa, &f_iter, mr->bm, BM_FACES_OF_MESH) {
    BMLoop *l_iter, *l_first;
    l_iter = l_first = BM_FACE_FIRST_LOOP(efa);
    do {
      const AttributeType *attr_data = nullptr;
      if (domain == ATTR_DOMAIN_POINT) {
        attr_data = static_cast<const AttributeType *>(BM_ELEM_CD_GET_VOID_P(l_iter->v, cd_ofs));
      }
      else if (domain == ATTR_DOMAIN_CORNER) {
        attr_data = static_cast<const AttributeType *>(BM_ELEM_CD_GET_VOID_P(l_iter, cd_ofs));
      }
      else if (domain == ATTR_DOMAIN_FACE) {
        attr_data = static_cast<const AttributeType *>(BM_ELEM_CD_GET_VOID_P(efa, cd_ofs));
      }
      else if (domain == ATTR_DOMAIN_EDGE) {
        attr_data = static_cast<const AttributeType *>(BM_ELEM_CD_GET_VOID_P(l_iter->e, cd_ofs));
      }
      else {
        BLI_assert(false);
        continue;
      }
      *vbo_data = converter::convert_value(*attr_data);
      vbo_data++;
    } while ((l_iter = l_iter->next) != l_first);
  }
}

#ifdef SPLIT_ATTRIBUTE_VBO_BY_TYPE
template<typename AttributeType, typename VBOType = AttributeType>
static void extract_attr_generic(const MeshRenderData *mr,
                                 GPUVertBuf *vbo,
                                 CustomDataType cd_type,
                                 const DRW_AttributeRequestsList *requests)
{
  init_vbo_for_attribute(mr, vbo, cd_type, requests, mr->loop_len);

  VBOType *vbo_data = static_cast<VBOType *>(GPU_vertbuf_get_data(vbo));

  for (int i = 0; i < requests->num_requests; i++) {
    const DRW_AttributeRequest req = requests->requests[i];
    if (mr->extract_type == MR_EXTRACT_BMESH) {
      fill_vertbuf_with_attribute_bm<AttributeType>(mr, vbo_data, cd_type, req);
    }
    else {
      fill_vertbuf_with_attribute<AttributeType>(mr, vbo_data, cd_type, req);
    }
  }
}
#else
template<typename AttributeType, typename VBOType = AttributeType>
static void extract_attr_generic(const MeshRenderData *mr,
                                 GPUVertBuf *vbo,
                                 const DRW_AttributeRequest *request)
{
  VBOType *vbo_data = static_cast<VBOType *>(GPU_vertbuf_get_data(vbo));

  const DRW_AttributeRequest req = *request;
  if (mr->extract_type == MR_EXTRACT_BMESH) {
    fill_vertbuf_with_attribute_bm<AttributeType>(
        mr, vbo_data, static_cast<CustomDataType>(req.cd_type), req);
  }
  else {
    fill_vertbuf_with_attribute<AttributeType>(
        mr, vbo_data, static_cast<CustomDataType>(req.cd_type), req);
  }
}
#endif

#ifdef SPLIT_ATTRIBUTE_VBO_BY_TYPE
static void extract_attr_bool_init(const MeshRenderData *mr,
                                   struct MeshBatchCache *cache,
                                   void *buf,
                                   void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.bool_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<bool, float>(mr, vbo, CD_PROP_BOOL, attrs_used);
}

static void extract_attr_int32_init(const MeshRenderData *mr,
                                    struct MeshBatchCache *cache,
                                    void *buf,
                                    void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.int32_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<int32_t, float>(mr, vbo, CD_PROP_INT32, attrs_used);
}

static void extract_attr_float_init(const MeshRenderData *mr,
                                    struct MeshBatchCache *cache,
                                    void *buf,
                                    void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.float_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float>(mr, vbo, CD_PROP_FLOAT, attrs_used);
}

static void extract_attr_float2_init(const MeshRenderData *mr,
                                     struct MeshBatchCache *cache,
                                     void *buf,
                                     void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.float2_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float2>(mr, vbo, CD_PROP_FLOAT2, attrs_used);
}

static void extract_attr_float3_init(const MeshRenderData *mr,
                                     struct MeshBatchCache *cache,
                                     void *buf,
                                     void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.float3_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<float3>(mr, vbo, CD_PROP_FLOAT3, attrs_used);
}

static void extract_attr_color_init(const MeshRenderData *mr,
                                    struct MeshBatchCache *cache,
                                    void *buf,
                                    void *UNUSED(tls_data))
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.color_attributes;
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  extract_attr_generic<MPropCol>(mr, vbo, CD_PROP_COLOR, attrs_used);
}

constexpr MeshExtract create_extractor_attr_bool()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_bool_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_bool);
  return extractor;
}

constexpr MeshExtract create_extractor_attr_int32()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_int32_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_int32);
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

constexpr MeshExtract create_extractor_attr_color()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_attr_color_init;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr_color);
  return extractor;
}
#else

static void extract_attr_init(const MeshRenderData *mr,
                              struct MeshBatchCache *cache,
                              void *buf,
                              void *UNUSED(tls_data),
                              int index)
{
  const DRW_AttributeRequestsList *attrs_used = &cache->attr_used.list;
  const DRW_AttributeRequest *request = &attrs_used->requests[index];

  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);

  init_vbo_for_attribute(mr, vbo, request);

  switch (request->cd_type) {
    case CD_PROP_BOOL: {
      extract_attr_generic<bool, float>(mr, vbo, request);
      break;
    }
    case CD_PROP_INT32: {
      extract_attr_generic<int32_t, float>(mr, vbo, request);
      break;
    }
    case CD_PROP_FLOAT: {
      extract_attr_generic<float>(mr, vbo, request);
      break;
    }
    case CD_PROP_FLOAT2: {
      extract_attr_generic<float2>(mr, vbo, request);
      break;
    }
    case CD_PROP_FLOAT3: {
      extract_attr_generic<float3>(mr, vbo, request);
      break;
    }
    case CD_PROP_COLOR: {
      extract_attr_generic<MPropCol>(mr, vbo, request);
      break;
    }
    default: {
      BLI_assert(false);
    }
  }
}

/* Wrappers around extract_attr_init so we can pass the index of the attribute that we want to
 * extract. The overall API does not allow us to pass this in a convenient way. */
#  define EXTRACT_INIT_WRAPPER(index) \
    static void extract_attr_init##index( \
        const MeshRenderData *mr, struct MeshBatchCache *cache, void *buf, void *tls_data) \
    { \
      extract_attr_init(mr, cache, buf, tls_data, index); \
    }

EXTRACT_INIT_WRAPPER(0)
EXTRACT_INIT_WRAPPER(1)
EXTRACT_INIT_WRAPPER(2)
EXTRACT_INIT_WRAPPER(3)
EXTRACT_INIT_WRAPPER(4)
EXTRACT_INIT_WRAPPER(5)
EXTRACT_INIT_WRAPPER(6)
EXTRACT_INIT_WRAPPER(7)
EXTRACT_INIT_WRAPPER(8)
EXTRACT_INIT_WRAPPER(9)
EXTRACT_INIT_WRAPPER(10)
EXTRACT_INIT_WRAPPER(11)
EXTRACT_INIT_WRAPPER(12)
EXTRACT_INIT_WRAPPER(13)
EXTRACT_INIT_WRAPPER(14)

constexpr MeshExtract create_extractor_attr(int index, ExtractInitFn fn)
{
  MeshExtract extractor = {nullptr};
  extractor.init = fn;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.attr[index]);
  return extractor;
}
#endif

/** \} */

}  // namespace blender::draw

extern "C" {
#ifdef SPLIT_ATTRIBUTE_VBO_BY_TYPE
const MeshExtract extract_attr_bool = blender::draw::create_extractor_attr_bool();
const MeshExtract extract_attr_int32 = blender::draw::create_extractor_attr_int32();
const MeshExtract extract_attr_float = blender::draw::create_extractor_attr_float();
const MeshExtract extract_attr_float2 = blender::draw::create_extractor_attr_float2();
const MeshExtract extract_attr_float3 = blender::draw::create_extractor_attr_float3();
const MeshExtract extract_attr_color = blender::draw::create_extractor_attr_color();
#else

#  define EXTRACT_ATTR_PARAMETERS(index) index, blender::draw::extract_attr_init##index

const MeshExtract extract_attr[GPU_MAX_ATTR] = {
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(0)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(1)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(2)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(3)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(4)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(5)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(6)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(7)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(8)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(9)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(10)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(11)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(12)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(13)),
    blender::draw::create_extractor_attr(EXTRACT_ATTR_PARAMETERS(14)),
};
#endif
}

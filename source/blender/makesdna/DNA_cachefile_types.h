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
 * The Original Code is Copyright (C) 2016 Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "DNA_ID.h"

#ifdef __cplusplus
extern "C" {
#endif

struct GSet;

/* CacheFile::type */
typedef enum {
  CACHEFILE_TYPE_ALEMBIC = 1,
  CACHEFILE_TYPE_USD = 2,
  CACHE_FILE_TYPE_INVALID = 0,
} eCacheFileType;

/* CacheFile::flag */
enum {
  CACHEFILE_DS_EXPAND = (1 << 0),
  CACHEFILE_UNUSED_0 = (1 << 1),
};

#if 0 /* UNUSED */
/* CacheFile::draw_flag */
enum {
  CACHEFILE_KEYFRAME_DRAWN = (1 << 0),
};
#endif

/* Representation of an object's path inside the archive.
 * Note that this is not a file path. */
typedef struct CacheObjectPath {
  struct CacheObjectPath *next, *prev;

  char path[4096];
} CacheObjectPath;

/* CacheFileLayer::flag */
enum { CACHEFILE_LAYER_HIDDEN = (1 << 0) };

typedef struct CacheFileLayer {
  struct CacheFileLayer *next, *prev;

  /** 1024 = FILE_MAX. */
  char filepath[1024];
  int flag;
  int _pad;
} CacheFileLayer;

/* CacheAttributeMapping::mapping */
enum {
  /* Default mapping, so we do not make an arbitrary decision as to what is the default. Also used
   * to mark the mapping as ignored. */
  CACHEFILE_ATTRIBUTE_MAP_NONE,
  CACHEFILE_ATTRIBUTE_MAP_TO_UVS,
  CACHEFILE_ATTRIBUTE_MAP_TO_VERTEX_COLORS,
  CACHEFILE_ATTRIBUTE_MAP_TO_WEIGHT_GROUPS,
  CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT2,
  CACHEFILE_ATTRIBUTE_MAP_TO_FLOAT3,
  CACHEFILE_ATTRIBUTE_MAP_TO_COLOR,
};

/* CacheAttributeMapping::domain */
enum {
  /* Try to automatically map the attribute to the right domain (default behavior without mapping.)
   */
  CACHEFILE_ATTR_MAP_DOMAIN_AUTO,
  CACHEFILE_ATTR_MAP_DOMAIN_POINT,
  CACHEFILE_ATTR_MAP_DOMAIN_FACE_CORNER,
  CACHEFILE_ATTR_MAP_DOMAIN_FACE,
};

/* Custom data mapping for the attributes in the CacheFile. Since there might not be a standard way
 * of expressing what an attribute should be (e.g. is this float2 attribute a UV map?), and since
 * some software might write multi-dimensionnal data as arrays of 1D element (although the size of
 * the array will be N-time its expected size were it written as N-D data), we delegate to the user
 * the task of telling us what is supposed to be what through these mappings. */
typedef struct CacheAttributeMapping {
  struct CacheAttributeMapping *next, *prev;

  char name[64];
  char mapping;
  char domain;
  char _pad[6];
} CacheAttributeMapping;

/* CacheFile::velocity_unit
 * Determines what temporal unit is used to interpret velocity vectors for motion blur effects. */
enum {
  CACHEFILE_VELOCITY_UNIT_FRAME,
  CACHEFILE_VELOCITY_UNIT_SECOND,
};

typedef struct CacheFile {
  ID id;
  struct AnimData *adt;

  /** Paths of the objects inside of the archive referenced by this CacheFile. */
  ListBase object_paths;

  ListBase layers;

  /** 1024 = FILE_MAX. */
  char filepath[1024];

  char is_sequence;
  char forward_axis;
  char up_axis;
  char override_frame;

  float scale;
  /** The frame/time to lookup in the cache file. */
  float frame;
  /** The frame offset to subtract. */
  float frame_offset;

  char _pad[4];

  /** Animation flag. */
  short flag;

  /* eCacheFileType enum. */
  char type;

  /** Do not load data from the cache file and display objects in the scene as boxes, Cycles will
   * load objects directly from the CacheFile. Other render engines which can load Alembic data
   * directly can take care of rendering it themselves.
   */
  char use_render_procedural;

  char _pad1[3];

  /** Enable data prefetching when using the Cycles Procedural. */
  char use_prefetch;

  /** Size in megabytes for the prefetch cache used by the Cycles Procedural. */
  int prefetch_cache_size;

  /** Index of the currently selected layer in the UI, starts at 1. */
  char active_layer;

  /** Index, starting at 1, of the active attribute mapping in the UI. */
  char active_attribute_mapping;

  char _pad2[5];

  char velocity_unit;
  /* Name of the velocity property in the archive. */
  char velocity_name[64];

  ListBase attribute_mappings;

  /* Runtime */
  struct CacheArchiveHandle *handle;
  char handle_filepath[1024];
  struct GSet *handle_readers;
} CacheFile;

#ifdef __cplusplus
}
#endif

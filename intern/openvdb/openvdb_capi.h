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

#ifndef __OPENVDB_CAPI_H__
#define __OPENVDB_CAPI_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct OpenVDBReader;
struct OpenVDBWriter;
struct OpenVDBFloatGrid;
struct OpenVDBIntGrid;
struct OpenVDBVectorGrid;

int OpenVDB_getVersionHex(void);

enum {
  VEC_INVARIANT = 0,
  VEC_COVARIANT = 1,
  VEC_COVARIANT_NORMALIZE = 2,
  VEC_CONTRAVARIANT_RELATIVE = 3,
  VEC_CONTRAVARIANT_ABSOLUTE = 4,
};

struct OpenVDBFloatGrid *OpenVDB_export_grid_fl(struct OpenVDBWriter *writer,
                                                const char *name,
                                                float *data,
                                                const int res[3],
                                                float matrix[4][4],
                                                const float clipping,
                                                struct OpenVDBFloatGrid *mask);

struct OpenVDBIntGrid *OpenVDB_export_grid_ch(struct OpenVDBWriter *writer,
                                              const char *name,
                                              unsigned char *data,
                                              const int res[3],
                                              float matrix[4][4],
                                              const float clipping,
                                              struct OpenVDBFloatGrid *mask);

struct OpenVDBVectorGrid *OpenVDB_export_grid_vec(struct OpenVDBWriter *writer,
                                                  const char *name,
                                                  const float *data_x,
                                                  const float *data_y,
                                                  const float *data_z,
                                                  const int res[3],
                                                  float matrix[4][4],
                                                  short vec_type,
                                                  const float clipping,
                                                  const bool is_color,
                                                  struct OpenVDBFloatGrid *mask);

void OpenVDB_import_grid_fl(struct OpenVDBReader *reader,
                            const char *name,
                            float **data,
                            const int res[3]);

void OpenVDB_import_grid_ch(struct OpenVDBReader *reader,
                            const char *name,
                            unsigned char **data,
                            const int res[3]);

void OpenVDB_import_grid_vec(struct OpenVDBReader *reader,
                             const char *name,
                             float **data_x,
                             float **data_y,
                             float **data_z,
                             const int res[3]);

struct OpenVDBWriter *OpenVDBWriter_create(void);
void OpenVDBWriter_free(struct OpenVDBWriter *writer);
void OpenVDBWriter_set_flags(struct OpenVDBWriter *writer, const int flag, const bool half);
void OpenVDBWriter_add_meta_fl(struct OpenVDBWriter *writer, const char *name, const float value);
void OpenVDBWriter_add_meta_int(struct OpenVDBWriter *writer, const char *name, const int value);
void OpenVDBWriter_add_meta_v3(struct OpenVDBWriter *writer,
                               const char *name,
                               const float value[3]);
void OpenVDBWriter_add_meta_v3_int(struct OpenVDBWriter *writer,
                                   const char *name,
                                   const int value[3]);
void OpenVDBWriter_add_meta_mat4(struct OpenVDBWriter *writer,
                                 const char *name,
                                 float value[4][4]);
void OpenVDBWriter_write(struct OpenVDBWriter *writer, const char *filename);

struct OpenVDBReader *OpenVDBReader_create(void);
void OpenVDBReader_free(struct OpenVDBReader *reader);
void OpenVDBReader_open(struct OpenVDBReader *reader, const char *filename);
void OpenVDBReader_get_meta_fl(struct OpenVDBReader *reader, const char *name, float *value);
void OpenVDBReader_get_meta_int(struct OpenVDBReader *reader, const char *name, int *value);
void OpenVDBReader_get_meta_v3(struct OpenVDBReader *reader, const char *name, float value[3]);
void OpenVDBReader_get_meta_v3_int(struct OpenVDBReader *reader, const char *name, int value[3]);
void OpenVDBReader_get_meta_mat4(struct OpenVDBReader *reader,
                                 const char *name,
                                 float value[4][4]);

/*for OpenVDB particle mesher modifier */
struct OpenVDBGeom;
struct OpenVDBPrimitive;
struct ParticleList;

enum {
    LEVEL_FILTER_MEDIAN    = 0,
    LEVEL_FILTER_MEAN      = 1,
    LEVEL_FILTER_GAUSSIAN  = 2,
    LEVEL_FILTER_MEAN_CURV = 3,
    LEVEL_FILTER_LAPLACIAN = 4,
    LEVEL_FILTER_OFFSET    = 5,
};

enum {
    LEVEL_FILTER_ACC_FISRT   = 0,
    LEVEL_FILTER_ACC_SECOND  = 1,
    LEVEL_FILTER_ACC_THIRD   = 2,
    LEVEL_FILTER_ACC_WENO5   = 3,
    LEVEL_FILTER_ACC_HJWENO5 = 4,
};

void OpenVDB_filter_level_set(struct OpenVDBPrimitive *level_set,
                              struct OpenVDBPrimitive *filter_mask,
                              int accuracy, int type, int iterations,
                              int width, float offset);


void OpenVDB_from_particles(struct OpenVDBPrimitive *level_set,
                            struct OpenVDBPrimitive *mask_grid,
                            struct ParticleList *Pa, bool mask, float mask_width, float min_radius, bool trail, float trail_size);



struct OpenVDBPrimitive *OpenVDB_from_polygons(struct OpenVDBGeom *geom,
                                               struct OpenVDBPrimitive *level_set,
                                               float voxel_size, float int_band, float ext_band);

struct OpenVDBGeom *OpenVDB_to_polygons(struct OpenVDBPrimitive *level_set,
                                        struct OpenVDBPrimitive *mask_grid,
                                        float isovalue, float adaptivity,
                                        float mask_offset, bool invert_mask);

struct ParticleList *OpenVDB_create_part_list(size_t totpart, float rad_scale, float vel_scale);
void OpenVDB_part_list_free(struct ParticleList *part_list);
void OpenVDB_add_particle(struct ParticleList *part_list, float pos[3], float rad, float vel[3]);
void OpenVDB_set_part_list_flags(struct ParticleList *part_list, const bool has_radius, const bool has_velocity);

enum {
    VDB_GRID_INVALID = 0,
    VDB_GRID_FLOAT   = 1,
    VDB_GRID_DOUBLE  = 2,
    VDB_GRID_INT32   = 3,
    VDB_GRID_INT64   = 4,
    VDB_GRID_BOOL    = 5,
    VDB_GRID_VEC3F   = 6,
    VDB_GRID_VEC3D   = 7,
    VDB_GRID_VEC3I   = 8,
};

struct OpenVDBPrimitive *OpenVDBPrimitive_create(int grid_type);
struct OpenVDBPrimitive *OpenVDBPrimitive_create_level_set(float voxel_size, float half_width);
void OpenVDBPrimitive_free(struct OpenVDBPrimitive *vdb_prim);
void OpenVDBPrimitive_set_transform(struct OpenVDBPrimitive *vdb_prim, float mat[4][4]);

struct OpenVDBGeom *OpenVDBGeom_create(size_t num_points, size_t num_polys);
void OpenVDBGeom_free(struct OpenVDBGeom *geom);
void OpenVDBGeom_add_point(struct OpenVDBGeom *geom, const float point[3]);
void OpenVDBGeom_add_quad(struct OpenVDBGeom *geom, const int quad[4]);
void OpenVDBGeom_addTriangle(struct OpenVDBGeom *geom, const int tri[3]);
void OpenVDBGeom_get_point(struct OpenVDBGeom *geom, const size_t idx, float r_point[3]);
void OpenVDBGeom_get_quad(struct OpenVDBGeom *geom, const size_t idx, int r_quad[4]);
void OpenVDBGeom_get_triangle(struct OpenVDBGeom *geom, const size_t idx, int r_triangle[3]);
size_t OpenVDBGeom_get_num_points(struct OpenVDBGeom *geom);
size_t OpenVDBGeom_get_num_quads(struct OpenVDBGeom *geom);
size_t OpenVDBGeom_get_num_tris(struct OpenVDBGeom *geom);

void OpenVDB_draw_primitive(struct OpenVDBPrimitive *vdb_prim,
                            const bool draw_root,
                            const bool draw_level_1,
                            const bool draw_level_2,
                            const bool draw_leaves);

#ifdef __cplusplus
}
#endif

#endif /* __OPENVDB_CAPI_H__ */

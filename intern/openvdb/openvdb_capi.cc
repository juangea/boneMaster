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

#include "openvdb_capi.h"
#include "openvdb_dense_convert.h"
#include "openvdb_util.h"

/*for particle mesher modifier*/
#include <vector>
#include "openvdb_intern.h"
#include "openvdb_primitive.h"
#include "particle_tools.h"
/*end for particle mesher modifier*/

struct OpenVDBFloatGrid {
  int unused;
};
struct OpenVDBIntGrid {
  int unused;
};
struct OpenVDBVectorGrid {
  int unused;
};

int OpenVDB_getVersionHex()
{
  return openvdb::OPENVDB_LIBRARY_VERSION;
}

OpenVDBFloatGrid *OpenVDB_export_grid_fl(OpenVDBWriter *writer,
                                         const char *name,
                                         float *data,
                                         const int res[3],
                                         float matrix[4][4],
                                         const float clipping,
                                         OpenVDBFloatGrid *mask)
{
  Timer(__func__);

  using openvdb::FloatGrid;

  FloatGrid *mask_grid = reinterpret_cast<FloatGrid *>(mask);
  FloatGrid *grid = internal::OpenVDB_export_grid<FloatGrid>(
      writer, name, data, res, matrix, clipping, mask_grid);

  return reinterpret_cast<OpenVDBFloatGrid *>(grid);
}

OpenVDBIntGrid *OpenVDB_export_grid_ch(OpenVDBWriter *writer,
                                       const char *name,
                                       unsigned char *data,
                                       const int res[3],
                                       float matrix[4][4],
                                       const float clipping,
                                       OpenVDBFloatGrid *mask)
{
  Timer(__func__);

  using openvdb::FloatGrid;
  using openvdb::Int32Grid;

  FloatGrid *mask_grid = reinterpret_cast<FloatGrid *>(mask);
  Int32Grid *grid = internal::OpenVDB_export_grid<Int32Grid>(
      writer, name, data, res, matrix, clipping, mask_grid);

  return reinterpret_cast<OpenVDBIntGrid *>(grid);
}

OpenVDBVectorGrid *OpenVDB_export_grid_vec(struct OpenVDBWriter *writer,
                                           const char *name,
                                           const float *data_x,
                                           const float *data_y,
                                           const float *data_z,
                                           const int res[3],
                                           float matrix[4][4],
                                           short vec_type,
                                           const float clipping,
                                           const bool is_color,
                                           OpenVDBFloatGrid *mask)
{
  Timer(__func__);

  using openvdb::FloatGrid;
  using openvdb::GridBase;
  using openvdb::VecType;

  FloatGrid *mask_grid = reinterpret_cast<FloatGrid *>(mask);
  GridBase *grid = internal::OpenVDB_export_vector_grid(writer,
                                                        name,
                                                        data_x,
                                                        data_y,
                                                        data_z,
                                                        res,
                                                        matrix,
                                                        static_cast<VecType>(vec_type),
                                                        is_color,
                                                        clipping,
                                                        mask_grid);

  return reinterpret_cast<OpenVDBVectorGrid *>(grid);
}

void OpenVDB_import_grid_fl(OpenVDBReader *reader,
                            const char *name,
                            float **data,
                            const int res[3])
{
  Timer(__func__);

  internal::OpenVDB_import_grid<openvdb::FloatGrid>(reader, name, data, res);
}

void OpenVDB_import_grid_ch(OpenVDBReader *reader,
                            const char *name,
                            unsigned char **data,
                            const int res[3])
{
  internal::OpenVDB_import_grid<openvdb::Int32Grid>(reader, name, data, res);
}

void OpenVDB_import_grid_vec(struct OpenVDBReader *reader,
                             const char *name,
                             float **data_x,
                             float **data_y,
                             float **data_z,
                             const int res[3])
{
  Timer(__func__);

  internal::OpenVDB_import_grid_vector(reader, name, data_x, data_y, data_z, res);
}

OpenVDBWriter *OpenVDBWriter_create()
{
  return new OpenVDBWriter();
}

void OpenVDBWriter_free(OpenVDBWriter *writer)
{
  delete writer;
}

void OpenVDBWriter_set_flags(OpenVDBWriter *writer, const int flag, const bool half)
{
  int compression_flags = openvdb::io::COMPRESS_ACTIVE_MASK;

#ifdef WITH_OPENVDB_BLOSC
  if (flag == 0) {
    compression_flags |= openvdb::io::COMPRESS_BLOSC;
  }
  else
#endif
      if (flag == 1) {
    compression_flags |= openvdb::io::COMPRESS_ZIP;
  }
  else {
    compression_flags = openvdb::io::COMPRESS_NONE;
  }

  writer->setFlags(compression_flags, half);
}

void OpenVDBWriter_add_meta_fl(OpenVDBWriter *writer, const char *name, const float value)
{
  writer->insertFloatMeta(name, value);
}

void OpenVDBWriter_add_meta_int(OpenVDBWriter *writer, const char *name, const int value)
{
  writer->insertIntMeta(name, value);
}

void OpenVDBWriter_add_meta_v3(OpenVDBWriter *writer, const char *name, const float value[3])
{
  writer->insertVec3sMeta(name, value);
}

void OpenVDBWriter_add_meta_v3_int(OpenVDBWriter *writer, const char *name, const int value[3])
{
  writer->insertVec3IMeta(name, value);
}

void OpenVDBWriter_add_meta_mat4(OpenVDBWriter *writer, const char *name, float value[4][4])
{
  writer->insertMat4sMeta(name, value);
}

void OpenVDBWriter_write(OpenVDBWriter *writer, const char *filename)
{
  writer->write(filename);
}

OpenVDBReader *OpenVDBReader_create()
{
  return new OpenVDBReader();
}

void OpenVDBReader_free(OpenVDBReader *reader)
{
  delete reader;
}

void OpenVDBReader_open(OpenVDBReader *reader, const char *filename)
{
  reader->open(filename);
}

void OpenVDBReader_get_meta_fl(OpenVDBReader *reader, const char *name, float *value)
{
  reader->floatMeta(name, *value);
}

void OpenVDBReader_get_meta_int(OpenVDBReader *reader, const char *name, int *value)
{
  reader->intMeta(name, *value);
}

void OpenVDBReader_get_meta_v3(OpenVDBReader *reader, const char *name, float value[3])
{
  reader->vec3sMeta(name, value);
}

void OpenVDBReader_get_meta_v3_int(OpenVDBReader *reader, const char *name, int value[3])
{
  reader->vec3IMeta(name, value);
}

void OpenVDBReader_get_meta_mat4(OpenVDBReader *reader, const char *name, float value[4][4])
{
  reader->mat4sMeta(name, value);
}

/*for particle mesher modifier*/
using namespace openvdb;

/* ****************************** Particle List ****************************** */

ParticleList *OpenVDB_create_part_list(size_t totpart, float rad_scale, float vel_scale)
{
	return new ParticleList(totpart, rad_scale, vel_scale);
}

void OpenVDB_part_list_free(ParticleList *part_list)
{
	delete part_list;
	part_list = NULL;
}

void OpenVDB_add_particle(ParticleList *part_list,
                          float pos[3], float rad, float vel[3])
{
	Vec3R nvel(vel);
	float nrad = rad * part_list->radius_scale();
	nvel *= part_list->velocity_scale();

	part_list->add(pos, nrad, nvel);
}

void OpenVDB_from_particles(OpenVDBPrimitive *level_set, OpenVDBPrimitive *mask_grid,
                            ParticleList *Pa, bool mask, float mask_width,
                            float min_radius, bool trail, float trail_size)
{
	internal::OpenVDB_from_particles(level_set, mask_grid, *Pa, mask, mask_width,
	                                 min_radius, trail, trail_size);
}

void OpenVDB_set_part_list_flags(struct ParticleList *part_list, const bool has_radius, const bool has_velocity)
{
	part_list->set_flags(has_radius, has_velocity);
}

/* **************************** OpenVDB Primitive **************************** */

OpenVDBPrimitive *OpenVDBPrimitive_create(int grid_type)
{
	OpenVDBPrimitive *vdb_prim = new OpenVDBPrimitive();
	GridBase::Ptr grid;

	switch (grid_type) {
		case VDB_GRID_FLOAT:
			grid = FloatGrid::create(0.0f);
			break;
		case VDB_GRID_DOUBLE:
			grid = DoubleGrid::create(0.0);
			break;
		case VDB_GRID_INT32:
			grid = Int32Grid::create(0);
			break;
		case VDB_GRID_INT64:
			grid = Int64Grid::create(0);
			break;
		case VDB_GRID_BOOL:
			grid = BoolGrid::create(false);
			break;
		case VDB_GRID_VEC3F:
			grid = Vec3fGrid::create(Vec3f(0.0f));
			break;
		case VDB_GRID_VEC3D:
			grid = Vec3dGrid::create(Vec3d(0.0));
			break;
		case VDB_GRID_VEC3I:
			grid = Vec3IGrid::create(Vec3I(0));
			break;
	}

	vdb_prim->setGrid(grid);

	return vdb_prim;
}

OpenVDBPrimitive *OpenVDBPrimitive_create_level_set(float voxel_size, float half_width)
{
	using namespace openvdb;

	OpenVDBPrimitive *vdb_prim = new OpenVDBPrimitive();

	FloatGrid::Ptr grid = createLevelSet<FloatGrid>(voxel_size, half_width);
	grid->setTransform(math::Transform::createLinearTransform(voxel_size));
	vdb_prim->setGrid(grid);

	return vdb_prim;
}

void OpenVDBPrimitive_free(OpenVDBPrimitive *vdb_prim)
{
	delete vdb_prim;
	vdb_prim = NULL;
}

void OpenVDBPrimitive_set_transform(OpenVDBPrimitive *vdb_prim, float mat[4][4])
{
	vdb_prim->setTransform(mat);
}

void OpenVDB_filter_level_set(OpenVDBPrimitive *level_set, OpenVDBPrimitive *filter_mask,
                              int accuracy, int type, int iterations, int width, float offset)
{
	internal::OpenVDB_filter_level_set(level_set, filter_mask, accuracy, type, iterations, width, offset);
}

OpenVDBPrimitive *OpenVDB_from_polygons(OpenVDBGeom *geom, OpenVDBPrimitive *level_set, float voxel_size, float int_band, float ext_band)
{
	return internal::OpenVDB_from_polygons(geom, level_set, voxel_size, int_band, ext_band);
}

OpenVDBGeom *OpenVDB_to_polygons(OpenVDBPrimitive *level_set, OpenVDBPrimitive *mask_grid, float isovalue, float adaptivity, float mask_offset, bool invert_mask)
{
	return internal::OpenVDB_to_polygons(level_set, mask_grid, isovalue, adaptivity, mask_offset, invert_mask);
}

/* ******************************* OpenVDBGeom ******************************* */

struct OpenVDBGeom *OpenVDBGeom_create(size_t num_points, size_t num_polys)
{
	OpenVDBGeom *geom = new OpenVDBGeom;
	geom->m_points.reserve(num_points);
	geom->m_polys.reserve(num_polys);

	return geom;
}

void OpenVDBGeom_free(struct OpenVDBGeom *geom)
{
	delete geom;
	geom = NULL;
}

void OpenVDBGeom_add_point(OpenVDBGeom *geom, const float point[3])
{
	geom->m_points.push_back(point);
}

void OpenVDBGeom_add_quad(OpenVDBGeom *geom, const int quad[4])
{
	geom->m_polys.push_back(quad);
}

void OpenVDBGeom_addTriangle(OpenVDBGeom *geom, const int tri[3])
{
	geom->m_tris.push_back(tri);
}

void OpenVDBGeom_get_point(OpenVDBGeom *geom, const size_t idx, float r_point[3])
{
	Vec3s point = geom->m_points[idx];
	r_point[0] = point.x();
	r_point[1] = point.y();
	r_point[2] = point.z();
}

void OpenVDBGeom_get_quad(OpenVDBGeom *geom, const size_t idx, int r_quad[4])
{
	Vec4I quad = geom->m_polys[idx];
	r_quad[0] = quad.x();
	r_quad[1] = quad.y();
	r_quad[2] = quad.z();
	r_quad[3] = quad.w();
}

void OpenVDBGeom_get_triangle(OpenVDBGeom *geom, const size_t idx, int r_triangle[3])
{
	Vec3I triangle = geom->m_tris[idx];
	r_triangle[0] = triangle.x();
	r_triangle[1] = triangle.y();
	r_triangle[2] = triangle.z();
}

size_t OpenVDBGeom_get_num_points(OpenVDBGeom *geom)
{
	return geom->m_points.size();
}

size_t OpenVDBGeom_get_num_quads(OpenVDBGeom *geom)
{
	return geom->m_polys.size();
}

size_t OpenVDBGeom_get_num_tris(struct OpenVDBGeom *geom)
{
	return geom->m_tris.size();
}

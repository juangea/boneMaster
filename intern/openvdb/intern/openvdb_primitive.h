#ifndef __OPENVDB_PRIMITIVE_H__
#define __OPENVDB_PRIMITIVE_H__

#include <openvdb/openvdb.h>

struct OpenVDBGeom {
	std::vector<openvdb::Vec3s> m_points;
	std::vector<openvdb::Vec4I> m_polys;
	std::vector<openvdb::Vec3I> m_tris;
};

class OpenVDBPrimitive {
	openvdb::GridBase::Ptr m_grid;

public:
	OpenVDBPrimitive();
	~OpenVDBPrimitive();

	openvdb::GridBase &getGrid();
	const openvdb::GridBase &getConstGrid() const;
	openvdb::GridBase::Ptr getGridPtr();
	openvdb::GridBase::ConstPtr getConstGridPtr() const;

	void setGrid(openvdb::GridBase::Ptr grid);
	void setTransform(const float mat[4][4]);
};

#endif /* __OPENVDB_PRIMITIVE_H__ */

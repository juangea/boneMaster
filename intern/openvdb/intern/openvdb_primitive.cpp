#include "openvdb_primitive.h"

OpenVDBPrimitive::OpenVDBPrimitive()
{}

OpenVDBPrimitive::~OpenVDBPrimitive()
{}

openvdb::GridBase &OpenVDBPrimitive::getGrid()
{
	return *m_grid;
}

const openvdb::GridBase &OpenVDBPrimitive::getConstGrid() const
{
	return *m_grid;
}

openvdb::GridBase::Ptr OpenVDBPrimitive::getGridPtr()
{
	return m_grid;
}

openvdb::GridBase::ConstPtr OpenVDBPrimitive::getConstGridPtr() const
{
	return m_grid;
}

void OpenVDBPrimitive::setGrid(openvdb::GridBase::Ptr grid)
{
	m_grid = grid->copyGrid();
}

static openvdb::Mat4R convertMatrix(const float mat[4][4])
{
	return openvdb::Mat4R(
	            mat[0][0], mat[0][1], mat[0][2], mat[0][3],
	            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
	            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
	            mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
}

/* A simple protection to avoid crashes for cases when something goes wrong for
 * some reason in the matrix creation. */
static openvdb::math::MapBase::Ptr createAffineMap(const float mat[4][4])
{
	using namespace openvdb::math;
	MapBase::Ptr transform;

	try {
		transform.reset(new AffineMap(convertMatrix(mat)));
	}
	catch (const openvdb::ArithmeticError &e) {
		std::cerr << e.what() << "\n";
		transform.reset(new AffineMap());
	}

	return transform;
}

void OpenVDBPrimitive::setTransform(const float mat[4][4])
{
	using namespace openvdb::math;

	Transform::Ptr transform = Transform::Ptr(new Transform(createAffineMap(mat)));
	m_grid->setTransform(transform);
}

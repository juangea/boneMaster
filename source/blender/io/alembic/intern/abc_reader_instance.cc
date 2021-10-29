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
 */

#include "abc_reader_instance.h"

#include "DNA_object_types.h"

#include "BLI_assert.h"

#include "BKE_lib_id.h"
#include "BKE_lib_remap.h"
#include "BKE_object.h"

#include "abc_reader_manager.h"

namespace blender::io::alembic {

AbcInstanceReader::AbcInstanceReader(const Alembic::Abc::IObject &object, ImportSettings &settings)
    : AbcObjectReader(object, settings)
{
}

bool AbcInstanceReader::valid() const
{
  // TODO(kevindietrich)
  return true;
}

bool AbcInstanceReader::accepts_object_type(
    const Alembic::AbcCoreAbstract::ObjectHeader & /*alembic_header*/,
    const Object *const /*ob*/,
    const char ** /*err_str*/) const
{
  /* TODO(kevindietrich) */
  return true;
}

void AbcInstanceReader::readObjectData(Main *bmain,
                                       const AbcReaderManager &manager,
                                       const Alembic::Abc::ISampleSelector & /* sample_sel */)
{
  /* For reference on duplication, see ED_object_add_duplicate_linked.
   *
   * In this function, we only duplicate the object, as the rest (adding to view layer, tagging the
   * depsgraph, etc.) is done at the end of the import.
   */

  Object *ob = manager.get_blender_object_for_path(m_iobject.instanceSourcePath());
  BLI_assert(ob);
  /* 0 = linked */
  uint dupflag = 0;
  uint duplicate_options = LIB_ID_DUPLICATE_IS_SUBPROCESS | LIB_ID_DUPLICATE_IS_ROOT_ID;

  m_object = static_cast<Object *>(
      ID_NEW_SET(ob, BKE_object_duplicate(bmain, ob, dupflag, duplicate_options)));

  /* link own references to the newly duplicated data T26816. */
  BKE_libblock_relink_to_newid(&m_object->id);
}

}  // namespace blender::io::alembic

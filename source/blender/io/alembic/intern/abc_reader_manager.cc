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

/** \file
 * \ingroup balembic
 */

#include "abc_reader_manager.h"

#include "abc_reader_instance.h"

namespace blender::io::alembic {

AbcObjectReader *AbcReaderManager::create_instance_reader(Alembic::Abc::v12::IObject iobject,
                                                          ImportSettings &settings)
{
  std::cerr << "Creating an instance reader...\n";
  AbcObjectReader *reader = new AbcInstanceReader(iobject, settings);
  m_instance_readers.push_back(reader);
  m_readers_all.push_back(reader);
  return reader;
}

Object *AbcReaderManager::get_blender_object_for_path(const std::string &path) const
{
  AbcObjectReader *reader = get_object_reader_for_path(path);
  if (!reader) {
    return nullptr;
  }
  return reader->object();
}

AbcObjectReader *AbcReaderManager::get_object_reader_for_path(const std::string &path) const
{
  MapIteratorType iter = m_readers_map.find(path);
  if (iter == m_readers_map.end()) {
    return nullptr;
  }
  return iter->second;
}

}  // namespace blender::io::alembic

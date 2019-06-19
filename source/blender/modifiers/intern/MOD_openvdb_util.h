/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
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
 *
 * Contributor(s): Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_openvdb_util.h
 *  \ingroup modifiers
 */


#ifndef __MOD_OPENVDB_UTIL_H__
#define __MOD_OPENVDB_UTIL_H__

struct Object;
struct Base;
struct Mesh;
struct ParticleMesherModifierData;
struct Depsgraph;

/* Performs skinning of a particle system. On success returns a DerivedMesh.
 * On failure returns NULL. */
Mesh *NewParticleMesh(Mesh *me, struct Object *ob,
					  Mesh *cutter_dm, struct Object *cutter_ob,
					  struct ParticleMesherModifierData *pmmd, Scene *scene, struct Depsgraph* depsgraph);

#endif  /* __MOD_OPENVDB_UTIL_H__ */

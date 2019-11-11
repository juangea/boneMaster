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
 * \ingroup bmesh
 *
 * Destructive extrue face regions.
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_alloca.h"
#include "BLI_memarena.h"
#include "BKE_customdata.h"

#include "bmesh.h"

#include "intern/bmesh_operators_private.h" /* own include */

#define VERT_MARK 1
#define EDGE_MARK 1
#define EDGE_WELD 2 /* used for tracking edges to be merged into existing edges */
#define FACE_MARK 1 /* used for faces to be destructed */

/**
 * Checks whether any faces not in BM_ELEM_TAG and connected to e are parrallel with dir.
 * force_side determines whether faces in the exact opposite side can be used.
 */
static BMFace *bm_edge_parallel_face(BMEdge *e, const float dir[3], const bool force_side)
{
  BMLoop *l = e->l;
  BMLoop *l_iter;
  l_iter = l;
  bool allow_opposite = !force_side && (BM_edge_face_count(e) == 2);
  BMFace *face_opposite = NULL; /* used to prioritize faces on the side of dir first */

  do {
    if (BM_elem_flag_test(l_iter->f, BM_ELEM_TAG)) {
      continue;
    }

    /* check if face is parallel */
    float dot = dot_v3v3(l_iter->f->no, dir);
    if (fabs(dot) < 0.0001f) {

      /* check which side the edge the face is on, compared to dir */
      float tan[3];
      BM_edge_calc_face_tangent(e, l_iter, tan);
      if (dot_v3v3(tan, dir) > 0.00001f) {
        /* face is parallel and on the same side; we can return it immediately */
        return l_iter->f;
      }
      else if (allow_opposite) {
        /* will be used if no other faces on the same side are found */
        face_opposite = l_iter->f;
      }
    }

  } while ((l_iter = l_iter->radial_next) != l);

  if (face_opposite) {
    return face_opposite;
  }
  else {
    return NULL;
  }
}

/* Finds an edge of v which is pointing in the same direction as dir. */
static BMEdge *bm_vert_parallel_edge(BMVert *v, const float dir[3])
{
  BMIter iter;
  BMEdge *e;
  BM_ITER_ELEM (e, &iter, v, BM_EDGES_OF_VERT) {
    if (BM_edge_calc_length(e) > 0.0001f) {
      float e_dir[3];
      sub_v3_v3v3(e_dir, BM_edge_other_vert(e, v)->co, v->co);
      normalize_v3(e_dir);

      if (dot_v3v3(e_dir, dir) > 0.999f) {
        return e;
      }
    }
  }
  return NULL;
}

/**
 * Implementation is as follows...
 *
 * - extrude using the extrude operator
 * - flag boundary edges which have a face parallel to the extrude direciton
 * - join those faces and the boundary edges' extrude quads
 * - merge verticies onto coincident parallel edges
 * - calculate the limit of depth when parallel edges aren't long enough
 * - translate verts
 */
void bmo_extrude_destructive_exec(BMesh *bm, BMOperator *op)
{
  const float depth = BMO_slot_float_get(op->slots_in, "depth");

  float extrudeDir[3];
  BMO_slot_vec_get(op->slots_in, "dir", extrudeDir);
  normalize_v3(extrudeDir);
  if (depth < 0) {
    mul_v3_fl(extrudeDir, -1.0f);
  }

  float depth_limit = fabs(depth) + 1; /* the depth limit without passing new geometry */
  /* depth_limit just needs to be bigger than depth at this point */

  const bool keep_orig_loop = BMO_slot_bool_get(op->slots_in, "original_loop");
  const bool keep_sides = BMO_slot_bool_get(op->slots_in, "keep_sides");

  BMIter iter;
  BMOIter oiter;
  BMVert *v;
  BMEdge *e;
  BMFace *f;

  BM_mesh_elem_hflag_disable_all(bm, BM_ALL, BM_ELEM_TAG, false);
  BMO_slot_buffer_hflag_enable(bm, op->slots_in, "faces", BM_FACE, BM_ELEM_TAG, false);

  /* tag edges of tagged faces */
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
      BMIter eiter;
      BM_ITER_ELEM (e, &eiter, f, BM_EDGES_OF_FACE) {
        BM_elem_flag_enable(e, BM_ELEM_TAG);
      }
    }
  }

  if (depth == 0) {
    /* do nothing */
    BMO_slot_buffer_from_enabled_hflag(bm, op, op->slots_out, "geom.out", BM_FACE, BM_ELEM_TAG);
    BMO_slot_float_set(op->slots_out, "depth_limit", depth_limit);
    return;
  }

  BMOperator extop;

  BMO_op_initf(bm, &extop, op->flag, "extrude_face_region geom=%hef use_keep_orig=%b", BM_ELEM_TAG, false);
  BMO_op_exec(bm, &extop);

  BM_mesh_elem_hflag_disable_all(bm, BM_ALL, BM_ELEM_TAG, false);
  BMO_slot_buffer_hflag_enable(bm, extop.slots_out, "geom.out", BM_VERT | BM_FACE, BM_ELEM_TAG, true);
  /* While other faces and edges will be added into BM_ELEM_TAG, the verts will remain
   * untouched and will be accessed only for the final translation. */

  BMO_mesh_flag_disable_all(bm, op, BM_EDGE, EDGE_MARK);
  BMO_mesh_flag_disable_all(bm, op, BM_EDGE, EDGE_WELD);
  BMO_mesh_flag_disable_all(bm, op, BM_VERT, VERT_MARK);
  BMO_mesh_flag_disable_all(bm, op, BM_FACE, FACE_MARK);

  /* Loop through original boundary edges. Setup by flagging certain geometry. */
  for (e = BMO_iter_new(&oiter, extop.slots_out, "boundary_map.out", 0); e;
       e = BMO_iter_step(&oiter)) {
    BMEdge *e_new = BMO_iter_map_value_ptr(&oiter);

    /* tag extrude quads */
    BMVert *varr[4] = {e->v2, e->v1, e_new->v1, e_new->v2};
    if (BM_face_exists(varr, 4) != NULL) {
      BMO_face_flag_enable(bm, BM_face_exists(varr, 4), FACE_MARK);
      /* tag with BM_ELEM_TAG so that bm_edge_parallel_face will ignore it */
      BM_elem_flag_enable(BM_face_exists(varr, 4), BM_ELEM_TAG);

      /* tag these edges in case they become wire and need to be deleted */
      BM_elem_flag_enable(e, BM_ELEM_TAG);
      BM_elem_flag_enable(BM_edge_exists(e->v1, e_new->v1), BM_ELEM_TAG);
      BM_elem_flag_enable(BM_edge_exists(e_new->v2, e->v2), BM_ELEM_TAG);
    }

    f = bm_edge_parallel_face(e, extrudeDir, keep_orig_loop);
    if (f != NULL) {
      /* these tags help keep track of destructing geometry */
      BMO_edge_flag_enable(bm, e, EDGE_MARK);
      BMO_face_flag_enable(bm, f, FACE_MARK);

      /* tag verts incase they need to be dissolved later */
      BMO_vert_flag_enable(bm, e->v1, VERT_MARK);
      BMO_vert_flag_enable(bm, e->v2, VERT_MARK);
    }

    f = bm_edge_parallel_face(e, extrudeDir, true);
    /* untag since all bm_edge_parallel_face checks are over */
    BM_elem_flag_disable(BM_face_exists(varr, 4), BM_ELEM_TAG);

    if (f != NULL && keep_sides) {
      /* remake extrude quads, but don't tag with FACE_MARK so they aren't changed */
      BM_face_create_verts(bm, varr, 4, BM_face_exists(varr, 4), BM_CREATE_NOP, false);
    }

    /* Prepare for merging verts onto coincident edges. */
    for (int j = 0; j < 2; j++) {
      v = (j == 0) ? e->v1 : e->v2;
      BMVert *v_new = (j == 0) ? e_new->v1 : e_new->v2;

      BMEdge *edge_merge = bm_vert_parallel_edge(v, extrudeDir);
      if (edge_merge != NULL) {

        /* The edge of v to v_new will be merged into edge_merge.
         * Tagging one vert with VERT_MARK keeps track of v versus v_new, for later. */
        BMO_vert_flag_enable(bm, v, VERT_MARK);
        BMO_edge_flag_enable(bm, BM_edge_exists(v, v_new), EDGE_WELD);

        /* Calculate depth_limit, given that verts shouldn't be moved
         * past edges they will be merged onto. */
        depth_limit = fminf(depth_limit, BM_edge_calc_length(edge_merge));
      }
    }
  }

  /* Join extrusion quads to faces to be destructed. This is the main destructive step. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGE) {
    if (BMO_edge_flag_test(bm, e, EDGE_MARK)) {
      BMFace *farr[2];
      int farr_len = 0;

      BMIter fiter;
      BM_ITER_ELEM (f, &fiter, e, BM_FACES_OF_EDGE) {
        if (BMO_face_flag_test(bm, f, FACE_MARK) && farr_len < 2) {
          farr[farr_len] = f;
          farr_len++;
        }
      }

      f = BM_faces_join(bm, farr, farr_len, false);
      /* Set do_del as false and delete any wire edge later. This is more reliable than
       * enabling do_del, in cases where this operator has to separate geometry. */

      /* mark the joined face in case another edge will also use this face */
      BMO_face_flag_enable(bm, f, FACE_MARK);
    }
  }

  /* Merge edges from extrusion onto the flagged, parallel edges. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGE) {
    if (BMO_edge_flag_test(bm, e, EDGE_WELD)) {

      v = (BMO_vert_flag_test(bm, e->v1, VERT_MARK)) ? e->v1 : e->v2;
      BMVert *v_new = BM_edge_other_vert(e, v);

      BMEdge *edge_merge = bm_vert_parallel_edge(v, extrudeDir);
      if (edge_merge != NULL) {

        BMVert *v_split = BM_edge_split(bm, edge_merge, v, NULL, 0.5f);

        BMVert *v_merge[2];
        v_merge[0] = v_new;
        v_merge[1] = v_split;

        BMO_op_callf(bm,
                    op->flag,
                    "pointmerge verts=%eb merge_co=%v",
                    v_merge,
                    2,
                    v_new->co);
      }
    }
  }

  /* Delete any wire edges left from BM_faces_join where do_del was false. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGE) {
    if (BM_elem_flag_test(e, BM_ELEM_TAG)) {
      if (BM_edge_is_wire(e)) {
        BMO_op_callf(bm, 0, "delete geom=%e context=%i", e, DEL_EDGES);
      }
    }
  }

  /* Dissolve verts between two straight edges, created from BM_faces_join or an edge merge. */
  BM_ITER_MESH (v, &iter, bm, BM_VERT) {
    if (BMO_vert_flag_test(bm, v, VERT_MARK)) {
      if (!keep_orig_loop && BM_vert_edge_count(v) == 2) {

        /* check if both edges parrallel */
        bool both_parallel = true;

        BMIter eiter;
        BM_ITER_ELEM (e, &eiter, v, BM_EDGES_OF_VERT) {

          float edgeDir[3];
          sub_v3_v3v3(edgeDir, e->v1->co, e->v2->co);
          normalize_v3(edgeDir);

          if (BM_edge_calc_length(e) > 0.0001f && fabs(dot_v3v3(edgeDir, extrudeDir)) < 0.999f) {
            both_parallel = false;
          }
        }

        if (both_parallel) {
          BMO_op_callf(bm,
                      0,
                      "dissolve_verts verts=%e use_face_split=%b use_boundary_tear=%b",
                      v,
                      false,
                      false);
        }
      }
    }
  }

  /* change from absolute to actual value for the operator output */
  depth_limit *= depth/fabs(depth);
  BMO_slot_float_set(op->slots_out, "depth_limit", depth_limit);

  /* translate verts */
  float trans[3];
  float final_depth = min_ff(fabs(depth), fabs(depth_limit));
  mul_v3_v3fl(trans, extrudeDir, final_depth);

  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
      add_v3_v3(v->co, trans);
    }
  }

  BMO_slot_copy(&extop, slots_out, "geom.out", op, slots_out, "geom.out");
  BMO_op_finish(bm, &extop);
}

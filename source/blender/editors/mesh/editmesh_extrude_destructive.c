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
 * \ingroup edmesh
 */

#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"

#include "BLI_string.h"
#include "BLI_math.h"

#include "BLT_translation.h"

#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_editmesh.h"
#include "BKE_unit.h"
#include "BKE_layer.h"

#include "RNA_define.h"
#include "RNA_access.h"

#include "WM_api.h"
#include "WM_types.h"

#include "UI_interface.h"

#include "ED_mesh.h"
#include "ED_numinput.h"
#include "ED_screen.h"
#include "ED_space_api.h"
#include "ED_transform.h"
#include "ED_view3d.h"

#include "mesh_intern.h" /* own include */

typedef struct {
  BMEditMesh *em;
  BMBackup mesh_backup;
} ExtrudeObjectStore;

typedef struct {
  float initial_val;
  float pixel_size; /* use when mouse input is interpreted as spatial distance */
  bool is_modal;
  bool shift;
  float shift_amount;
  bool snap;
  float extrude_dir[3];
  float screen_extrude_dir[2];
  float max_obj_scale;
  NumInput num_input;

  ExtrudeObjectStore *ob_store;
  uint ob_store_len;

  /* modal only */
  short gizmo_flag;
} ExtrudeData;

static void edbm_weld_extrusion (wmOperator *op, BMEditMesh *em, const bool destroy_end)
{
  EDBM_automerge_and_split(em->ob, true, true, true, BM_ELEM_SELECT, 0.001f);
  EDBM_selectmode_flush_ex(em, SCE_SELECT_EDGE);

  if (destroy_end) {
    BMIter iter;
    BMFace *f;
    BM_ITER_MESH (f, &iter, em->bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
        /* check if face contains boundary edges */
        BMLoop *l_iter = f->l_first;
        bool boundary_face = false;

        do {
          if (BM_edge_is_boundary(l_iter->e)) {
            boundary_face = true;
          }
        } while ((l_iter = l_iter->next) != f->l_first && !boundary_face);

        if (boundary_face) {
          /* delete the face */
          EDBM_op_callf(em, op, "delete geom=%e context=%i", f, DEL_FACES);
        }
      }
    }
  }
  BM_mesh_elem_hflag_disable_all(em->bm, BM_VERT | BM_EDGE, BM_ELEM_SELECT, true);
}

static void edbm_extrude_destructive_update_header(wmOperator *op, bContext *C)
{
  ExtrudeData *opdata = op->customdata;

  const char *str = IFACE_(
      "Confirm: Enter/LClick, Cancel: (Esc/RClick), "
      "Depth: %s, Original Loop (D): (%s), Destroy End: (%s), Successive Loops (E): (%s)");

  char msg[UI_MAX_DRAW_STR];
  ScrArea *sa = CTX_wm_area(C);
  Scene *sce = CTX_data_scene(C);

  if (sa) {
    char flts_str[NUM_STR_REP_LEN * 2];
    if (hasNumInput(&opdata->num_input)) {
      outputNumInput(&opdata->num_input, flts_str, &sce->unit);
    }
    else {
      BLI_snprintf(flts_str, NUM_STR_REP_LEN, "%f", RNA_float_get(op->ptr, "depth"));
    }
    BLI_snprintf(msg,
                  sizeof(msg),
                  str,
                  flts_str,
                  WM_bool_as_string(RNA_boolean_get(op->ptr, "original_loop")),
                  WM_bool_as_string(RNA_boolean_get(op->ptr, "destroy_end")),
                  WM_bool_as_string(RNA_boolean_get(op->ptr, "iterate_loops")));

    ED_area_status_text(sa, msg);
  }
}

static bool edbm_extrude_destructive_init(bContext *C, wmOperator *op, const bool is_modal)
{
  ExtrudeData *opdata;
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  ARegion *ar = CTX_wm_region(C);

  if (is_modal) {
    RNA_float_set(op->ptr, "depth", 0.0f);
  }

  op->customdata = opdata = MEM_mallocN(sizeof(ExtrudeData), "inset_operator_data");

  uint objects_used_len = 0;

  opdata->max_obj_scale = FLT_MIN;

  {
    uint ob_store_len = 0;
    Object **objects = BKE_view_layer_array_from_objects_in_edit_mode_unique_data(
        view_layer, CTX_wm_view3d(C), &ob_store_len);
    opdata->ob_store = MEM_malloc_arrayN(ob_store_len, sizeof(*opdata->ob_store), __func__);
    for (uint ob_index = 0; ob_index < ob_store_len; ob_index++) {
      Object *obedit = objects[ob_index];
      float scale = mat4_to_scale(obedit->obmat);
      opdata->max_obj_scale = max_ff(opdata->max_obj_scale, scale);
      BMEditMesh *em = BKE_editmesh_from_object(obedit);
      if (em->bm->totvertsel > 0) {
        opdata->ob_store[objects_used_len].em = em;
        objects_used_len++;
      }
    }
    MEM_freeN(objects);
    opdata->ob_store_len = objects_used_len;
  }

  /* Find the direction of the extrusion and the projection of it into screen space.*/

  /* calculate combined normals of selected faces and average position*/
  zero_v3(opdata->extrude_dir);
  for (uint ob_index = 0; ob_index < opdata->ob_store_len; ob_index++) {
    BMEditMesh *em = opdata->ob_store[ob_index].em;
    BMFace *f;
    BMIter iter;
    BM_ITER_MESH (f, &iter, em->bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
        add_v3_v3(opdata->extrude_dir, f->no);
      }
    }
  }
  normalize_v3(opdata->extrude_dir);

  float extrude_pos[3];
  calculateTransformCenter(C, V3D_AROUND_CENTER_MEDIAN, extrude_pos, NULL);
  float extrude_pos_offset[3];
  add_v3_v3v3(extrude_pos_offset, extrude_pos, opdata->extrude_dir);

  /* calculate direction of extrude_dir in screen space by projecting
   * the position and the offset position then subtracting*/
  float screen_pos[2];
  float screen_pos_offset[2];

  ED_view3d_project_float_global(ar, extrude_pos, screen_pos, V3D_PROJ_RET_OK);
  ED_view3d_project_float_global(ar, extrude_pos_offset, screen_pos_offset, V3D_PROJ_RET_OK);
  sub_v2_v2v2(opdata->screen_extrude_dir, screen_pos_offset, screen_pos);

  normalize_v2(opdata->screen_extrude_dir);

  opdata->shift = false;
  opdata->shift_amount = 0.0f;
  opdata->snap = false;
  opdata->is_modal = is_modal;

  initNumInput(&opdata->num_input);
  opdata->num_input.idx_max = 1; /* Two elements. */
  opdata->num_input.unit_sys = scene->unit.system;
  opdata->num_input.unit_type[0] = B_UNIT_LENGTH;

  if (is_modal) {
    View3D *v3d = CTX_wm_view3d(C);

    for (uint ob_index = 0; ob_index < opdata->ob_store_len; ob_index++) {
      opdata->ob_store[ob_index].mesh_backup = EDBM_redo_state_store(
          opdata->ob_store[ob_index].em);
    }

    G.moving = G_TRANSFORM_EDIT;
    if (v3d) {
      opdata->gizmo_flag = v3d->gizmo_flag;
      v3d->gizmo_flag = V3D_GIZMO_HIDE;
    }
  }

  return true;
}

static void edbm_extrude_destructive_exit(bContext *C, wmOperator *op)
{
  ExtrudeData *opdata = op->customdata;
  ScrArea *sa = CTX_wm_area(C);

  if (opdata->is_modal) {
    View3D *v3d = CTX_wm_view3d(C);
    ARegion *ar = CTX_wm_region(C);
    for (uint ob_index = 0; ob_index < opdata->ob_store_len; ob_index++) {
      EDBM_redo_state_free(&opdata->ob_store[ob_index].mesh_backup, NULL, false);
    }

    if (v3d) {
      v3d->gizmo_flag = opdata->gizmo_flag;
    }
    G.moving = 0;
  }

  if (sa) {
    ED_area_status_text(sa, NULL);
  }

  MEM_SAFE_FREE(opdata->ob_store);
  MEM_SAFE_FREE(op->customdata);
}

static void edbm_extrude_destructive_cancel(bContext *C, wmOperator *op)
{
  ExtrudeData *opdata;

  opdata = op->customdata;
  if (opdata->is_modal) {
    for (uint ob_index = 0; ob_index < opdata->ob_store_len; ob_index++) {
      EDBM_redo_state_free(
          &opdata->ob_store[ob_index].mesh_backup, opdata->ob_store[ob_index].em, true);
      EDBM_update_generic(opdata->ob_store[ob_index].em, false, true);
    }
  }

  edbm_extrude_destructive_exit(C, op);

  /* need to force redisplay or we may still view the modified result */
  ED_region_tag_redraw(CTX_wm_region(C));
}

/* Disabling allow_snap is used after already calling this function with snapping. Snapping
 * changes the rna depth to be within the margin, so otherwise the depth could continualy decrease. */
static bool edbm_extrude_destructive_calc(wmOperator *op, const bool allow_snapping)
{
  ExtrudeData *opdata;
  BMEditMesh *em;
  BMOperator bmop;
  bool changed = false;

  float depth = RNA_float_get(op->ptr, "depth");
  const bool original_loop = RNA_boolean_get(op->ptr, "original_loop");
  const bool keep_sides = RNA_boolean_get(op->ptr, "keep_sides");
  /* not passed onto the BMO */
  const bool destroy_end = RNA_boolean_get(op->ptr, "destroy_end");
  const bool iterate_loops = RNA_boolean_get(op->ptr, "iterate_loops");

  opdata = op->customdata;
  const bool snapping = opdata->snap && allow_snapping;
  float extrude_dir[3];
  copy_v3_v3(extrude_dir, opdata->extrude_dir);

  for (uint ob_index = 0; ob_index < opdata->ob_store_len; ob_index++) {
    em = opdata->ob_store[ob_index].em;

    if (opdata->is_modal) {
      EDBM_redo_state_restore(opdata->ob_store[ob_index].mesh_backup, em, false);
    }

    /* find snapping margin */
    float snap_dist = snapping * opdata->pixel_size * 15.0f;
    snap_dist = max_ff(0.001f, snap_dist);

    if (fabs(depth) < snap_dist) {
      /* snap to original location */
      depth = 0.0f;
    }

    EDBM_op_init(
            em,
            &bmop,
            op,
            "extrude_destructive faces=%hf "
            "depth=%f dir=%v original_loop=%b keep_sides=%b",
            BM_ELEM_SELECT,
            depth,
            extrude_dir,
            original_loop,
            keep_sides);
    BMO_op_exec(em->bm, &bmop);

    EDBM_flag_disable_all(em, BM_ELEM_SELECT);
    BMO_slot_buffer_hflag_enable(em->bm, bmop.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);

    float limit = BMO_slot_float_get(bmop.slots_out, "depth_limit");

    if (!EDBM_op_finish(em, &bmop, op, true)) {
      continue;
    }

    /* Snapping and successive loops are handled by the following iterating code. */

    float total_limit = limit; /* the total depth possible before having to do another iteration */
    if (fabs(depth) > fabs(total_limit) - snap_dist) {
      bool faces_sel = true; /* safely exit from any more iterations if no faces are selected*/

      if (fabs(depth) > fabs(total_limit) + snap_dist) {
        int count = 0;
        do {
          edbm_weld_extrusion(op, em, destroy_end);
          if (BM_mesh_elem_hflag_count_enabled(em->bm, BM_FACE, BM_ELEM_SELECT, false) == 0) {
            depth = total_limit;
            faces_sel = false;
            break;
          }

          BMOperator iterate_op;
          EDBM_op_init(
                  em,
                  &iterate_op,
                  op,
                  "extrude_destructive faces=%hf "
                  "depth=%f dir=%v original_loop=%b keep_sides=%b",
                  BM_ELEM_SELECT,
                  depth - total_limit,
                  extrude_dir,
                  iterate_loops,
                  keep_sides);
          BMO_op_exec(em->bm, &iterate_op);

          EDBM_flag_disable_all(em, BM_ELEM_SELECT);
          BMO_slot_buffer_hflag_enable(
                    em->bm, iterate_op.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);

          limit = BMO_slot_float_get(iterate_op.slots_out, "depth_limit");
          total_limit += limit;
          EDBM_op_finish(em, &iterate_op, op, true);

          count++;
        } while (fabs(depth) > fabs(total_limit) + snap_dist && count < 8);
      }

      if(fabs(depth) > fabs(total_limit) && faces_sel){
        /* snap by not iterating again to go the full depth */
        depth = total_limit;
        edbm_weld_extrusion(op, em, destroy_end);
      }
      else if(fabs(depth) > fabs(total_limit) - snap_dist && faces_sel){
        /* snap by extruding again to the limit */
        BMOperator snap_op;
        EDBM_op_init(
                em,
                &snap_op,
                op,
                "extrude_destructive faces=%hf "
                "depth=%f dir=%v original_loop=%b keep_sides=%b",
                BM_ELEM_SELECT,
                total_limit - depth,
                extrude_dir,
                false,
                keep_sides);
        BMO_op_exec(em->bm, &snap_op);
        EDBM_flag_disable_all(em, BM_ELEM_SELECT);
        BMO_slot_buffer_hflag_enable(
                  em->bm, snap_op.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);
        EDBM_op_finish(em, &snap_op, op, true);
        depth = total_limit;

        edbm_weld_extrusion(op, em, destroy_end);
      }
    }
    /* update rna to depth, which may have been changed because of snapping or destroy_end */
    RNA_float_set(op->ptr, "depth", depth);

    EDBM_update_generic(em, true, true);
    changed = true;
  }
  return changed;
}

static int edbm_extrude_destructive_exec(bContext *C, wmOperator *op)
{
  if (!edbm_extrude_destructive_init(C, op, false)) {
    return OPERATOR_CANCELLED;
  }

  if (!edbm_extrude_destructive_calc(op, false)) {
    edbm_extrude_destructive_exit(C, op);
    return OPERATOR_CANCELLED;
  }

  edbm_extrude_destructive_exit(C, op);
  return OPERATOR_FINISHED;
}

static int edbm_extrude_destructive_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  RegionView3D *rv3d = CTX_wm_region_view3d(C);
  ExtrudeData *opdata;
  float center_3d[3];

  if (!edbm_extrude_destructive_init(C, op, true)) {
    return OPERATOR_CANCELLED;
  }

  opdata = op->customdata;

  calculateTransformCenter(C, V3D_AROUND_CENTER_MEDIAN, center_3d, NULL);
  float mval[2];
  mval[0] = (float) event->mval[0];
  mval[1] = (float) event->mval[1];
  opdata->initial_val = dot_v2v2(mval, opdata->screen_extrude_dir);
  opdata->pixel_size = rv3d ? ED_view3d_pixel_size(rv3d, center_3d) : 1.0f;

  edbm_extrude_destructive_calc(op, false);

  edbm_extrude_destructive_update_header(op, C);

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static int edbm_extrude_destructive_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  ExtrudeData *opdata = op->customdata;
  const bool has_numinput = hasNumInput(&opdata->num_input);

  /* Modal numinput active, try to handle numeric inputs first... */
  if (event->val == KM_PRESS && has_numinput && handleNumInput(C, &opdata->num_input, event)) {
    float amounts[2] = {RNA_float_get(op->ptr, "depth"), 0};
    applyNumInput(&opdata->num_input, amounts);
    RNA_float_set(op->ptr, "depth", amounts[0]);

    if (edbm_extrude_destructive_calc(op, false)) {
      edbm_extrude_destructive_update_header(op, C);
      return OPERATOR_RUNNING_MODAL;
    }
    else {
      edbm_extrude_destructive_cancel(C, op);
      return OPERATOR_CANCELLED;
    }
  }
  else {
    bool handled = false;
    switch (event->type) {
      case ESCKEY:
      case RIGHTMOUSE:
        edbm_extrude_destructive_cancel(C, op);
        return OPERATOR_CANCELLED;

      case MOUSEMOVE:
        if (!has_numinput) {
          float mval[2];
          mval[0] = (float) event->mval[0];
          mval[1] = (float) event->mval[1];

          float amount;

          float current_val = dot_v2v2(mval, opdata->screen_extrude_dir);
          amount = ((current_val - opdata->initial_val) * opdata->pixel_size) /
                         opdata->max_obj_scale;

          /* Fake shift-transform... */
          if (opdata->shift) {
            amount = (amount - opdata->shift_amount) * 0.1f + opdata->shift_amount;
          }

          RNA_float_set(op->ptr, "depth", amount);

          if (edbm_extrude_destructive_calc(op, true)) {
            edbm_extrude_destructive_update_header(op, C);
          }
          else {
            edbm_extrude_destructive_cancel(C, op);
            return OPERATOR_CANCELLED;
          }
          handled = true;
        }
        break;

      case LEFTMOUSE:
      case PADENTER:
      case RETKEY:
        if ((event->val == KM_PRESS) ||
            ((event->val == KM_RELEASE) && RNA_boolean_get(op->ptr, "release_confirm"))) {
          edbm_extrude_destructive_calc(op, false);
          edbm_extrude_destructive_exit(C, op);
          return OPERATOR_FINISHED;
        }
        break;
      case LEFTSHIFTKEY:
      case RIGHTSHIFTKEY:
        if (event->val == KM_PRESS) {
          opdata->shift_amount = RNA_float_get(op->ptr, "depth");
          opdata->shift = true;
          handled = true;
        }
        else {
          opdata->shift_amount = 0.0f;
          opdata->shift = false;
          handled = true;
        }
        break;
      case DKEY:
        if (event->val == KM_PRESS) {
          const bool original_loop = RNA_boolean_get(op->ptr, "original_loop");
          RNA_boolean_set(op->ptr, "original_loop", !original_loop);
          if (edbm_extrude_destructive_calc(op, true)) {
            edbm_extrude_destructive_update_header(op, C);
          }
          else {
            edbm_extrude_destructive_cancel(C, op);
            return OPERATOR_CANCELLED;
          }
          handled = true;
        }
        break;
      case EKEY:
        if (event->val == KM_PRESS) {
          const bool loops = RNA_boolean_get(op->ptr, "iterate_loops");
          RNA_boolean_set(op->ptr, "iterate_loops", !loops);
          if (edbm_extrude_destructive_calc(op, true)) {
            edbm_extrude_destructive_update_header(op, C);
          }
          else {
            edbm_extrude_destructive_cancel(C, op);
            return OPERATOR_CANCELLED;
          }
          handled = true;
        }
        break;
      case LEFTCTRLKEY:
      case RIGHTCTRLKEY:
        if (event->val == KM_PRESS) {
          opdata->snap = true;
          handled = true;
        }
        else {
          opdata->snap = false;
          handled = true;
        }
        break;
    }

    /* Modal numinput inactive, try to handle numeric inputs last... */
    if (!handled && event->val == KM_PRESS && handleNumInput(C, &opdata->num_input, event)) {
      float amounts[2] = {RNA_float_get(op->ptr, "depth"), 0};
      applyNumInput(&opdata->num_input, amounts);
      RNA_float_set(op->ptr, "depth", amounts[0]);

      if (edbm_extrude_destructive_calc(op, true)) {
        edbm_extrude_destructive_update_header(op, C);
        return OPERATOR_RUNNING_MODAL;
      }
      else {
        edbm_extrude_destructive_cancel(C, op);
        return OPERATOR_CANCELLED;
      }
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

void MESH_OT_extrude_destructive(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Destructive Extrude";
  ot->idname = "MESH_OT_extrude_destructive";
  ot->description = "Destructively extrude faces";

  /* api callbacks */
  ot->invoke = edbm_extrude_destructive_invoke;
  ot->modal = edbm_extrude_destructive_modal;
  ot->exec = edbm_extrude_destructive_exec;
  ot->cancel = edbm_extrude_destructive_cancel;
  ot->poll = ED_operator_editmesh;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_GRAB_CURSOR_XY | OPTYPE_BLOCKING;

  /* properties */
  prop = RNA_def_float_distance(
      ot->srna, "depth", 0.0f, -1e12f, 1e12f, "Depth", "", -10.0f, 10.0f);
  RNA_def_property_ui_range(prop, -10.0f, 10.0f, 0.01, 4);

  RNA_def_boolean(
      ot->srna, "original_loop", true, "Original Loop", "Keep the loop of original edges");

  RNA_def_boolean(
      ot->srna, "iterate_loops", false, "Successive Loops", "Create a loops after passing edges");

  RNA_def_boolean(
      ot->srna, "destroy_end", true, "Destroy End", "Cut through outside mesh");

  RNA_def_boolean(
      ot->srna, "keep_sides", false, "Keep Sides", "Keep the quads formed from the extrusion. "
      "Note that this creates non-manifold meshes");

  prop = RNA_def_boolean(ot->srna, "release_confirm", 0, "Confirm on Release", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

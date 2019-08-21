#
# Copyright 2011-2019 Blender Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# <pep8 compliant>

import bpy
from bpy.types import Operator
from bpy.props import StringProperty

from bpy.app.translations import pgettext_tip as tip_


class CYCLES_OT_use_shading_nodes(Operator):
    """Enable nodes on a material, world or light"""
    bl_idname = "cycles.use_shading_nodes"
    bl_label = "Use Nodes"

    @classmethod
    def poll(cls, context):
        return (getattr(context, "material", False) or getattr(context, "world", False) or
                getattr(context, "light", False))

    def execute(self, context):
        if context.material:
            context.material.use_nodes = True
        elif context.world:
            context.world.use_nodes = True
        elif context.light:
            context.light.use_nodes = True

        return {'FINISHED'}


class CYCLES_OT_lightgroup_add(bpy.types.Operator):
    """Add a lightgroup"""
    bl_idname="cycles.lightgroup_add"
    bl_label="Add Lightgroup"

    def execute(self, context):
        view_layer = context.view_layer
        cycles_view_layer = view_layer.cycles
        lgs = cycles_view_layer.lightgroups

        new_lg = lgs.add()

        number = 0
        name = "Lightgroup"
        while name in lgs.keys():
            number = number + 1
            name = "Lightgroup.{:03d}".format(number)
        new_lg.name = name

        cycles_view_layer.active_lightgroup = len(lgs) - 1

        view_layer.update_render_passes()
        return {'FINISHED'}


class CYCLES_OT_lightgroup_remove(bpy.types.Operator):
    """Remove a lightgroup"""
    bl_idname="cycles.lightgroup_remove"
    bl_label="Remove Lightgroup"

    def execute(self, context):
        view_layer = context.view_layer
        cycles_view_layer = view_layer.cycles
        lgs = cycles_view_layer.lightgroups

        cycles_view_layer.lightgroups.remove(cycles_view_layer.active_lightgroup)

        if cycles_view_layer.active_lightgroup >= len(lgs):
            cycles_view_layer.active_lightgroup = max(len(lgs) - 1, 0)

        view_layer.update_render_passes()
        return {'FINISHED'}


class CYCLES_OT_lightgroup_move(bpy.types.Operator):
    bl_idname = "cycles.lightgroup_move"
    bl_label = "Move Lightgroup"

    direction: bpy.props.EnumProperty(
        name="Move Direction",
        description="Direction to move the active Lightgroup: UP (default) or DOWN",
        items=[
            ('UP', "Up", "", -1),
            ('DOWN', "Down", "", 1),
        ],
        default='UP',
        options={'HIDDEN'},
    )

    @classmethod
    def poll(cls, context):
        view_layer = context.view_layer
        cycles_view_layer = view_layer.cycles
        return len(cycles_view_layer.lightgroups) > 1

    def execute(self, context):
        view_layer = context.view_layer
        cycles_view_layer = view_layer.cycles

        active_idx = cycles_view_layer.active_lightgroup
        new_idx = active_idx + (-1 if self.direction == 'UP' else 1)

        if new_idx < 0 or new_idx >= len(cycles_view_layer.lightgroups):
            return {'FINISHED'}

        cycles_view_layer.lightgroups.move(active_idx, new_idx)
        cycles_view_layer.active_lightgroup = new_idx

        return {'FINISHED'}


class CYCLES_OT_denoise_animation(Operator):
    "Denoise rendered animation sequence using current scene and view " \
    "layer settings. Requires denoising data passes and output to " \
    "OpenEXR multilayer files"
    bl_idname = "cycles.denoise_animation"
    bl_label = "Denoise Animation"

    input_filepath: StringProperty(
        name='Input Filepath',
        description='File path for image to denoise. If not specified, uses the render file path and frame range from the scene',
        default='',
        subtype='FILE_PATH')

    output_filepath: StringProperty(
        name='Output Filepath',
        description='If not specified, renders will be denoised in-place',
        default='',
        subtype='FILE_PATH')

    def execute(self, context):
        import os

        preferences = context.preferences
        scene = context.scene
        view_layer = context.view_layer

        in_filepath = self.input_filepath
        out_filepath = self.output_filepath

        in_filepaths = []
        out_filepaths = []

        if in_filepath != '':
            # Denoise a single file
            if out_filepath == '':
                out_filepath = in_filepath

            in_filepaths.append(in_filepath)
            out_filepaths.append(out_filepath)
        else:
            # Denoise animation sequence with expanded frames matching
            # Blender render output file naming.
            in_filepath = scene.render.filepath
            if out_filepath == '':
                out_filepath = in_filepath

            # Backup since we will overwrite the scene path temporarily
            original_filepath = scene.render.filepath

            for frame in range(scene.frame_start, scene.frame_end + 1):
                scene.render.filepath = in_filepath
                filepath = scene.render.frame_path(frame=frame)
                in_filepaths.append(filepath)

                if not os.path.isfile(filepath):
                    scene.render.filepath = original_filepath
                    err_msg = tip_("Frame '%s' not found, animation must be complete") % filepath
                    self.report({'ERROR'}, err_msg)
                    return {'CANCELLED'}

                scene.render.filepath = out_filepath
                filepath = scene.render.frame_path(frame=frame)
                out_filepaths.append(filepath)

            scene.render.filepath = original_filepath

        # Run denoiser
        # TODO: support cancel and progress reports.
        import _cycles
        try:
            _cycles.denoise(preferences.as_pointer(),
                            scene.as_pointer(),
                            view_layer.as_pointer(),
                            input=in_filepaths,
                            output=out_filepaths)
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'FINISHED'}

        self.report({'INFO'}, "Denoising completed.")
        return {'FINISHED'}


class CYCLES_OT_merge_images(Operator):
    "Combine OpenEXR multilayer images rendered with different sample" \
    "ranges into one image with reduced noise"
    bl_idname = "cycles.merge_images"
    bl_label = "Merge Images"

    input_filepath1: StringProperty(
        name='Input Filepath',
        description='File path for image to merge',
        default='',
        subtype='FILE_PATH')

    input_filepath2: StringProperty(
        name='Input Filepath',
        description='File path for image to merge',
        default='',
        subtype='FILE_PATH')

    output_filepath: StringProperty(
        name='Output Filepath',
        description='File path for merged image',
        default='',
        subtype='FILE_PATH')

    def execute(self, context):
        in_filepaths = [self.input_filepath1, self.input_filepath2]
        out_filepath = self.output_filepath

        import _cycles
        try:
            _cycles.merge(input=in_filepaths, output=out_filepath)
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'FINISHED'}

        return {'FINISHED'}


classes = (
    CYCLES_OT_use_shading_nodes,
    CYCLES_OT_denoise_animation,
    CYCLES_OT_lightgroup_add,
    CYCLES_OT_lightgroup_remove,
    CYCLES_OT_lightgroup_move,
    CYCLES_OT_merge_images
)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)


def unregister():
    from bpy.utils import unregister_class
    for cls in classes:
        unregister_class(cls)

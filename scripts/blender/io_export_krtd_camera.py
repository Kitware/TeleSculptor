"""
Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.

This is a blender addon to export a camera animation path as a sequence of
KRTD camera files.
"""

bl_info = {
    "name": "Export KRTD Camera Path",
    "author": "Matt Leotta",
    "version": (0, 1),
    "blender": (2, 6, 2),
    "location": "File > Export > KRTD Camera Path (.py)",
    "description": "Export KRTD Camera Path (.py)",
    "category": "Import-Export"}


import bpy
import mathutils
from math import radians, log10
import os.path


def writeCameras(context, filepath, frame_start, frame_end,
                 frame_step=1, only_selected=False):
    scene = bpy.context.scene
    width = scene.render.resolution_x
    height = scene.render.resolution_y
    width *= scene.render.resolution_percentage
    height *= scene.render.resolution_percentage
    width //= 100
    height //= 100

    base_path, file_ext = os.path.splitext(filepath)

    for obj in scene.objects:
        if only_selected and not obj.select:
            continue
        if obj.type != 'CAMERA':
            continue
        camera = obj
        break

    frame_range = range(frame_start, frame_end + 1, frame_step)
    num_digits = int(log10(frame_end))+1

    for f in frame_range:
        scene.frame_set(f)

        fpath = base_path + (("-%0"+str(num_digits)+"d") % f) + file_ext
        fw = open(fpath, 'w').write

        focal_len = camera.data.lens * width / camera.data.sensor_width
        # write the camera calibration matrix
        fw("%g 0 %g\n" % (focal_len, width / 2.0))
        fw("0 %g %g\n" % (focal_len, height / 2.0))
        fw("0 0 1\n\n")

        mat = camera.matrix_world.copy()
        # get the camera center
        c = mat.to_translation()
        # get the rotation matrix
        R = mat.to_quaternion().to_matrix().transposed()
        Rflip = mathutils.Matrix.Rotation(radians(180), 3, 'X')
        R = Rflip * R
        # write the rotation matrix
        fw("%g %g %g\n" % R[0][:])
        fw("%g %g %g\n" % R[1][:])
        fw("%g %g %g\n\n" % R[2][:])
        # compute the translation vector
        t = -1 * (R * c)
        fw("%g %g %g\n\n" % t[:])
        # write the empty lens distortion coefficients
        fw("0\n")


from bpy.props import StringProperty, IntProperty, BoolProperty
from bpy_extras.io_utils import ExportHelper


class CameraExporter(bpy.types.Operator, ExportHelper):
    '''Save camera animation as a sequence of KRTD camera files'''
    bl_idname = "export_animation.krtd_cameras"
    bl_label = "Export KRTD Cameras"

    filename_ext = ".krtd"
    filter_glob = StringProperty(default="*.krtd", options={'HIDDEN'})

    frame_start = IntProperty(name="Start Frame",
            description="Start frame for export",
            default=1, min=1, max=300000)
    frame_end = IntProperty(name="End Frame",
            description="End frame for export",
            default=250, min=1, max=300000)
    frame_step = IntProperty(name="Frame Step",
            description="Frame step for export",
            default=250, min=1, max=300000)
    only_selected = BoolProperty(name="Only Selected",
            default=False)

    def execute(self, context):
        writeCameras(context, self.filepath, self.frame_start, self.frame_end,
                     self.frame_step, self.only_selected)
        return {'FINISHED'}

    def invoke(self, context, event):
        self.frame_start = context.scene.frame_start
        self.frame_end = context.scene.frame_end
        self.frame_step = context.scene.frame_step

        wm = context.window_manager
        wm.fileselect_add(self)
        return {'RUNNING_MODAL'}


def menu_export(self, context):
    import os
    default_path = os.path.splitext(bpy.data.filepath)[0] + ".krtd"
    self.layout.operator(CameraExporter.bl_idname,
                         text="KRTD Cameras (.krtd)").filepath = \
                         default_path


def register():
    bpy.utils.register_module(__name__)

    bpy.types.INFO_MT_file_export.append(menu_export)


def unregister():
    bpy.utils.unregister_module(__name__)

    bpy.types.INFO_MT_file_export.remove(menu_export)


if __name__ == "__main__":
    register()
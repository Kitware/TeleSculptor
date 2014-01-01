"""
Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.

This is a blender addon to all the import of KRTD camera files.
"""

bl_info = {
    "name": "Import KRTD Camera Path",
    "author": "Matt Leotta",
    "version": (0, 1),
    "blender": (2, 6, 2),
    "location": "File > Import > KRTD Camera Path (.py)",
    "description": "Import KRTD Camera Path (.py)",
    "category": "Import-Export"}


import bpy
import mathutils
from math import radians

def parseCameraKrtd(fin):
    """Parse a single camera in KRTD format from the file object.

    Returns a (K,R,t,d) tuple using numpy.matrix types where:

    - K is a 3x3 calibration matrix
    - R is a 3x3 rotation matrix
    - t is a 1x3 translation vector (transposed).
    - d is a 1xN distortion parameter vector
    """
    K1 = mathutils.Vector(map(float, next(fin).split()))
    K2 = mathutils.Vector(map(float, next(fin).split()))
    K3 = mathutils.Vector(map(float, next(fin).split()))
    K = mathutils.Matrix([K1, K2, K3])
    next(fin)
    R1 = mathutils.Vector(map(float, next(fin).split()))
    R2 = mathutils.Vector(map(float, next(fin).split()))
    R3 = mathutils.Vector(map(float, next(fin).split()))
    R = mathutils.Matrix([R1, R2, R3])
    Rflip = mathutils.Matrix.Rotation(radians(180), 3, 'X')
    R = Rflip * R
    R = R.transposed().to_4x4()
    next(fin)
    t = mathutils.Matrix.Translation(mathutils.Vector(map(float, next(fin).split())))
    next(fin)
    d = map(float, next(fin).split())
    return (K, R, t, d)


def readCameras(context, filepath):
    with open(filepath, 'r') as f:
        (K, R, t, d) = parseCameraKrtd(f)
        cam = bpy.data.cameras.new("KRTD")
        cam_ob = bpy.data.objects.new("KRTD", cam)
        cam_ob.matrix_world = R*t
        bpy.context.scene.objects.link(cam_ob)
        cam_ob.data.lens = K[0][0] / (2.0 * K[0][2]) * cam_ob.data.sensor_width


from bpy.props import StringProperty
from bpy_extras.io_utils import ImportHelper


class CameraImporter(bpy.types.Operator, ImportHelper):
    '''Load camera animation from a sequence of KRTD camera files'''
    bl_idname = "import_animation.krtd_cameras"
    bl_label = "Import KRTD Cameras"

    filename_ext = ".krtd"
    filter_glob = StringProperty(default="*.krtd", options={'HIDDEN'})

    def execute(self, context):
        readCameras(context, self.filepath)
        return {'FINISHED'}

    def invoke(self, context, event):
        wm = context.window_manager
        wm.fileselect_add(self)
        return {'RUNNING_MODAL'}


def menu_import(self, context):
    import os
    default_path = os.path.splitext(bpy.data.filepath)[0] + ".krtd"
    self.layout.operator(CameraImporter.bl_idname,
                         text="KRTD Cameras (.krtd)").filepath = \
                         default_path


def register():
    bpy.utils.register_module(__name__)

    bpy.types.INFO_MT_file_import.append(menu_import)


def unregister():
    bpy.utils.unregister_module(__name__)

    bpy.types.INFO_MT_file_import.remove(menu_import)


if __name__ == "__main__":
    register()
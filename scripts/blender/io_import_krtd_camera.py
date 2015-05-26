#ckwg +28
#Copyright 2013-2014 by Kitware, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither name of Kitware, Inc. nor the names of any contributors may be used
#    to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
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

from bpy.props import StringProperty, FloatProperty, BoolProperty
from bpy_extras.io_utils import ImportHelper


def parseCameraKrtd(fin):
    """Parse a single camera in KRTD format from the file object.

    Returns a (K,R,t,d) tuple using mathutils types where:

    - K is a 3x3 calibration matrix
    - R is a 3x3 rotation matrix
    - t is a 1x3 translation vector (transposed).
    - d is a 1xN distortion parameter list
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
    next(fin)
    t = mathutils.Vector(map(float, next(fin).split()))
    t = -1 * R.transposed() * t
    next(fin)
    d = map(float, next(fin).split())
    Rflip = mathutils.Matrix.Rotation(radians(180), 3, 'X')
    R = Rflip * R
    R = R.transposed()
    return (K, R, t, d)


def readCamera(context, filepath, scale):
    """Read a camera from a KRTD file and construct a blender camera object
    """
    with open(filepath, 'r') as f:
        (K, R, t, d) = parseCameraKrtd(f)
        t = scale * t
        cam = bpy.data.cameras.new("camera_KRTD")
        cam_ob = bpy.data.objects.new("KRTD", cam)
        cam_ob.matrix_world = mathutils.Matrix.Translation(t) * R.to_4x4()
        bpy.context.scene.objects.link(cam_ob)
        if K[0][2] != 0.0:
            cam_ob.data.lens = K[0][0] / (2.0 * K[0][2]) \
                               * cam_ob.data.sensor_width
            cam_ob.data.sensor_fit = 'HORIZONTAL'
            bpy.context.scene.render.resolution_x = 2.0 * K[0][2]
            bpy.context.scene.render.resolution_y = 2.0 * K[1][2]


def readCameraPath(context, files, scale):
    """Read a camera path from a sequence KRTD files
    """
    cam = bpy.data.cameras.new("camera_KRTD")
    cam_ob = bpy.data.objects.new("KRTD", cam)
    bpy.context.scene.objects.link(cam_ob)
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = len(files)
    for fnum, filepath in enumerate(files):
        with open(filepath, 'r') as f:
            (K, R, t, d) = parseCameraKrtd(f)
            t = scale * t
            cam_ob.matrix_world = mathutils.Matrix.Translation(t) * R.to_4x4()
            if K[0][2] != 0.0:
                cam_ob.data.lens = K[0][0] / (2.0 * K[0][2]) \
                                   * cam_ob.data.sensor_width
                cam_ob.data.sensor_fit = 'HORIZONTAL'
                bpy.context.scene.render.resolution_x = 2.0 * K[0][2]
                bpy.context.scene.render.resolution_y = 2.0 * K[1][2]
            cam.keyframe_insert("lens", frame=fnum)
            cam_ob.keyframe_insert("location", frame=fnum)
            cam_ob.keyframe_insert("rotation_euler", frame=fnum)


def readCameras(context, filepath, scale, load_all, make_path):
    """Read cameras from a KRTD files and construct a blender camera objects

    If load_all is false then load a single camera from filepath, otherwise
    load all files matching '*.krtd' in the filepath directory
    """
    if not load_all:
        readCamera(context, filepath, scale)
    else:
        import glob
        import os.path
        directory = os.path.dirname(filepath)
        files = sorted(glob.glob(os.path.join(directory, '*.krtd')))
        if make_path:
            readCameraPath(context, files, scale)
        else:
            for f in files:
                readCamera(context, f, scale)


class CameraImporter(bpy.types.Operator, ImportHelper):
    '''Load camera animation from a sequence of KRTD camera files'''
    bl_idname = "import_animation.krtd_cameras"
    bl_label = "Import KRTD Cameras"

    filename_ext = ".krtd"
    filter_glob = StringProperty(default="*.krtd", options={'HIDDEN'})
    filepath = StringProperty(
            name="File Path",
            description="Directory used for importing the file",
            maxlen=1024,
            subtype='FILE_PATH',
            )
    scale = FloatProperty(
            name = "Scale",
            description="Scale camera coordinate system",
            default = 1.0, min = 0.001, max = 1000.0
            )
    load_all = BoolProperty(
            name="Load all files in directory",
            default=False
            )
    make_path = BoolProperty(
            name="Make a camera path",
            default=False
            )

    def execute(self, context):
        readCameras(context, self.filepath, self.scale,
                    self.load_all, self.make_path)
        return {'FINISHED'}

    def invoke(self, context, event):
        wm = context.window_manager
        wm.fileselect_add(self)
        return {'RUNNING_MODAL'}


def menu_import(self, context):
    self.layout.operator(CameraImporter.bl_idname,
                         text="KRTD Cameras (.krtd)")


def register():
    bpy.utils.register_module(__name__)

    bpy.types.INFO_MT_file_import.append(menu_import)


def unregister():
    bpy.utils.unregister_module(__name__)

    bpy.types.INFO_MT_file_import.remove(menu_import)


if __name__ == "__main__":
    register()

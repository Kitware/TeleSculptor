#ckwg +28
# Copyright 2015 by Kitware, Inc. All Rights Reserved. Please refer to
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

## Author = 'jonathan.owens'

require_relative './krtd.rb'

def load_camera(file_path)
  model = Sketchup.active_model
  ents = model.active_entities
  pages = Sketchup.active_model.pages
  view = Sketchup.active_model.active_view
  
  krtd_ff = from_file(file_path)
  
  cam_point3d = Geom::Point3d.new( krtd_ff.position[0].m,
                                   krtd_ff.position[1].m,
                                   krtd_ff.position[2].m )
  target_point3d = Geom::Point3d.new( krtd_ff.target[0].m,
                                      krtd_ff.target[1].m,
                                      krtd_ff.target[2].m )
  up_point3d = Geom::Vector3d.new( krtd_ff.up[0], krtd_ff.up[1], krtd_ff.up[2] )
  
  new_cam = Sketchup::Camera.new( cam_point3d, target_point3d, up_point3d )
  # SketchUp does not allow perspective cameras with FOV < 1 degree.
  # The focal_length is also limited to less than 3000.
  # However, image_width is unconstrained.  This work around sets an
  # artificially small image_width such that when combined with a
  # focal_length of 1000 we get the desired FOV.
  if krtd_ff.fov_y < 1.0
    new_cam.image_width = 1000 / krtd_ff.focal_length_x * krtd_ff.y_dim
    new_cam.focal_length = 1000
  else
    new_cam.fov = krtd_ff.fov_y
  end
  new_cam.aspect_ratio = krtd_ff.x_dim / krtd_ff.y_dim
  view.camera = new_cam
  
  return new_cam
end



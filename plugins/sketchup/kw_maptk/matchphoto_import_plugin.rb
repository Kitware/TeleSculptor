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
## Author = 'david.russell'

require 'sketchup.rb'
#require_relative isn't supported in SketchUp 8.
require File.join(File.dirname(__FILE__), 'krtd_importer.rb')

class MatchphotoMaptkImporter < Sketchup::Importer
  # This method is not necessarily used by sketchup, but allows us to instantiate
  # an instance of this class for use in conjunction with other importers.
  def instantiate(krtd_prefix)
    @_krtd_prefix = krtd_prefix
    @_image_fps = Array.new
    @_krtd_fps = Array.new
  end

  def initialize
    @_krtd_prefix = nil
    @_image_fps = Array.new
    @_krtd_fps = Array.new
  end

  def description
    return "Read in a list of image filepaths (*.txt)"
  end

  def file_extension
    return "txt"
  end

  def id
    return "com.kitware.importers.matchphotomaptkimporter"
  end

  def supports_options?
    return false
  end

  def image_fps
    return @_image_fps
  end

  def krtd_fps
    return @_krtd_fps
  end

  def read_in_image_fps(fp)
    image_fps = Dir.entries(fp).reject {|f| File.directory? f}
    full_image_fps = Array.new
    image_fps.each do |image_fps|
      full = File.join(fp,image_fps)
      full_image_fps.push(full)
    end

    return full_image_fps
  end

  def check_for_krtd(img_fps, file_path, guess_krtd_location_flag)
    valid_img_fps = Array.new
    krtd_fps = Array.new
    num_bad_img_fps = 0
    num_bad_krtd_fps = 0
    img_fps.each do |img_fp|
      # if not a valid path, try prepending the directory of the image list file
      if ! File.file?(img_fp)
        img_fp = File.join(File.dirname(file_path), img_fp)
      end
      if ! File.file?(img_fp)
        num_bad_img_fps += 1
        next
      end

      krtd_fname = nil

      if guess_krtd_location_flag
        krtd_fname = guess_krtd_location(img_fp)
      else
        krtd_fname = File.join(@_krtd_prefix, swap_img_ext_for_krtd(img_fp))
      end

      if File.file?(krtd_fname)
        valid_img_fps.push(img_fp)
        krtd_fps.push(krtd_fname)
      else
        num_bad_krtd_fps += 1
      end
    end

    return [valid_img_fps,
           krtd_fps,
           num_bad_img_fps,
           num_bad_krtd_fps]
  end

  def make_camera_mesh(camera, length=1.0)
    zs = length
    ys = Math.tan(camera.fov * Math::PI / 360.0) * zs
    xs = ys * camera.aspect_ratio
    pm = Geom::PolygonMesh.new
    # add the pyramid
    pm.add_point([0, 0, 0])
    pm.add_point([xs, -ys, zs])
    pm.add_point([xs, ys, zs])
    pm.add_point([-xs, ys, zs])
    pm.add_point([-xs, -ys, zs])
    pm.add_polygon(5,4,3,2)
    pm.add_polygon(1,2,3)
    pm.add_polygon(1,3,4)
    pm.add_polygon(1,4,5)
    pm.add_polygon(1,5,2)
    # add the up triangle
    d = 0.2 * ys
    xs = 0.8 * xs
    pm.add_point([-xs, ys+d, zs])
    pm.add_point([xs, ys+d, zs])
    pm.add_point([0, 2*ys+d, zs])
    pm.add_polygon(6,7,8)

    x_axis = camera.up.cross(camera.target - camera.eye)
    tr = Geom::Transformation.new(camera.eye, x_axis, camera.up)
    pm.transform!(tr)
    return pm
  end

  def load_file(file_path, status)
    guess_krtd_location_flag = false
    @_list_fp = file_path
    if @_krtd_prefix.nil?
      UI.messagebox("You are trying to directly load in a list of .png files without "\
                    "specifying a prefix to the corresponding krtd files. I will try "\
                    "to find krtd files with the same prefix and filename.")
      guess_krtd_location_flag = true
    end

    model = Sketchup.active_model
    entities = model.entities
    pages = model.pages
    cam_layer = model.layers.add('Cameras')
    cam_layer.page_behavior = LAYER_IS_HIDDEN_ON_NEW_PAGES
    material = model.materials.add('camera')
    material.alpha = 0.1
    material.color = 'red'
    cam_group = entities.add_group
    cam_group.layer = cam_layer
    mesh = Geom::PolygonMesh.new
    smooth_flags = 0#Geom::PolygonMesh::NO_SMOOTH_OR_HIDE#

    raw_img_fps = read_in_image_fps(file_path).sort
    fps_conglomerate = check_for_krtd(raw_img_fps,file_path,guess_krtd_location_flag)
    img_fps  = fps_conglomerate[0]
    krtd_fps = fps_conglomerate[1]
    # if there are more than 10 images in the list, let the user select how many to use
    if img_fps.size > 10
      prompts = ["How many frames do you want to use?"]
      defaults = ["10"]
      # subtracting one to remove off-by-one error in following loop
      num_frames = (UI.inputbox(prompts, defaults, "Number of #{img_fps.size} available frames to use")[0].to_i) - 1

      new_img_fps = Array.new
      new_krtd_fps = Array.new
      for i in 0..num_frames
        idx = ((img_fps.size-1) * (i.to_f / num_frames)).to_i
        new_krtd_fps.push krtd_fps[idx]
        new_img_fps.push img_fps[idx]
      end
      krtd_fps = new_krtd_fps
      img_fps = new_img_fps
    end

    #for loop ranges are inclusive on both bounds
    for i in 0..(krtd_fps.length - 1)
      img_fp  = img_fps[i]
      krtd_fname = krtd_fps[i]
      new_cam = load_camera(krtd_fname)
      scale = new_cam.eye.distance(Geom::Point3d.new) / 3
      cam_mesh = make_camera_mesh(new_cam, scale)
      cam_group.entities.add_faces_from_mesh(cam_mesh, smooth_flags, material, material)
      page = pages.add_matchphoto_page(img_fp, camera = new_cam, page_name = File.basename(img_fp))
      page.transition_time = 0
    end

    if fps_conglomerate[3] != 0 || fps_conglomerate[2] !=0
      UI.messagebox("Failed to open #{fps_conglomerate[3]} krtd files and #{fps_conglomerate[2]} image files.")
    end

    #SketchUp 8 does not support pages.length
    if pages.count > 0
      pages.selected_page = pages[0]
      return 0
    else
      UI.messagebox("Failed to add any pages.")
      return 1
    end
  end

  # Only called if the @_krtd_prefix is not set
  def guess_krtd_location(s)
    return File.join(File.dirname(s), swap_img_ext_for_krtd(File.basename(s)))
  end

  def swap_img_ext_for_krtd(s)
    file_name = File.basename(s)
    img_ext_to_swap = File.extname(file_name)
    bname =  file_name.split(img_ext_to_swap)[0]
    return  "#{bname}.krtd"
  end

  def swap_krtd_ext_for(s, img_ext)
    # have to make sure there is a dot in the specified extension
    img_ext = img_ext[0] == "." ? img_ext : "." + img_ext
    return s.split(".krtd")[0] + img_ext
  end

end

Sketchup.register_importer(MatchphotoMaptkImporter.new)

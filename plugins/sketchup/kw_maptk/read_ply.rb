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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.erermer

## Author = 'jonathan.owens'

require 'sketchup.rb'

class PLYImporter < Sketchup::Importer
  def description
    return "ply point cloud ASCII files (*.ply)"
  end

  def file_extension
    return "ply"
  end

  def id
    return "com.kitware.importers.plyimporter"
  end

  def supports_options?
    return false
  end

  def load_file(file_path, status)
    file = File.new(file_path, "r")
    model = Sketchup.active_model
    pt_layer = model.layers.add('MAP-Tk Landmarks')
    pt_layer.page_behavior = LAYER_IS_HIDDEN_ON_NEW_PAGES
    entities = model.entities
    pt_group = entities.add_group
    pt_group.layer = pt_layer

    passed_end_header = false
    while (raw_line = file.gets)
      if passed_end_header == true
        string_coords = raw_line.strip.split
        pt = Geom::Point3d::new(string_coords[0].to_f.m,
                                string_coords[1].to_f.m,
                                string_coords[2].to_f.m)
        pt_group.entities.add_cpoint(pt)
      elsif raw_line.include? "end_header"
        passed_end_header = true
        next
      end
    end

    return 0
  end
end

Sketchup.register_importer(PLYImporter.new)

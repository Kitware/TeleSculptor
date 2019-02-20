#ckwg +28
# Copyright 2019 by Kitware, Inc. All Rights Reserved. Please refer to
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

## Author = 'matt.leotta'

require 'sketchup.rb'

class GeoJsonImporter < Sketchup::Importer
  def description
    return "geojson point cloud files (*.json)"
  end

  def file_extension
    return "json"
  end

  def id
    return "com.kitware.importers.jsonimporter"
  end

  def supports_options?
    return false
  end
    
  def load_file(file_path, layer_name, is_hidden)
    file = IO.read(file_path)
    model = Sketchup.active_model
    pt_layer = model.layers.add(layer_name)
    if is_hidden == true
      pt_layer.page_behavior = LAYER_IS_HIDDEN_ON_NEW_PAGES
    end
    entities = model.entities
    pt_group = entities.add_group
    pt_group.layer = pt_layer

    gcp_format = %r{
        "location"\s*:\s*\[
        (\s*[+-]?\d*\.?\d*e?[+-]?\d*),
        (\s*[+-]?\d*\.?\d*e?[+-]?\d*),
        (\s*[+-]?\d*\.?\d*e?[+-]?\d*)
        \s*\]
    }x

    gcp_values = file.scan(gcp_format)

    gcp_values.each do |loc|
      pt = Geom::Point3d::new(loc[0].to_f.m,
                              loc[1].to_f.m,
                              loc[2].to_f.m)
      pt_group.entities.add_cpoint(pt)
    end

    return 0
  end
end

Sketchup.register_importer(PLYImporter.new)

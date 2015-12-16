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

require 'sketchup.rb'
require_relative './krtd_importer.rb'

class MatchphotoMapTKImporter < Sketchup::Importer
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
    file = File.new(fp, "r")
    image_fps = Array.new

    while (line = file.gets)
      image_fps.push line.strip
    end
    file.close
    @_image_fps = image_fps
    return image_fps
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
    img_fps = read_in_image_fps(file_path)

    img_fps.each do |img_fp|

      # if not a valid path, try prepending the directory of the image list file
      if ! File.file?(img_fp)
        img_fp = File.join(File.dirname(file_path), img_fp)
      end

      krtd_fname = nil
      if guess_krtd_location_flag
        krtd_fname = guess_krtd_location(img_fp)
      else
        krtd_fname = File.join(@_krtd_prefix, swap_img_ext_for_krtd(img_fp))
      end

      if not File.exists?(krtd_fname)
        UI.messagebox("Opening the krtd file corresponding to the image " + File.basename(img_fp)\
                      + " failed. Skipping and moving on.")
        next
      end
      new_cam = load_camera(krtd_fname)

      pages.add_matchphoto_page(img_fp, camera = new_cam, page_name = File.basename(img_fp))
    end

    if pages.length > 0
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

Sketchup.register_importer(MatchphotoMapTKImporter.new)
      

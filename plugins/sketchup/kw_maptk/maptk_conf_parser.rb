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

## Author = "jonathan.owens"

require 'sketchup.rb'
#require_relative './read_ply.rb'
#require_relative './matchphoto_import_plugin.rb'
#taken from https://stackoverflow.com/questions/2681031/why-doesnt-relative-require-work-on-ruby-1-8-6, and seen in other files
require File.join(File.dirname(__FILE__),'read_ply.rb')
require File.join(File.dirname(__FILE__),'matchphoto_import_plugin.rb')


# These are the keywords that correspond to the relevant values of interest from the
# maptk configuration file and are the only lines we care about in this file.
IMAGE_FOLDER_KW = 'output_keyframes_dir'
OUTPUT_PLY_FILE_KW = 'output_ply_file'
OUTPUT_KRTD_DIR_KW = 'output_krtd_dir'

class MaptkConfImporter < Sketchup::Importer
  def description
    return "MAP-Tk configuration file (*.conf)"
  end

  def file_extension
    return "conf"
  end

  def id
    return "com.kitware.importers.maptkconfimporter"
  end

  def supports_options?
    return false
  end

  def get_kws_of_interest(fp)
    output_image_dir = ""
    output_ply_file = ""
    output_krtd_dir = ""

    conf_file = File.new(fp, "r")

    while (line = conf_file.gets)
      key_value = line.split("=")
      key = key_value[0].strip
      value = key_value[1] ? key_value[1].strip : ""

      case key
      when IMAGE_FOLDER_KW
        output_image_dir = value
      when OUTPUT_PLY_FILE_KW
        output_ply_file = value
      when OUTPUT_KRTD_DIR_KW
        output_krtd_dir = value
      end
    end

    # Check to ensure all of the required keywords were found and show a warning message
    # and return nil if they weren't
    if output_image_dir == ""
      UI.messagebox("Error parsing MAP-Tk conf file: missing #{IMAGE_FOLDER_KW} keyword")
      return nil
    elsif output_ply_file == ""
      UI.messagebox("Error parsing MAP-Tk conf file: missing #{OUTPUT_PLY_FILE_KW} keyword")
      return nil
    elsif output_krtd_dir == ""
      UI.messagebox("Error parsing MAP-Tk conf file: missing #{OUTPUT_KRTD_DIR_KW} keyword")
      return nil
    end
	
    # if not a valid path, try prepending the directory of the conf file
    if ! File.directory?(output_image_dir)
      output_image_dir = File.join(File.dirname(fp), output_image_dir)
      if ! File.directory?(output_image_dir)
        UI.messagebox("The value of #{IMAGE_FOLDER_KW} is incorrect. #{output_image_dir} is not a valid directory")
        return nil
      end
    end

    # if not a valid directory, try prepending the directory of the conf file
    if ! File.directory?(output_krtd_dir)
      output_krtd_dir = File.join(File.dirname(fp), output_krtd_dir)
      if ! File.directory?(output_krtd_dir)
        UI.messagebox("The value of #{OUTPUT_KRTD_DIR_KW} is incorrect. #{output_krtd_dir} is not a valid directory")
        return nil
      end
    end

    # if not a valid directory, try prepending the directory of the conf file
    if ! File.file?(output_ply_file)
      output_ply_file = File.join(File.dirname(fp), output_ply_file)
      if ! File.file?(output_ply_file)
        UI.messagebox("The value of #{OUTPUT_PLY_FILE_KW} is incorrect. #{output_ply_file} is not a valid file.")
        return nil
      end
    end

    return output_image_dir, output_ply_file, output_krtd_dir
  end

  def load_file(file_path, status)
    kwds = get_kws_of_interest(file_path)

    if kwds == nil
      # Signals to Sketchup that there was an import error. More info should be shown
      # at the point of discovery of the absence of a required keyword.
      return 1
    end

    output_image_folder = kwds[0]
    output_ply_file = kwds[1]
    output_krtd_dir = kwds[2]

    # We delegate the importing of the photos/krtd files to the matchphoto_import_plugin.
    photo_krtd_importer = MatchphotoMaptkImporter.new
    photo_krtd_importer.instantiate(output_krtd_dir)
    status_images = photo_krtd_importer.load_file(output_image_folder, 0)
    # And the ply importing to the PLYImporter plugin.
    ply_importer = PLYImporter.new
    status_ply = ply_importer.load_file(output_ply_file, 0)
    return 0
  end

  def get_file
    conf_file = UI.openpanel( "Open MAP-Tk Config File", "", "" )
    if conf_file
      load_file(conf_file, 0)
    end
  end

end

Sketchup.register_importer(MaptkConfImporter.new)

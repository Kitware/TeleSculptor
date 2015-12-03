## Author = 'jonathan.owens'

require 'sketchup.rb'
require_relative './krtd_importer.rb'

class MapTKImporter < Sketchup::Importer
  def description
    return "Read in list of krtd files to read in (*.krtdinfo)"
  end

  def file_extension
    return "krtdinfo"
  end

  def id
    return "com.sketchup.importers.maptkimporter"
  end

  def supports_options?
    return false
  end

  def read_in_image_fps(fp)
    file = File.new(fp, "r")
    image_fps = Array.new

    while (line = file.gets)
      image_fps.push line.strip
    end
    file.close
    # return
    return image_fps
  end

  # def startup(pts_file)
  #   puts "hi"
  # end
  
  def load_file(file_path, status)
    puts "here"
    model = Sketchup.active_model
    entities = model.entities
    pages = model.pages
    img_fps = read_in_image_fps(file_path)
    
    img_fps.each do |fp|
      new_cam, cam_center = load_camera(fp)
      vector = Geom::Vector3d.new 0,0,1
      vector2 = vector.normalize!
      entities.add_circle(cam_center, vector2, 0.01)
      pages.add_matchphoto_page(swap_krtd_ext_for(fp, '.png'), camera = new_cam, page_name = fp)
    end

    pages.selected_page = pages[0]
    # return
    return 0
  end

  def swap_krtd_ext_for(s, img_ext)
    # have to make sure there is a dot in the specified extension
    img_ext = img_ext[0] == "." ? img_ext : "." + img_ext
    return s.split(".krtd")[0] + img_ext
  end
    
end

Sketchup.register_importer(MapTKImporter.new)
      

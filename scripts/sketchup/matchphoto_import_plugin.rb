class MapTKImporter < Sketchup::Importer
  def description
    "Loads in scenes to Sketchup using maptk data."
  end

  def file_extension
    "dat"
  end

  def id
    "com.sketchup.importers.maptkimporter"
  end

  def supports_options?
    false
  end

  def read_in_image_fps(fp)
    file = File.new(fp, "r")
    image_fps = Array.new

    while (line = file.gets)
      image_fps.push line.strip
    end
    file.close
    image_fps
  end
  
  def load_file(file_path, status)
    model = Sketchup.active_model
    pages = model.pages
    img_fps = read_in_image_fps file_path

    img_fps.each do |fp|
      pages.add_matchphoto_page fp, camera = nil, page_name = fp
    end

    pages.selected_page = pages[0]

    0
  end

end

Sketchup.register_importer(MapTKImporter.new)
      

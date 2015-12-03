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
    return "com.sketchup.importers.plyimporter"
  end

  def supports_options?
    return false
  end

  def load_file(file_path, status)
    file = File.new(file_path, "r")
    model = Sketchup.active_model
    entities = model.entities
    
    passed_end_header = false
    while (raw_line = file.gets)
      if passed_end_header == true
        string_coords = raw_line.strip.split
        pt = Geom::Point3d::new(string_coords[0].to_f, string_coords[1].to_f, string_coords[2].to_f)
        entities.add_cpoint(pt)
      elsif raw_line.include? "end_header"
        passed_end_header = true
        next
      end
    end
      
    return 0
  end
end

Sketchup.register_importer(PLYImporter.new)

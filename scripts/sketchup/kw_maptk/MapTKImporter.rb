require 'sketchup.rb'

module KW

  module MapTKImporter
    # pwd for this plugin
    @dir = File.dirname(__FILE__)

    require File.join(@dir, "maptk_conf_parser.rb")
    require File.join(@dir, "matchphoto_import_plugin.rb")
  end
  
end

Sketchup.register_importer(MatchphotoMapTKImporter.new)
Sketchup.register_importer(MapTKConfImporter.new)
    

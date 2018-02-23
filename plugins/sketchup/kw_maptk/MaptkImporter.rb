require 'sketchup.rb'

module KW

  module MaptkImporter
    # pwd for this plugin
    @dir = File.dirname(__FILE__)

    require File.join(@dir, "maptk_conf_parser.rb")
    require File.join(@dir, "matchphoto_import_plugin.rb")
  end

  unless file_loaded?(__FILE__)
    @@kw_tools_menu = UI.menu("Plugins").add_submenu("Kitware")
    maptk_conf_import = MaptkConfImporter.new
		@@kw_tools_menu.add_item("Import MAP-Tk Config"){maptk_conf_import.get_file}
    @@kw_tools_menu.add_separator
		@@kw_tools_menu.add_item("KWIVER Website"){UI.openURL "http://www.kwiver.org"}
  end

  file_loaded(__FILE__)
end

Sketchup.register_importer(MatchphotoMaptkImporter.new)
Sketchup.register_importer(MaptkConfImporter.new)
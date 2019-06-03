require 'sketchup.rb'

module KW

  module TeleSculptorImporter
    # pwd for this plugin
    @dir = File.dirname(__FILE__)

    require File.join(@dir, "telesculptor_conf_parser.rb")
    require File.join(@dir, "matchphoto_import_plugin.rb")
  end

  unless file_loaded?(__FILE__)
    @@kw_tools_menu = UI.menu("Plugins").add_submenu("Kitware")
    telesculptor_conf_import = TeleSculptorConfImporter.new
    @@kw_tools_menu.add_item("Import TeleSculptor Project"){telesculptor_conf_import.get_file}
    @@kw_tools_menu.add_separator
    @@kw_tools_menu.add_item("KWIVER Website"){UI.openURL "http://www.kwiver.org"}
  end

  file_loaded(__FILE__)
end

Sketchup.register_importer(MatchphotoTeleSculptorImporter.new)
Sketchup.register_importer(TeleSculptorConfImporter.new)

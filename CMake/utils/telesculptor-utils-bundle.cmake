#
# TeleSculptor Bundle creation support
#

# Provide an override for gp_item_default_embedded_path function
# in GetPrerequisites.cmake used by fixup_bundle.  This allows
# MAP-Tk to copy libraries into lib instead of MacOS.
function(gp_item_default_embedded_path_override
         item default_embedded_path_var)
  if(APPLE)
    if(item MATCHES "\\.dylib$")
      set(path "@executable_path/../lib")
      set(${default_embedded_path_var} "${path}" PARENT_SCOPE)
    endif()
  endif()

endfunction()

# Provide an override for gp_resolved_file_type to correct
# fix-up bundle for Unix.
function(gp_resolved_file_type_override
         orig_file type_val)
  # This is an ugly hack.  For some reason fixup_bundle wants
  # libraries to be "local" (in the same directory as the executable)
  # and fails if they are "embedded" (same prefix).  This override
  # works around the problem by reporting "embedded" as "local"
  if(UNIX AND NOT APPLE AND "${${type_val}}" STREQUAL "embedded")
    set(${type_val} "local" PARENT_SCOPE)
  endif()
endfunction()

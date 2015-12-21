#
# MAPTK Bundle creation support
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

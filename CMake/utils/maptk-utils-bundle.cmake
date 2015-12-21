#
# MAPTK Bundle creation support
#

# Provide an override for gp_item_default_embedded_path function
# in GetPrerequisites.cmake used by fixup_bundle.  This allows
# MAP-Tk to copy libraries into lib instead of MacOS.
function(gp_item_default_embedded_path_override
         item default_embedded_path_var)

  set(path "@executable_path")
  set(overridden 0)
  if(APPLE)
    if(item MATCHES "\\.dylib$")
      set(path "@executable_path/../lib")
      set(overridden 1)
    endif()
    # Embed frameworks in the embedded "Frameworks" directory (sibling of MacOS):
    #
    if(NOT overridden)
      if(item MATCHES "[^/]+\\.framework/")
        set(path "@executable_path/../Frameworks")
        set(overridden 1)
      endif()
    endif()
  endif()

  set(${default_embedded_path_var} "${path}" PARENT_SCOPE)
endfunction()

# Installation logic for MAP-Tk CMake utilities
#
# Variables that modify function:
#
#   maptk_cmake_install_dir
#     - Directory to install files to
#
set(utils_dir "${CMAKE_CURRENT_LIST_DIR}")

install(
  FILES "${utils_dir}/maptk-utils.cmake"
        "${utils_dir}/FindPROJ.cmake"
        "${utils_dir}/FindEigen3.cmake"
  DESTINATION "${maptk_cmake_install_dir}"
  )
install(
  DIRECTORY "${utils_dir}/utils"
            "${utils_dir}/tools"
  DESTINATION "${maptk_cmake_install_dir}"
  )
install(
  DIRECTORY "${utils_dir}/templates/cxx"
  DESTINATION "${maptk_cmake_install_dir}/templates"
  )

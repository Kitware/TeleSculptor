# Installation logic for TeleSculptor CMake utilities
#
# Variables that modify function:
#
#   telesculptor_cmake_install_dir
#     - Directory to install files to
#
set(utils_dir "${CMAKE_CURRENT_LIST_DIR}")

install(
  FILES "${utils_dir}/telesculptor-utils.cmake"
  DESTINATION "${telesculptor_cmake_install_dir}"
  )
install(
  DIRECTORY "${utils_dir}/utils"
            "${utils_dir}/tools"
  DESTINATION "${telesculptor_cmake_install_dir}"
  )

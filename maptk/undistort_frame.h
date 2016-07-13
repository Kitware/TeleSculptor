#ifndef UNDISTORT_FRAME_H
#define UNDISTORT_FRAME_H

#endif // UNDISTORT_FRAME_H

#include <vital/types/camera.h>

#include <maptk/maptk_export.h>

template <class T> class vtkSmartPointer;
class vtkImageData;

namespace kwiver {
namespace maptk {


/// Undistort the input frame using the distortion coeffiscients from the input camera
/**
 * \param framePath path to the input distorted frame
 * \param cam input camera for undistortion
 * \return undistorted frame
 */

MAPTK_EXPORT
vtkSmartPointer<vtkImageData>
undistortFrame(const std::string framePath, vital::camera_sptr cam);


/// Undistort the input frames using the distortion coeffiscients from the input cameras
/**
 * \param framePathList input vector containing the paths to the distorted frames
 * \param camList input vector contianing the cameras for undistortion
 * \param outputDir path to the output undistorted frames directory
 */
MAPTK_EXPORT
void undistortFrames(const std::vector<std::string> framePathList,
                     const std::vector<vital::camera_sptr> camList,
                     const std::string outputDir);

} // end namespace maptk
} // end namespace kwiver

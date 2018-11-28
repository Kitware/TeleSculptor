/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAPTK_VTKMAPTKPOINTPLACER_H_
#define MAPTK_VTKMAPTKPOINTPLACER_H_

// VTK includes
#include <vtkPointPlacer.h>

// Forward declarations

class vtkMaptkPointPlacer : public vtkPointPlacer
{
public:
  vtkTypeMacro(vtkMaptkPointPlacer, vtkPointPlacer);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkPointPlacer* New();

  /**
   * Given a renderer and a display position in pixel coordinates,
   * compute the world position and orientation where this point
   * will be placed. This method is typically used by the
   * representation to place the point initially. A return value of 1
   * indicates that constraints of the placer are met.
   */
  virtual int ComputeWorldPosition(vtkRenderer* ren,
                                   double displayPos[2],
                                   double worldPos[3],
                                   double worldOrient[9]) override;

protected:
  vtkMaptkPointPlacer() = default;
  ~vtkMaptkPointPlacer() = default;

private:
  vtkMaptkPointPlacer(const vtkMaptkPointPlacer&) = delete;
  void operator=(const vtkMaptkPointPlacer) = delete;
};

#endif // vtkMaptkPointPlacer_h

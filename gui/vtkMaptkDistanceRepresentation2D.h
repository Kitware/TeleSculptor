/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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

#ifndef TELESCULPTOR_VTKMAPTKDISTANCEREPRESENTATION2D_H_
#define TELESCULPTOR_VTKMAPTKDISTANCEREPRESENTATION2D_H_

// VTK includes
#include <vtkDistanceRepresentation2D.h>

class vtkMaptkDistanceRepresentation2D : public vtkDistanceRepresentation2D
{
public:
  vtkTypeMacro(vtkMaptkDistanceRepresentation2D, vtkDistanceRepresentation2D);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkMaptkDistanceRepresentation2D* New();

  /**
   * Set the distance measurement
   */
  vtkSetMacro(Distance, double);

  /**
   * Override to use the pre-set distance measurement
   */
  void BuildRepresentation() override;

  /**
   * Set/Get whether to compute the distance label
   */
  vtkSetMacro(ComputeDistance, vtkTypeBool);
  vtkGetMacro(ComputeDistance, vtkTypeBool);
  vtkBooleanMacro(ComputeDistance, vtkTypeBool);

protected:
  vtkMaptkDistanceRepresentation2D() = default;
  ~vtkMaptkDistanceRepresentation2D() = default;

  vtkTypeBool ComputeDistance = true;

private:
  vtkMaptkDistanceRepresentation2D(
    const vtkMaptkDistanceRepresentation2D&) = delete;
  void operator=(const vtkMaptkDistanceRepresentation2D) = delete;
};

#endif

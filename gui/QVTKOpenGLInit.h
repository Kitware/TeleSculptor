/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#ifndef MAPTK_QVTKOPENGLINIT_H
#define MAPTK_QVTKOPENGLINIT_H

/*
 * Initialization class that creates a valid QSurfaceFormat i.e. OpenGL context
 * with the expected parameters for the QVTKOpenGLWidget.
 * Typical use case is to construct the class before constructing a
 * QApplication object in the main function.
 *
 * Typical usage for QVTKOpenGLInit is as follows:
 * @code{.cpp}
 *
 *   int main(int argc, char* argv[])
 *   {
 *     // Initialize before constructing the QApplication
 *     QVTKOpenGLInit init;
 *     // Construct the QApplication
 *     QApplication app(argc, argv);
 *     // Show the application (that uses QVTKOpenGLWidget)
 *     app.exec();
 *     return 0;
 *   }
 *
 * @endcode
 */

class QVTKOpenGLInit
{
public:
  QVTKOpenGLInit();
};

#endif // MAPTK_QVTKOPENGLINIT_H

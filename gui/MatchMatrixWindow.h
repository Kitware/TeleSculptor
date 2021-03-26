/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
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

#ifndef TELESCULPTOR_MATCHMATRIXWINDOW_H_
#define TELESCULPTOR_MATCHMATRIXWINDOW_H_

#include <qtGlobal.h>

#include <QMainWindow>

#include <Eigen/SparseCore>

#include <vital/vital_types.h>

class MatchMatrixWindowPrivate;

class MatchMatrixWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MatchMatrixWindow(QWidget* parent = nullptr,
                             Qt::WindowFlags flags = {});
  ~MatchMatrixWindow() override;

public slots:
  void setMatrix(Eigen::SparseMatrix<unsigned int> const&,
                 std::vector<kwiver::vital::frame_id_t> const&);

  void saveImage();
  void saveImage(QString const& path);

protected slots:
  void updateControls();
  void updateImage();
  void updateImageTransform();

private:
  QTE_DECLARE_PRIVATE_RPTR(MatchMatrixWindow)
  QTE_DECLARE_PRIVATE(MatchMatrixWindow)

  QTE_DISABLE_COPY(MatchMatrixWindow)
};

#endif

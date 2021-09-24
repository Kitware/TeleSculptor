// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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

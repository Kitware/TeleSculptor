/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include "MatchMatrixWindow.h"

#include "ui_MatchMatrixWindow.h"
#include "am_MatchMatrixWindow.h"

#include "MatchMatrixAlgorithms.h"

#include <vital/util/enumerate_matrix.h>

#include <vital/vital_foreach.h>

#include <qtGradient.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QFileDialog>

namespace // anonymous
{

//-----------------------------------------------------------------------------
enum Orientation
{
  Horizontal,
  Vertical,
  Diagonal,
};

//-----------------------------------------------------------------------------
enum valueAlgorithm
{
  Absolute,
  RelativeX,
  RelativeY,
  RelativeXY,
};

//-----------------------------------------------------------------------------
enum ScaleAlgorithm
{
  Linear,
  Logarithmic,
};

//-----------------------------------------------------------------------------
enum GradientPreset
{
  Gray,
  Royal,
  Ember,
  Ocean,
  Sunset,
  Carribean,
  Jet,
  Parula,
  Viridis,
  Earth,
  Blackbody,
};

//-----------------------------------------------------------------------------
template <typename T>
T sparseMax(Eigen::SparseMatrix<T> const& m)
{
  auto result = T(0);

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(m))
  {
    result = std::max(result, it.value());
  }

  return result;
}

//-----------------------------------------------------------------------------
qtGradient buildGradient(int id)
{
  auto gradient = qtGradient();

  switch (id)
  {
    case Royal:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(64, 32, 120));
      gradient.insertStop(0.55, QColor(160, 112, 112));
      gradient.insertStop(0.85, QColor(255, 232, 128));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case Ember:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(128, 0, 0));
      gradient.insertStop(0.50, QColor(255, 128, 0));
      gradient.insertStop(0.75, QColor(255, 255, 128));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case Ocean:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.15, QColor(0, 24, 92));
      gradient.insertStop(0.30, QColor(0, 56, 120));
      gradient.insertStop(0.50, QColor(0, 112, 160));
      gradient.insertStop(0.75, QColor(120, 188, 212));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case Sunset:
      gradient.insertStop(0.00, QColor(16, 56, 126));
      gradient.insertStop(0.25, QColor(116, 72, 176));
      gradient.insertStop(0.50, QColor(184, 116, 160));
      gradient.insertStop(0.70, QColor(224, 156, 144));
      gradient.insertStop(0.85, QColor(240, 200, 144));
      gradient.insertStop(1.00, QColor(244, 244, 184));
      return gradient;

    case Carribean:
      gradient.insertStop(0.00, QColor(0, 32, 96));
      gradient.insertStop(0.15, QColor(32, 60, 152));
      gradient.insertStop(0.30, QColor(32, 112, 176));
      gradient.insertStop(0.60, QColor(88, 192, 192));
      gradient.insertStop(0.80, QColor(216, 240, 192));
      gradient.insertStop(1.00, QColor(255, 255, 216));
      return gradient;

    case Jet:
      gradient.insertStop(0.00, QColor(0, 0, 128));
      gradient.insertStop(0.15, QColor(0, 0, 255));
      gradient.insertStop(0.35, QColor(0, 255, 255));
      gradient.insertStop(0.65, QColor(255, 255, 0));
      gradient.insertStop(0.85, QColor(255, 0, 0));
      gradient.insertStop(1.00, QColor(128, 0, 0));
      return gradient;

    case Parula:
      gradient.insertStop(0.00, QColor(52, 44, 140));
      gradient.insertStop(0.05, QColor(52, 60, 168));
      gradient.insertStop(0.10, QColor(28, 84, 212));
      gradient.insertStop(0.15, QColor(0, 104, 224));
      gradient.insertStop(0.25, QColor(20, 128, 212));
      gradient.insertStop(0.35, QColor(4, 160, 204));
      gradient.insertStop(0.50, QColor(48, 184, 160));
      gradient.insertStop(0.60, QColor(120, 192, 124));
      gradient.insertStop(0.75, QColor(212, 184, 88));
      gradient.insertStop(0.85, QColor(255, 192, 60));
      gradient.insertStop(1.00, QColor(248, 248, 16));
      return gradient;

    case Viridis:
      gradient.insertStop(0.00, QColor(68, 0, 84));
      gradient.insertStop(0.15, QColor(68, 52, 128));
      gradient.insertStop(0.30, QColor(52, 96, 140));
      gradient.insertStop(0.50, QColor(32, 144, 140));
      gradient.insertStop(0.65, QColor(44, 176, 124));
      gradient.insertStop(0.80, QColor(120, 208, 80));
      gradient.insertStop(1.00, QColor(255, 232, 36));
      return gradient;

    case Earth:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.02, QColor(3, 0, 88));
      gradient.insertStop(0.05, QColor(7, 10, 116));
      gradient.insertStop(0.15, QColor(24, 68, 120));
      gradient.insertStop(0.25, QColor(40, 114, 125));
      gradient.insertStop(0.30, QColor(48, 130, 121));
      gradient.insertStop(0.40, QColor(62, 145, 89));
      gradient.insertStop(0.45, QColor(68, 152, 73));
      gradient.insertStop(0.50, QColor(94, 160, 75));
      gradient.insertStop(0.60, QColor(142, 171, 86));
      gradient.insertStop(0.65, QColor(164, 178, 90));
      gradient.insertStop(0.70, QColor(183, 181, 94));
      gradient.insertStop(0.75, QColor(189, 169, 98));
      gradient.insertStop(0.80, QColor(197, 164, 114));
      gradient.insertStop(0.85, QColor(212, 176, 147));
      gradient.insertStop(0.90, QColor(227, 195, 182));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case Blackbody:
      gradient.insertStop(0.000, QColor(0, 0, 0));
      gradient.insertStop(0.150, QColor(152, 24, 0));
      gradient.insertStop(0.270, QColor(232, 44, 0));
      gradient.insertStop(0.400, QColor(255, 176, 40));
      gradient.insertStop(0.550, QColor(255, 232, 126));
      gradient.insertStop(0.750, QColor(255, 255, 255));
      gradient.insertStop(1.000, QColor(144, 172, 255));
      return gradient;

    default: // Gray
      return gradient;
  }
}

//-----------------------------------------------------------------------------
QImage gradientPreview(qtGradient const& gradient, int w = 64, int h = 12)
{
  auto image = QImage(w, h, QImage::Format_ARGB32);
  auto const s = 1.0 / (static_cast<double>(w) - 1.0);

  for (int i = 0; i < w; ++i)
  {
    auto const a = static_cast<double>(i) * s;
    auto const c = gradient.at(a).rgba();

    for (int j = 0; j < h; ++j)
    {
      image.setPixel(i, j, c);
    }
  }

  return image;
}

//-----------------------------------------------------------------------------
QImage buildHorizontalImage(
  qtGradient const& gradient,
  Eigen::SparseMatrix<uint> const& matrix,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = matrix.rows();
  auto minY = k;
  auto maxY = k;

  auto image = QImage(k, k*2, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(matrix))
  {
    auto const y = it.col() + k - 1 - it.row();
    minY = qMin(minY, y);
    maxY = qMax(maxY, y);

    auto const a = scaleAlgorithm(valueAlgorithm(matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(it.row(), y, c.rgba());
  }

  return image.copy(0, minY, k, maxY - minY + 1);
}

//-----------------------------------------------------------------------------
QImage buildVerticalImage(
  qtGradient const& gradient,
  Eigen::SparseMatrix<uint> const& matrix,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = matrix.rows();
  auto minX = k;
  auto maxX = k;

  auto image = QImage(k*2, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(matrix))
  {
    auto const x = it.row() + k - 1 - it.col();
    minX = qMin(minX, x);
    maxX = qMax(maxX, x);

    auto const a = scaleAlgorithm(valueAlgorithm(matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(x, it.col(), c.rgba());
  }

  return image.copy(minX, 0, maxX - minX + 1, k);
}

//-----------------------------------------------------------------------------
QImage buildDiagonalImage(
  qtGradient const& gradient,
  Eigen::SparseMatrix<uint> const& matrix,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = matrix.rows();
  auto image = QImage(k, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(matrix))
  {
    auto const a = scaleAlgorithm(valueAlgorithm(matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(it.row(), it.col(), c.rgba());
  }

  return image;
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class MatchMatrixWindowPrivate
{
public:
  void addGradient(GradientPreset, QString const& name);
  void persist(QString const& key, QComboBox* widget);

  Ui::MatchMatrixWindow UI;
  Am::MatchMatrixWindow AM;
  qtUiState uiState;

  Eigen::SparseMatrix<uint> matrix;
  uint maxValue;

  QImage image;
  QGraphicsScene scene;
};

QTE_IMPLEMENT_D_FUNC(MatchMatrixWindow)

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::addGradient(
  GradientPreset preset, QString const& name)
{
  auto const& gradient = buildGradient(preset);
  auto const& preview = QPixmap::fromImage(gradientPreview(gradient));

  auto const i = this->UI.color->count();
  this->UI.color->addItem(name);
  this->UI.color->setItemData(i, QIcon(preview), Qt::DecorationRole);
}

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::persist(QString const& key, QComboBox* widget)
{
  auto const item = new qtUiState::Item<int, QComboBox>(
    widget, &QComboBox::currentIndex, &QComboBox::setCurrentIndex);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
MatchMatrixWindow::MatchMatrixWindow(QWidget* parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags), d_ptr(new MatchMatrixWindowPrivate)
{
  QTE_D();

  this->setAttribute(Qt::WA_DeleteOnClose);

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->UI.menuView->insertAction(d->UI.actionShowStatusBar,
                               d->UI.optionsDock->toggleViewAction());

  d->UI.view->setScene(&d->scene);

  d->addGradient(Gray, "Gray");
  d->addGradient(Royal, "Royal");
  d->addGradient(Ember, "Ember");
  d->addGradient(Ocean, "Ocean");
  d->addGradient(Sunset, "Sunset");
  d->addGradient(Carribean, "Carribean");
  d->addGradient(Jet, "Jet");
  d->addGradient(Parula, "Parula");
  d->addGradient(Viridis, "Viridis");
  d->addGradient(Earth, "Earth");
  d->addGradient(Blackbody, "Blackbody");
  d->UI.color->setCurrentIndex(Viridis);

  // Set up UI persistence and restore previous state
  d->uiState.setCurrentGroup("MatchMatrixWindow");
  d->persist("Orientation", d->UI.orientation);
  d->persist("Values", d->UI.values);
  d->persist("Scale", d->UI.scale);
  d->persist("Color", d->UI.color);

  d->uiState.mapState("Window/state", this);
  d->uiState.mapGeometry("Window/geometry", this);
  d->uiState.restore();

  // Set up signals/slots
  connect(d->UI.actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));

  connect(d->UI.orientation, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.values, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.scale, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.color, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
}

//-----------------------------------------------------------------------------
MatchMatrixWindow::~MatchMatrixWindow()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::setMatrix(Eigen::SparseMatrix<uint> const& matrix)
{
  QTE_D();

  d->matrix = matrix;
  d->maxValue = sparseMax(matrix);

  this->updateImage();
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::saveImage()
{
  // TODO
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::saveImage(QString const& path)
{
  // TODO
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::updateImage()
{
  QTE_D();

  // Set up visualization
  QScopedPointer<AbstractValueAlgorithm> valueAlgorithm;
  QScopedPointer<AbstractScaleAlgorithm> scaleAlgorithm;
  auto const& gradient = buildGradient(d->UI.color->currentIndex());

  switch (d->UI.values->currentIndex())
  {
    case RelativeX:
      valueAlgorithm.reset(new RelativeXValueAlgorithm);
      break;
    case RelativeY:
      valueAlgorithm.reset(new RelativeYValueAlgorithm);
      break;
    case RelativeXY:
      valueAlgorithm.reset(new RelativeXYValueAlgorithm);
      break;
    default: // Absolute
      valueAlgorithm.reset(new AbsoluteValueAlgorithm);
      break;
  }

  auto const maxValue = valueAlgorithm->max(d->maxValue);
  switch (d->UI.scale->currentIndex())
  {
    case Logarithmic:
      scaleAlgorithm.reset(new LogarithmicScaleAlgorithm(maxValue));
      break;
    default: // Linear
      scaleAlgorithm.reset(new LinearScaleAlgorithm(maxValue));
      break;
  }

  // Build image
  switch (d->UI.orientation->currentIndex())
  {
    case Horizontal:
      d->image = buildHorizontalImage(gradient, d->matrix,
                                      *valueAlgorithm, *scaleAlgorithm);
      break;
    case Vertical:
      d->image = buildVerticalImage(gradient, d->matrix,
                                    *valueAlgorithm, *scaleAlgorithm);
      break;
    default: // Diagonal
      d->image = buildDiagonalImage(gradient, d->matrix,
                                    *valueAlgorithm, *scaleAlgorithm);
      break;
  }

  d->scene.clear();
  d->scene.addPixmap(QPixmap::fromImage(d->image));
}

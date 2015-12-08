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
#include <QtGui/QGraphicsSceneHoverEvent>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QImageWriter>
#include <QtGui/QMessageBox>

#include <cmath>

using std::floor;
using std::pow;

///////////////////////////////////////////////////////////////////////////////

namespace // anonymous
{

//BEGIN preset enumerations

//-----------------------------------------------------------------------------
enum Layout
{
  Horizontal,
  Vertical,
  Diagonal,
};

//-----------------------------------------------------------------------------
enum Orientation
{
  Matrix,
  Graph,
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
  Exponential,
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

//END preset enumerations

///////////////////////////////////////////////////////////////////////////////

//BEGIN miscellaneous helpers

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
QString supportedImageFilter()
{
  auto formats = QMap<QString, QString>();
  VITAL_FOREACH(auto const& format, QImageWriter::supportedImageFormats())
  {
    auto const type = QString::fromLatin1(format).toLower();
    auto const filter = QString("*.%1").arg(type);
    formats.insert(type, filter);
  }

  formats.remove("jpg");
  formats.remove("png");

  auto filters = QString("*.jpg *.png");
  VITAL_FOREACH (auto const& filter, formats)
  {
    filters += " " + filter;
  }

  return filters;
}

//END miscellaneous helpers

} // namespace <anonymous>

///////////////////////////////////////////////////////////////////////////////

//BEGIN MatchMatrixWindowPrivate

//-----------------------------------------------------------------------------
class MatchMatrixWindowPrivate
{
public:
  void addGradient(GradientPreset, QString const& name);
  void persist(QString const& key, QComboBox* widget);
  void persist(QString const& key, qtDoubleSlider* widget);

  void buildHorizontalImage(
    qtGradient const& gradient,
    AbstractValueAlgorithm const& valueAlgorithm,
    AbstractScaleAlgorithm const& scaleAlgorithm);
  void buildVerticalImage(
    qtGradient const& gradient,
    AbstractValueAlgorithm const& valueAlgorithm,
    AbstractScaleAlgorithm const& scaleAlgorithm);
  void buildDiagonalImage(
    qtGradient const& gradient,
    AbstractValueAlgorithm const& valueAlgorithm,
    AbstractScaleAlgorithm const& scaleAlgorithm);

  Ui::MatchMatrixWindow UI;
  Am::MatchMatrixWindow AM;
  qtUiState uiState;

  Eigen::SparseMatrix<uint> matrix;
  uint maxValue;

  QImage image;
  int offset;

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
void MatchMatrixWindowPrivate::persist(
  QString const& key, QComboBox* widget)
{
  auto const item = new qtUiState::Item<int, QComboBox>(
    widget, &QComboBox::currentIndex, &QComboBox::setCurrentIndex);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::persist(
  QString const& key, qtDoubleSlider* widget)
{
  auto const item = new qtUiState::Item<double, qtDoubleSlider>(
    widget, &qtDoubleSlider::value, &qtDoubleSlider::setValue);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::buildHorizontalImage(
  qtGradient const& gradient,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = this->matrix.rows();
  auto minY = k;
  auto maxY = k;

  auto image = QImage(k, k*2, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(this->matrix))
  {
    auto const y = it.col() + k - 1 - it.row();
    minY = qMin(minY, y);
    maxY = qMax(maxY, y);

    auto const a = scaleAlgorithm(valueAlgorithm(this->matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(it.row(), y, c.rgba());
  }

  this->image = image.copy(0, minY, k, maxY - minY + 1);
  this->offset = minY;
}

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::buildVerticalImage(
  qtGradient const& gradient,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = this->matrix.rows();
  auto minX = k;
  auto maxX = k;

  auto image = QImage(k*2, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(this->matrix))
  {
    auto const x = it.row() + k - 1 - it.col();
    minX = qMin(minX, x);
    maxX = qMax(maxX, x);

    auto const a = scaleAlgorithm(valueAlgorithm(this->matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(x, it.col(), c.rgba());
  }

  this->image = image.copy(minX, 0, maxX - minX + 1, k);
  this->offset = minX;
}

//-----------------------------------------------------------------------------
void MatchMatrixWindowPrivate::buildDiagonalImage(
  qtGradient const& gradient,
  AbstractValueAlgorithm const& valueAlgorithm,
  AbstractScaleAlgorithm const& scaleAlgorithm
)
{
  auto const k = this->matrix.rows();
  auto image = QImage(k, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  VITAL_FOREACH (auto it, kwiver::vital::enumerate(this->matrix))
  {
    auto const a = scaleAlgorithm(valueAlgorithm(this->matrix, it));
    auto const c = gradient.at(a);
    image.setPixel(it.row(), it.col(), c.rgba());
  }

  this->image = image;
  this->offset = 0;
}

//END MatchMatrixWindowPrivate

///////////////////////////////////////////////////////////////////////////////

//BEGIN MatchMatrixImageItem

//-----------------------------------------------------------------------------
class MatchMatrixImageItem : public QGraphicsPixmapItem
{
public:
  MatchMatrixImageItem(QImage const& image, MatchMatrixWindowPrivate* q);

protected:
  virtual void hoverEnterEvent(QGraphicsSceneHoverEvent* event) QTE_OVERRIDE;
  virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) QTE_OVERRIDE;
  virtual void hoverMoveEvent(QGraphicsSceneHoverEvent* event) QTE_OVERRIDE;

  void updateStatusText(QPointF const& pos);

  QTE_DECLARE_PUBLIC_PTR(MatchMatrixWindowPrivate);
  QTE_DECLARE_PUBLIC(MatchMatrixWindowPrivate);
};

//-----------------------------------------------------------------------------
MatchMatrixImageItem::MatchMatrixImageItem(
  QImage const& image, MatchMatrixWindowPrivate* q)
  : QGraphicsPixmapItem(QPixmap::fromImage(image)), q_ptr(q)
{
  this->setAcceptHoverEvents(true);
}

//-----------------------------------------------------------------------------
void MatchMatrixImageItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
  this->updateStatusText(event->pos());
  QGraphicsItem::hoverEnterEvent(event);
}

//-----------------------------------------------------------------------------
void MatchMatrixImageItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
  QTE_Q();

  q->UI.statusBar->clearMessage();

  QGraphicsItem::hoverLeaveEvent(event);
}

//-----------------------------------------------------------------------------
void MatchMatrixImageItem::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
{
  this->updateStatusText(event->pos());
  QGraphicsItem::hoverMoveEvent(event);
}

//-----------------------------------------------------------------------------
void MatchMatrixImageItem::updateStatusText(const QPointF& pos)
{
  QTE_Q();

  auto const k = q->matrix.rows();
  auto x = static_cast<int>(floor(pos.x()));
  auto y = static_cast<int>(floor(pos.y()));

  // Convert back to original matrix row/column
  switch (q->UI.layout->currentIndex())
  {
    case Horizontal:
      y = q->offset + x + y - (k - 1);
      break;

    case Vertical:
      x = q->offset + x + y - (k - 1);
      qSwap(x, y);
      break;

    case Diagonal:
      break;
  }

  // Test if we're within the data space
  if (x < 0 || x >= k || y < 0 || y >= k)
  {
    q->UI.statusBar->clearMessage();
    return;
  }

  // Show status text
  if (x == y)
  {
    static auto const format =
      QString("Frame %1 has %2 feature point(s)");

    q->UI.statusBar->showMessage(format.arg(x).arg(q->matrix.coeff(x, y)));
  }
  else
  {
    static auto const format =
      QString("Frames %1 (%3) and %2 (%4) have %5 correlated point(s)");

    auto const cx = q->matrix.coeff(x, x);
    auto const cy = q->matrix.coeff(y, y);
    auto const cxy = q->matrix.coeff(x, y);

    q->UI.statusBar->showMessage(format.arg(x).arg(y).arg(cx).arg(cy).arg(cxy));
  }
}

//END MatchMatrixImageItem

///////////////////////////////////////////////////////////////////////////////

//BEGIN MatchMatrixWindow

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
  d->persist("Layout", d->UI.layout);
  d->persist("Orientation", d->UI.orientation);
  d->persist("Values", d->UI.values);
  d->persist("Scale", d->UI.scale);
  d->persist("Exponent", d->UI.exponent);
  d->persist("Range", d->UI.range);
  d->persist("Color", d->UI.color);

  d->uiState.mapChecked("Window/status", d->UI.actionShowStatusBar);
  d->uiState.mapState("Window/state", this);
  d->uiState.mapGeometry("Window/geometry", this);
  d->uiState.restore();

  this->updateControls();
  this->updateImageTransform();

  // Set up signals/slots
  connect(d->UI.actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));

  connect(d->UI.layout, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.orientation, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImageTransform()));
  connect(d->UI.values, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.scale, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.color, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateImage()));
  connect(d->UI.exponent, SIGNAL(valueChanged(double)),
          this, SLOT(updateImage()));
  connect(d->UI.range, SIGNAL(valueChanged(double)),
          this, SLOT(updateImage()));

  connect(d->UI.values, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateControls()));
  connect(d->UI.scale, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(updateControls()));
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
  auto const path = QFileDialog::getSaveFileName(
    this, "Save Image", QString(),
    "Supported Images (" + supportedImageFilter() + ");;"
    "JPEG image (*.jpg *.jpeg);;"
    "PNG image (*.png);;"
    "All Files (*)");

  if (!path.isEmpty())
  {
    this->saveImage(path);
  }
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::saveImage(QString const& path)
{
  QTE_D();

  auto const flip = (d->UI.orientation->currentIndex() == Graph);
  auto const& image = (flip ? d->image.mirrored() : d->image);

  if (!image.save(path))
  {
    static auto const msgFormat = QString("Failed to write image to \"%1\".");
    QMessageBox::critical(this, "Error", msgFormat.arg(path));
  }
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::updateControls()
{
  QTE_D();

  auto const showExponent = (d->UI.scale->currentIndex() == Exponential);
  auto const showRange = (d->UI.values->currentIndex() != Absolute &&
                          d->UI.scale->currentIndex() == Logarithmic);

  auto const layout = qobject_cast<QFormLayout*>(d->UI.options->layout());

  d->UI.exponent->setVisible(showExponent);
  d->UI.exponentLabel->setVisible(showExponent);

  d->UI.range->setVisible(showRange);
  d->UI.rangeLabel->setVisible(showRange);

  // Remove widgets following scale (apparently this is the only way to get
  // QFormLayout to not include space for empty rows)
  layout->removeWidget(d->UI.exponent);
  layout->removeWidget(d->UI.exponentLabel);
  layout->removeWidget(d->UI.range);
  layout->removeWidget(d->UI.rangeLabel);
  layout->removeWidget(d->UI.color);
  layout->removeWidget(d->UI.colorLabel);

  // Re-add interesting widgets
  if (d->UI.scale->currentIndex() == Exponential)
  {
    // Show exponent
    layout->addRow(d->UI.exponentLabel, d->UI.exponent);
  }
  else if (d->UI.values->currentIndex() != Absolute &&
           d->UI.scale->currentIndex() == Logarithmic)
  {
    // Show range
    layout->addRow(d->UI.rangeLabel, d->UI.range);
  }

  layout->addRow(d->UI.colorLabel, d->UI.color);
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
    case Exponential:
      scaleAlgorithm.reset(
        new ExponentialScaleAlgorithm(maxValue, d->UI.exponent->value()));
      break;

    case Logarithmic:
      if (d->UI.values->currentIndex() == Absolute)
      {
        scaleAlgorithm.reset(new LogarithmicScaleAlgorithm(maxValue));
      }
      else
      {
        auto const range = pow(d->UI.range->value(), 2.0);
        scaleAlgorithm.reset(new LogarithmicScaleAlgorithm(maxValue, range));
      }
      break;

    default: // Linear
      scaleAlgorithm.reset(new LinearScaleAlgorithm(maxValue));
      break;
  }

  // Build image
  switch (d->UI.layout->currentIndex())
  {
    case Horizontal:
      d->buildHorizontalImage(gradient, *valueAlgorithm, *scaleAlgorithm);
      break;
    case Vertical:
      d->buildVerticalImage(gradient, *valueAlgorithm, *scaleAlgorithm);
      break;
    default: // Diagonal
      d->buildDiagonalImage(gradient, *valueAlgorithm, *scaleAlgorithm);
      break;
  }

  d->scene.clear();
  d->scene.addItem(new MatchMatrixImageItem(d->image, d));
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::updateImageTransform()
{
  QTE_D();

  auto const s = (d->UI.orientation->currentIndex() == Graph ? -1.0 : 1.0);
  d->UI.view->setTransform(QTransform::fromScale(1.0, s));
}

//END MatchMatrixWindow

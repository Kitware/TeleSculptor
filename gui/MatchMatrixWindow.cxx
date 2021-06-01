// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "MatchMatrixWindow.h"

#include "ui_MatchMatrixWindow.h"
#include "am_MatchMatrixWindow.h"

#include "MatchMatrixAlgorithms.h"

#include <vital/util/enumerate_matrix.h>

#include <qtGradient.h>
#include <qtIndexRange.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QFileDialog>
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsPixmapItem>
#include <QImageWriter>
#include <QMessageBox>

#include <cmath>

namespace kv = kwiver::vital;

using std::floor;
using std::pow;

///////////////////////////////////////////////////////////////////////////////

namespace // anonymous
{

//BEGIN option enumerations

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

//END option enumerations

///////////////////////////////////////////////////////////////////////////////

//BEGIN miscellaneous helpers

//-----------------------------------------------------------------------------
template <typename T>
T sparseMax(Eigen::SparseMatrix<T> const& m)
{
  auto result = T(0);

  foreach (auto it, kv::enumerate(m))
  {
    result = std::max(result, it.value());
  }

  return result;
}

//-----------------------------------------------------------------------------
QString supportedImageFilter()
{
  auto formats = QMap<QString, QString>();
  foreach (auto const& format, QImageWriter::supportedImageFormats())
  {
    auto const type = QString::fromLatin1(format).toLower();
    auto const filter = QString("*.%1").arg(type);
    formats.insert(type, filter);
  }

  formats.remove("jpg");
  formats.remove("png");

  auto filters = QString("*.jpg *.png");
  foreach (auto const& filter, formats)
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

  Eigen::SparseMatrix<unsigned int> matrix;
  unsigned int maxValue;
  std::vector<kv::frame_id_t> frames;

  QImage image;
  int offset;

  QGraphicsScene scene;
};

QTE_IMPLEMENT_D_FUNC(MatchMatrixWindow)

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
  AbstractScaleAlgorithm const& scaleAlgorithm)
{
  auto const k = this->matrix.rows();
  auto minY = k;
  auto maxY = k;

  auto image = QImage(k, k * 2, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  foreach (auto it, kv::enumerate(this->matrix))
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
  AbstractScaleAlgorithm const& scaleAlgorithm)
{
  auto const k = this->matrix.rows();
  auto minX = k;
  auto maxX = k;

  auto image = QImage(k * 2, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  foreach (auto it, kv::enumerate(this->matrix))
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
  AbstractScaleAlgorithm const& scaleAlgorithm)
{
  auto const k = this->matrix.rows();
  auto image = QImage(k, k, QImage::Format_RGB32);
  image.fill(gradient.at(0.0));

  foreach (auto it, kv::enumerate(this->matrix))
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
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;

  void updateStatusText(QPointF pos);

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
void MatchMatrixImageItem::updateStatusText(QPointF pos)
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

  kv::frame_id_t fx = q->frames[x];
  kv::frame_id_t fy = q->frames[y];

  // Show status text
  if (x == y)
  {
    static auto const format =
      QString("Frame %1 has %2 feature point(s)");

    q->UI.statusBar->showMessage(format.arg(fx).arg(q->matrix.coeff(x, y)));
  }
  else
  {
    static auto const format =
      QString("Frames %1 (%3) and %2 (%4) have %5 correlated point(s)");

    auto const cx = q->matrix.coeff(x, x);
    auto const cy = q->matrix.coeff(y, y);
    auto const cxy = q->matrix.coeff(x, y);

    q->UI.statusBar->showMessage(
      format.arg(fx).arg(fy).arg(cx).arg(cy).arg(cxy));
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

  d->UI.color->setCurrentIndex(GradientSelector::Viridis);

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
  connect(d->UI.actionSaveImage, &QAction::triggered,
          this, QOverload<>::of(&MatchMatrixWindow::saveImage));

  auto const indexChangedSignal =
    QOverload<int>::of(&QComboBox::currentIndexChanged);
  connect(d->UI.layout, indexChangedSignal,
          this, &MatchMatrixWindow::updateImage);
  connect(d->UI.orientation, indexChangedSignal,
          this, &MatchMatrixWindow::updateImageTransform);
  connect(d->UI.values, indexChangedSignal,
          this, &MatchMatrixWindow::updateImage);
  connect(d->UI.scale, indexChangedSignal,
          this, &MatchMatrixWindow::updateImage);
  connect(d->UI.color, indexChangedSignal,
          this, &MatchMatrixWindow::updateImage);
  connect(d->UI.exponent, &qtDoubleSlider::valueChanged,
          this, &MatchMatrixWindow::updateImage);
  connect(d->UI.range, &qtDoubleSlider::valueChanged,
          this, &MatchMatrixWindow::updateImage);

  connect(d->UI.values, indexChangedSignal,
          this, &MatchMatrixWindow::updateControls);
  connect(d->UI.scale, indexChangedSignal,
          this, &MatchMatrixWindow::updateControls);
}

//-----------------------------------------------------------------------------
MatchMatrixWindow::~MatchMatrixWindow()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void MatchMatrixWindow::setMatrix(Eigen::SparseMatrix<unsigned int> const& matrix,
                                  std::vector<kv::frame_id_t> const& frames)
{
  QTE_D();

  d->matrix = matrix;
  d->maxValue = sparseMax(matrix);
  d->frames = frames;

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
  auto const& gradient = d->UI.color->currentGradient();

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

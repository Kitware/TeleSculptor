// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "GroundControlPointsView.h"

#include "ui_GroundControlPointsView.h"
#include "am_GroundControlPointsView.h"

#include "GroundControlPointsHelper.h"
#include "GroundControlPointsModel.h"

#include <vital/types/geodesy.h>

#include <qtScopedValueChange.h>
#include <qtUtil.h>

#include <QClipboard>
#include <QDebug>
#include <QEvent>
#include <QFile>
#include <QMenu>
#include <QPainter>
#include <QScreen>
#include <QSvgRenderer>
#include <QToolButton>
#include <QWindow>

#include <limits>

namespace kv = kwiver::vital;

using id_t = kv::ground_control_point_id_t;

namespace
{

constexpr auto INVALID_POINT = std::numeric_limits<id_t>::max();

//-----------------------------------------------------------------------------
QPixmap colorize(QByteArray svg, int physicalSize, int logicalSize,
                 double devicePixelRatio, QColor color)
{
  svg.replace("#ffffff", color.name().toLatin1());

  QPixmap p{physicalSize, physicalSize};
  p.setDevicePixelRatio(devicePixelRatio);
  p.fill(Qt::transparent);

  QSvgRenderer renderer{svg};
  QPainter painter{&p};
  renderer.render(&painter, QRect{0, 0, logicalSize, logicalSize});

  return p;
}

}

//-----------------------------------------------------------------------------
class GroundControlPointsViewPrivate
{
public:
  void updateIcons(QWidget* widget);

  void enableControls(bool state, bool haveLocation = true);

  void showPoint(id_t id);
  void setPointPosition(id_t id);
  id_t selectedPoint() const;

  void copyLocation(bool northingFirst, bool includeElevation);

  Ui::GroundControlPointsView UI;
  Am::GroundControlPointsView AM;

  QMenu* popupMenu;
  QToolButton* copyLocationButton;

  QMetaObject::Connection screenChanged;

  GroundControlPointsModel model;
  GroundControlPointsHelper* helper = nullptr;

  id_t currentPoint = INVALID_POINT;
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsView)

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::updateIcons(QWidget* widget)
{
  QIcon icon;

  auto const& palette = widget->palette();
  auto const normalColor =
    palette.color(QPalette::Active, QPalette::Text);
  auto const selectedColor =
    palette.color(QPalette::Active, QPalette::HighlightedText);
  auto const disabledColor =
    palette.color(QPalette::Disabled, QPalette::Text);

  auto const dpr = widget->devicePixelRatioF();

  auto buildIcon = [&](QString const& resource){
    QFile f{resource};
    f.open(QIODevice::ReadOnly);
    auto const svg = f.readAll();

    for (auto const size : {16, 20, 22, 24, 32})
    {
      auto const dsize = static_cast<int>(size * dpr);

      icon.addPixmap(colorize(svg, dsize, size, dpr, normalColor),
                     QIcon::Normal);
      icon.addPixmap(colorize(svg, dsize, size, dpr, selectedColor),
                     QIcon::Selected);
      icon.addPixmap(colorize(svg, dsize, size, dpr, disabledColor),
                     QIcon::Disabled);
    }

    return icon;
  };

  this->model.setCameraIcon(
    buildIcon(QStringLiteral(":/icons/scalable/camera")));
  this->model.setSurveyedIcon(
    buildIcon(QStringLiteral(":/icons/scalable/surveyed")));
  this->model.setRegisteredIcon(
    buildIcon(QStringLiteral(":/icons/scalable/registered")));
}

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::enableControls(
  bool state, bool haveLocation)
{
  this->UI.easting->setEnabled(state);
  this->UI.northing->setEnabled(state);
  this->UI.elevation->setEnabled(state);

  this->UI.actionDelete->setEnabled(state);
  this->UI.actionRevert->setEnabled(state);
  this->UI.actionApplySimilarity->setEnabled(state);
  this->UI.actionCopyLocationLatLon->setEnabled(state && haveLocation);
  this->UI.actionCopyLocationLatLonElev->setEnabled(state && haveLocation);
  this->UI.actionCopyLocationLonLat->setEnabled(state && haveLocation);
  this->UI.actionCopyLocationLonLatElev->setEnabled(state && haveLocation);

  this->copyLocationButton->setEnabled(state);
}

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::showPoint(id_t id)
{
  if (this->helper && id != INVALID_POINT)
  {
    auto const& gcp = this->helper->groundControlPoint(id);
    if (gcp)
    {
      this->currentPoint = id;

      auto const& gl = gcp->geo_loc();
      auto const grl = [&gl]() -> kv::vector_3d {
        if (!gl.is_empty())
        {
          try
          {
            return gl.location(kv::SRID::lat_lon_WGS84);
          }
          catch (...)
          {
            qWarning() << "Geo-conversion from GCS" << gl.crs() << "failed";
          }
        }
        return { 0.0, 0.0, 0.0 };
      }();

      with_expr (qtScopedBlockSignals{this->UI.easting})
      {
        this->UI.easting->setValue(grl.x());
      }
      with_expr (qtScopedBlockSignals{this->UI.northing})
      {
        this->UI.northing->setValue(grl.y());
      }
      with_expr (qtScopedBlockSignals{this->UI.elevation})
      {
        this->UI.elevation->setValue(gcp->elevation());
      }

      this->enableControls(true, !gl.is_empty());

      return;
    }
  }

  this->currentPoint = INVALID_POINT;
  this->enableControls(false);
  return;
}

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::setPointPosition(id_t id)
{
  if (this->helper && id != INVALID_POINT)
  {
    auto const& gcp = this->helper->groundControlPoint(id);
    if (gcp)
    {
      auto const grl =
        kv::vector_2d{this->UI.easting->value(), this->UI.northing->value()};

      gcp->set_geo_loc({grl, kv::SRID::lat_lon_WGS84},
                       this->UI.elevation->value());
      gcp->set_geo_loc_user_provided(true);

      this->model.modifyPoint(id);
    }
  }
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsViewPrivate::selectedPoint() const
{
  auto const& s = this->UI.pointsList->selectionModel()->selectedIndexes();
  if (!s.isEmpty())
  {
    return this->model.id(s.first());
  }
  return INVALID_POINT;
}

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::copyLocation(
  bool northingFirst, bool includeElevation)
{
  auto const& gcp = this->helper->groundControlPoint(this->currentPoint);
  if (gcp)
  {
    auto const& gl = gcp->geo_loc();
    if (!gl.is_empty())
    {
      auto const grl = [&gl]() -> kv::vector_3d {
        try
        {
          return gl.location(kv::SRID::lat_lon_WGS84);
        }
        catch (...)
        {
          qWarning() << "Geo-conversion from GCS" << gl.crs() << "failed";
        }
        return { 0.0, 0.0, 0.0 };
      }();

      QStringList values;
      if (northingFirst)
      {
        values.append(QString::number(grl.y(), 'f', 9));
        values.append(QString::number(grl.x(), 'f', 9));
      }
      else
      {
        values.append(QString::number(grl.x(), 'f', 9));
        values.append(QString::number(grl.y(), 'f', 9));
      }
      if (includeElevation)
      {
        values.append(QString::number(gcp->elevation(), 'f', 3));
      }

      QApplication::clipboard()->setText(values.join(','));
    }
  }
}

//-----------------------------------------------------------------------------
GroundControlPointsView::GroundControlPointsView(
  QWidget* parent, Qt::WindowFlags flags)
  : QWidget{parent, flags}, d_ptr{new GroundControlPointsViewPrivate}
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->UI.pointsList->setModel(&d->model);
  d->UI.pointsList->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(d->UI.pointsList->selectionModel(),
          &QItemSelectionModel::selectionChanged,
          this, [d]{
            auto const id = d->selectedPoint();

            d->showPoint(id);

            if (d->helper)
            {
              d->helper->setActivePoint(id);
            }
          });
  connect(d->UI.pointsList, &QTreeView::activated, this,
          [d, this](QModelIndex const& index){
            auto const& parent = index.parent();
            if (parent.isValid())
            {
              auto const& fi = d->model.index(index.row(), 0, parent);
              auto const& data = d->model.data(fi, Qt::EditRole);
              if (data.isValid())
              {
                emit this->cameraRequested(data.value<kv::frame_id_t>());
              }
            }
          });

  d->updateIcons(this);

  auto const clText = QStringLiteral("Copy Location");

  auto* const clMenu = new QMenu{clText, this};
  clMenu->addAction(d->UI.actionCopyLocationLatLon);
  clMenu->addAction(d->UI.actionCopyLocationLatLonElev);
  clMenu->addAction(d->UI.actionCopyLocationLonLat);
  clMenu->addAction(d->UI.actionCopyLocationLonLatElev);

  d->copyLocationButton = new QToolButton{d->UI.toolBar};
  d->copyLocationButton->setText(clText);
  d->copyLocationButton->setToolTip(clText);
  d->copyLocationButton->setIcon(
    qtUtil::standardActionIcon(QStringLiteral("copy-location")));
  d->copyLocationButton->setMenu(clMenu);
  d->copyLocationButton->setPopupMode(QToolButton::InstantPopup);

  auto* const spacer = new QWidget{d->UI.toolBar};
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  d->UI.toolBar->insertWidget(d->UI.actionApplySimilarity, d->copyLocationButton);
  d->UI.toolBar->insertWidget(d->UI.actionApplySimilarity, spacer);

  d->popupMenu = new QMenu{this};
  d->popupMenu->addMenu(clMenu);
  d->popupMenu->addAction(d->UI.actionRevert);
  d->popupMenu->addAction(d->UI.actionDelete);

  connect(d->UI.pointsList, &QWidget::customContextMenuRequested,
          this, [d](QPoint const& pt){
            auto const i = d->UI.pointsList->indexAt(pt);
            if (i.isValid())
            {
              auto const gp = d->UI.pointsList->viewport()->mapToGlobal(pt);
              d->popupMenu->exec(gp);
            }
          });

  connect(d->UI.actionDelete, &QAction::triggered,
          this, [d]{
            if (d->helper && d->currentPoint != INVALID_POINT)
            {
              d->helper->removePoint(d->currentPoint);
            }
          });

  connect(d->UI.actionRevert, &QAction::triggered,
          this, [d]{
            if (d->helper && d->currentPoint != INVALID_POINT)
            {
              d->helper->resetPoint(d->currentPoint);
              d->model.modifyPoint(d->currentPoint);
            }
          });

  connect(d->UI.actionApplySimilarity, &QAction::triggered,
          this, [d]{
            d->helper->applySimilarityTransform();
          });

  connect(d->UI.actionCopyLocationLatLon, &QAction::triggered,
          this, [d]{ d->copyLocation(true, false); });
  connect(d->UI.actionCopyLocationLatLonElev, &QAction::triggered,
          this, [d]{ d->copyLocation(true, true); });
  connect(d->UI.actionCopyLocationLonLat, &QAction::triggered,
          this, [d]{ d->copyLocation(false, false); });
  connect(d->UI.actionCopyLocationLonLatElev, &QAction::triggered,
          this, [d]{ d->copyLocation(false, true); });

  auto const dsvc = QOverload<double>::of(&QDoubleSpinBox::valueChanged);
  auto const spp = [d]{ d->setPointPosition(d->currentPoint); };
  connect(d->UI.easting, dsvc, this, spp);
  connect(d->UI.northing, dsvc, this, spp);
  connect(d->UI.elevation, dsvc, this, spp);

  d->enableControls(false);
}

//-----------------------------------------------------------------------------
GroundControlPointsView::~GroundControlPointsView()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::setHelper(GroundControlPointsHelper* helper)
{
  QTE_D();

  if (d->helper)
  {
    disconnect(d->helper, nullptr, this, nullptr);
    disconnect(d->helper, nullptr, &d->model, nullptr);
  }

  d->helper = helper;

  connect(helper, &GroundControlPointsHelper::pointChanged,
          this, [d](id_t id){
            if (d->currentPoint == id)
            {
              d->showPoint(id);
            }
            d->model.modifyPoint(id);
          });
  connect(helper, &GroundControlPointsHelper::pointsRecomputed,
          this, [d](){
            if (d->currentPoint != INVALID_POINT)
            {
              d->showPoint(d->currentPoint);
            }
          });

  connect(helper, &GroundControlPointsHelper::activePointChanged,
          this, [d](id_t id){
            if (d->currentPoint != id)
            {
              constexpr auto flags =
                QItemSelectionModel::ClearAndSelect |
                QItemSelectionModel::Current | QItemSelectionModel::Rows;

              d->showPoint(id);

              auto const& index = d->model.find(id);
              d->UI.pointsList->selectionModel()->select(index, flags);
            }
          });

  connect(helper, &GroundControlPointsHelper::pointAdded,
          &d->model, &GroundControlPointsModel::addPoint);
  connect(helper, &GroundControlPointsHelper::pointRemoved,
          &d->model, &GroundControlPointsModel::removePoint);
  connect(helper, &GroundControlPointsHelper::pointsReloaded,
          &d->model, &GroundControlPointsModel::resetPoints);

  d->model.setDataSource(helper);
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::setActiveCamera(qint64 id)
{
  QTE_D();
  d->model.setActiveCamera(id);
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::shiftSelection(int offset)
{
  QTE_D();

  auto* const sm = d->UI.pointsList->selectionModel();
  auto const& s = sm->selectedIndexes();
  if (!s.isEmpty())
  {
    auto const& i = s.first();
    auto const& p = i.parent();
    auto const r = (p.isValid() ? p.row() : i.row());

    auto const n = d->model.index(r + offset, 0);
    if (n.isValid())
    {
      constexpr auto flags =
        QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Current |
        QItemSelectionModel::Rows;

      sm->select(n, flags);
    }
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::changeEvent(QEvent* e)
{
  if (e && e->type() == QEvent::PaletteChange)
  {
    QTE_D();
    d->updateIcons(this);
  }

  QWidget::changeEvent(e);
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::showEvent(QShowEvent* e)
{
  QTE_D();

  disconnect(d->screenChanged);
  d->screenChanged =
    connect(this->window()->windowHandle(), &QWindow::screenChanged,
            this, [d, this] { d->updateIcons(this); });

  QWidget::showEvent(e);
}

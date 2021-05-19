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

#include "GroundControlPointsModel.h"

#include "GroundControlPointsHelper.h"

#include <qtGet.h>
#include <qtIndexRange.h>
#include <qtStlUtil.h>

#include <QDebug>
#include <QIcon>

#include <limits>

namespace kv = kwiver::vital;

namespace
{

using id_t = kv::ground_control_point_id_t;
using gcp_sptr = GroundControlPointsHelper::gcp_sptr;
using crt_sptr = GroundControlPointsHelper::crt_sptr;

constexpr auto IndexIsValid =
  QAbstractItemModel::CheckIndexOption::IndexIsValid;

//-----------------------------------------------------------------------------
enum
{
  COLUMN_ID,
  COLUMN_NAME,
  COLUMNS
};

//-----------------------------------------------------------------------------
struct gcp_ref
{
  id_t id;
  kv::ground_control_point_sptr gcp;
  kv::track_sptr crt;

  QVector<kv::frame_id_t> crtFrames;

  bool operator<(gcp_ref const& other) const
  { return this->id < other.id; }

  operator id_t() const
  { return this->id; }

  kv::ground_control_point* operator->() const
  { return this->gcp.get(); }
};

//-----------------------------------------------------------------------------
gcp_ref buildPoint(id_t id, gcp_sptr const& gcp, crt_sptr const& crt)
{
  auto crtFrames = QVector<kv::frame_id_t>{};
  if (crt)
  {
    for (auto const& s : *crt)
    {
      crtFrames.append(s->frame());
    }
  }

  return {id, gcp, crt, crtFrames};
}

//-----------------------------------------------------------------------------
int rowForIndex(QModelIndex const& index)
{
  if (auto const data = index.internalId())
  {
    return static_cast<int>(data - 1);
  }
  return index.row();
}

} // namespace (anonymous)

//-----------------------------------------------------------------------------
class GroundControlPointsModelPrivate
{
public:
  GroundControlPointsHelper* helper = nullptr;
  QVector<gcp_ref> points;

  QIcon registeredIcon;
  QIcon surveyedIcon;
  QIcon cameraIcon;
  QIcon emptyIcon;

  kv::frame_id_t activeCamera = -1;
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsModel)

//-----------------------------------------------------------------------------
GroundControlPointsModel::GroundControlPointsModel(QObject* parent)
  : QAbstractItemModel{parent}, d_ptr{new GroundControlPointsModelPrivate}
{
}

//-----------------------------------------------------------------------------
GroundControlPointsModel::~GroundControlPointsModel()
{
}

//-----------------------------------------------------------------------------
id_t GroundControlPointsModel::id(QModelIndex const& index) const
{
  QTE_D();

  auto const r = rowForIndex(index);
  if (r < 0 || r > d->points.count())
  {
    return std::numeric_limits<id_t>::max();
  }

  return d->points[r].id;
}

//-----------------------------------------------------------------------------
QModelIndex GroundControlPointsModel::find(id_t id, int column) const
{
  QTE_D();

  auto const begin = d->points.begin();
  auto const end = d->points.end();

  auto const i = std::lower_bound(begin, end, id);
  if (i != end)
  {
    auto const r = static_cast<int>(i - begin);
    return this->index(r, column, {});
  }

  return {};
}

//-----------------------------------------------------------------------------
int GroundControlPointsModel::rowCount(const QModelIndex& parent) const
{
  QTE_D();
  if (parent.isValid())
  {
    auto const& item = d->points[parent.row()];
    return item.crtFrames.count();
  }

  return d->points.count();
}

//-----------------------------------------------------------------------------
int GroundControlPointsModel::columnCount(const QModelIndex& parent) const
{
  return COLUMNS;
}

//-----------------------------------------------------------------------------
QModelIndex GroundControlPointsModel::index(
  int row, int column, QModelIndex const& parent) const
{
  if (column < 0 || column > this->columnCount(parent) ||
      row < 0 || row >= this->rowCount(parent))
  {
    return {};
  }

  auto const data =
    (parent.isValid() ? static_cast<unsigned>(parent.row() + 1) : 0);
  return this->createIndex(row, column, data);
}

//-----------------------------------------------------------------------------
QModelIndex GroundControlPointsModel::parent(QModelIndex const& child) const
{
  if (auto const data = child.internalId())
  {
    auto const row = static_cast<int>(data - 1);
    return this->createIndex(row, 0, quintptr{0});
  }

  return {};
}

//-----------------------------------------------------------------------------
QVariant GroundControlPointsModel::data(
  QModelIndex const& index, int role) const
{
  if (!this->checkIndex(index, IndexIsValid))
  {
    return {};
  }

  QTE_D();

  if (auto const data = index.internalId())
  {
    if (index.column() == COLUMN_ID)
    {
      auto const& item = d->points[static_cast<int>(data - 1)];

      switch (role)
      {
        case Qt::DisplayRole:
        case Qt::EditRole:
          return QVariant::fromValue(item.crtFrames[index.row()]);

        case Qt::DecorationRole:
          return d->cameraIcon;

        case Qt::TextAlignmentRole:
          return int{Qt::AlignRight | Qt::AlignVCenter};

        default:
          return {};
      }
    }

    return {};
  }

  auto const& item = d->points[index.row()];

  switch (index.column())
  {
    case COLUMN_ID:
      switch (role)
      {
        case Qt::DisplayRole:
          if (!item.gcp)
          {
            static const auto t = QStringLiteral("(%1)\u2003");
            return t.arg(item.id);
          }
          else
          {
            static const auto t = QStringLiteral("%1\u2007\u2003");
            return t.arg(item.id);
          }

        case Qt::EditRole:
          return item.id;

        case Qt::TextAlignmentRole:
          return int{Qt::AlignRight | Qt::AlignVCenter};

        case Qt::DecorationRole:
          if (item.crt && item.crt->contains(d->activeCamera))
          {
            return d->registeredIcon;
          }
          return d->emptyIcon;

        default:
          return {};
      }

    case COLUMN_NAME:
      if (item.gcp)
      {
        switch (role)
        {
          case Qt::DisplayRole:
          case Qt::EditRole:
            return qtString(item.gcp->name());

          case Qt::DecorationRole:
            if (item.gcp->is_geo_loc_user_provided())
            {
              return d->surveyedIcon;
            }
            return d->emptyIcon;

          default:
            return {};
        }
      }
      return {};

    default:
      return {};
  }
}

//-----------------------------------------------------------------------------
Qt::ItemFlags GroundControlPointsModel::flags(QModelIndex const& index) const
{
  auto const baseFlags = QAbstractItemModel::flags(index);

  if (this->checkIndex(index, IndexIsValid))
  {
    if (index.internalId())
    {
      return baseFlags | Qt::ItemNeverHasChildren;
    }
    else
    {
      if (index.column() == COLUMN_NAME)
      {
        return baseFlags | Qt::ItemIsEditable;
      }
    }
  }

  return baseFlags;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsModel::setData(
  QModelIndex const& index, QVariant const& value, int role)
{
  if (!this->checkIndex(index, IndexIsValid) ||
      index.internalId() || index.column() != COLUMN_NAME)
  {
    return false;
  }

  QTE_D();

  auto const& gcp = d->points[index.row()].gcp;
  if (!gcp)
  {
    qDebug() << "Eek! Missing ground control point for index"
             << index << "with ID" << d->points[index.row()].id << "?!?!";
    return false;
  }

  switch (role)
  {
    case Qt::DisplayRole:
    case Qt::EditRole:
      gcp->set_name(stdString(value.toString()));
      emit this->dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole});
      return true;

    default:
      return false;
  }
}

//-----------------------------------------------------------------------------
QVariant GroundControlPointsModel::headerData(
  int section, Qt::Orientation orientation, int role) const
{
  if (role != Qt::DisplayRole || orientation != Qt::Horizontal)
  {
    return QAbstractItemModel::headerData(section, orientation, role);
  }

  switch (section)
  {
    case COLUMN_ID:
      return QStringLiteral("ID");
    case COLUMN_NAME:
      return QStringLiteral("Name");
    default:
      return {};
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::addPoint(id_t id)
{
  QTE_D();

  auto const begin = d->points.begin();
  auto const end = d->points.end();

  auto const i = std::upper_bound(begin, end, id);
  if (i != begin && (i - 1)->id == id)
  {
    this->modifyPoint(id);
    return;
  }

  auto const& gcp = d->helper->groundControlPoint(id);
  auto const& crt = d->helper->registrationTrack(id);

  auto const r = static_cast<int>(i - begin);
  this->beginInsertRows({}, r, r);
  d->points.insert(i, buildPoint(id, gcp, crt));
  this->endInsertRows();
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::removePoint(id_t id)
{
  QTE_D();

  auto const begin = d->points.begin();
  auto const end = d->points.end();

  auto const i = std::lower_bound(begin, end, id);
  if (i != end)
  {
    auto const r = static_cast<int>(i - begin);
    this->beginRemoveRows({}, r, r);
    d->points.erase(i);
    this->endRemoveRows();
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::modifyPoint(id_t id)
{
  auto const& index = this->find(id);
  if (index.isValid())
  {
    QTE_D();

    auto& item = d->points[index.row()];
    item.gcp = d->helper->groundControlPoint(id);
    item.crt = d->helper->registrationTrack(id);

    if (!item.crt || item.crt->empty())
    {
      if (auto const oldFrames = item.crtFrames.count())
      {
        this->beginRemoveRows(index, 0, oldFrames - 1);
        item.crtFrames.clear();
        this->endRemoveRows();
      }
    }
    else
    {
      auto i = item.crt->begin();
      auto j = 0;
      auto k = item.crtFrames.count();
      auto const lastI = item.crt->end();

      while (true)
      {
        // Have we run out of frames?
        if (i == lastI)
        {
          if (j < k)
          {
            // Remove extra frames from end
            this->beginRemoveRows(index, j, k - 1);
            while (j < k)
            {
              item.crtFrames.removeLast();
              --k;
            }
            this->endRemoveRows();
          }
          break;
        }
        // Do we need to add new frames to the end?
        else if (j == k)
        {
          while (i != lastI)
          {
            this->beginInsertRows(index, k, k);
            item.crtFrames.append((*i)->frame());
            this->endInsertRows();

            ++k;
            ++i;
          }

          break;
        }
        // Are these frames the same?
        else if ((*i)->frame() == item.crtFrames[j])
        {
          ++i;
          ++j;
        }
        // Need to insert or remove a frame in the middle
        else
        {
          auto const fi = (*i)->frame();
          auto const fj = item.crtFrames[j];

          if (fi > fj) // removal
          {
            this->beginRemoveRows(index, j, j);
            item.crtFrames.removeAt(j);
            this->endRemoveRows();

            --k;
          }
          else // insertion
          {
            this->beginInsertRows(index, j, j);
            item.crtFrames.insert(j, fi);
            this->endInsertRows();

            ++i;
            ++j;
            ++k;
          }
        }
      }
    }

    emit this->dataChanged(index, index);
  }
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::resetPoints()
{
  QTE_D();

  this->beginResetModel();

  d->points.clear();
  if (d->helper)
  {
    for (auto const& id : d->helper->identifiers())
    {
      auto const& gcp = d->helper->groundControlPoint(id);
      auto const& crt = d->helper->registrationTrack(id);
      d->points.append(buildPoint(id, gcp, crt));
    }
  }

  this->endResetModel();
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setDataSource(GroundControlPointsHelper* helper)
{
  QTE_D();

  d->helper = helper;
  this->resetPoints();
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setActiveCamera(kv::frame_id_t id)
{
  QTE_D();

  d->activeCamera = id;

  emit this->dataChanged(
    this->index(0, COLUMN_ID, {}),
    this->index(this->rowCount({}), COLUMN_ID, {}),
    {Qt::DecorationRole});
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setRegisteredIcon(QIcon const& icon)
{
  QTE_D();

  d->registeredIcon = icon;

  for (auto const s : icon.availableSizes())
  {
    QPixmap p{s};
    p.fill(Qt::transparent);
    d->emptyIcon.addPixmap(p);
  }

  emit this->dataChanged(
    this->index(0, COLUMN_ID, {}),
    this->index(this->rowCount({}), COLUMN_ID, {}),
    {Qt::DecorationRole});
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setSurveyedIcon(QIcon const& icon)
{
  QTE_D();

  d->surveyedIcon = icon;

  for (auto const s : icon.availableSizes())
  {
    QPixmap p{s};
    p.fill(Qt::transparent);
    d->emptyIcon.addPixmap(p);
  }

  emit this->dataChanged(
    this->index(0, COLUMN_NAME, {}),
    this->index(this->rowCount({}), COLUMN_NAME, {}),
    {Qt::DecorationRole});
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setCameraIcon(QIcon const& icon)
{
  QTE_D();

  d->cameraIcon = icon;

  for (auto const s : icon.availableSizes())
  {
    QPixmap p{s};
    p.fill(Qt::transparent);
    d->emptyIcon.addPixmap(p);
  }

  for (auto const row : qtIndexRange(this->rowCount({})))
  {
    auto const& parent = this->index(row, 0, {});
    if (auto const rows = this->rowCount(parent))
    {
      emit this->dataChanged(
        this->index(0, COLUMN_ID, parent),
        this->index(this->rowCount(parent), COLUMN_ID, parent),
        {Qt::DecorationRole});
    }
  }
}

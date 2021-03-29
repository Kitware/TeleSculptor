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
#include <qtStlUtil.h>

#include <QDebug>
#include <QIcon>

#include <limits>

namespace kv = kwiver::vital;

namespace
{

using id_t = kv::ground_control_point_id_t;

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

  bool operator<(gcp_ref const& other) const
  { return this->id < other.id; }

  operator id_t() const
  { return this->id; }

  kv::ground_control_point* operator->() const
  { return this->gcp.get(); }
};

} // namespace (anonymous)

//-----------------------------------------------------------------------------
class GroundControlPointsModelPrivate
{
public:
  GroundControlPointsHelper* helper = nullptr;
  QVector<gcp_ref> points;

  QIcon registeredIcon;
  QIcon emptyIcon;
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

  auto const r = index.row();
  if (r < 0 || r > this->rowCount(index.parent()))
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
  return (parent.isValid() ? 0 : d->points.count());
}

//-----------------------------------------------------------------------------
int GroundControlPointsModel::columnCount(const QModelIndex& parent) const
{
  return (parent.isValid() ? 0 : COLUMNS);
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

  return createIndex(row, column);
}

//-----------------------------------------------------------------------------
QModelIndex GroundControlPointsModel::parent(QModelIndex const& child) const
{
  Q_UNUSED(child)
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

  auto const& item = d->points[index.row()];

  switch (index.column())
  {
    case COLUMN_ID:
      switch (role)
      {
        case Qt::DisplayRole:
        case Qt::EditRole:
          return item.id;

        case Qt::TextAlignmentRole:
          return int{Qt::AlignRight | Qt::AlignVCenter};

        case Qt::DecorationRole:
          if (item.gcp && item.gcp->is_geo_loc_user_provided())
          {
            return d->registeredIcon;
          }
          return d->emptyIcon;

        default:
          return {};
      }

    case COLUMN_NAME:
      if (!item.gcp)
      {
      qDebug() << "Eek! Missing ground control point for index"
               << index << "with ID" << item.id << "?!?!";
        return {};
      }

      switch (role)
      {
        case Qt::DisplayRole:
        case Qt::EditRole:
          return qtString(item.gcp->name());

        default:
          return {};
      }

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
    if (index.column() == COLUMN_NAME)
    {
      return baseFlags | Qt::ItemIsEditable;
    }
    return baseFlags | Qt::ItemNeverHasChildren;
  }

  return baseFlags;
}

//-----------------------------------------------------------------------------
bool GroundControlPointsModel::setData(
  QModelIndex const& index, QVariant const& value, int role)
{
  if (!this->checkIndex(index, IndexIsValid) || index.column() != COLUMN_NAME)
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
    qDebug() << "GroundControlPointsModel::addPoint: ID" << id
             << "already exists?!";
    return;
  }

  auto const& gcp = d->helper->groundControlPoint(id);
  if (!gcp)
  {
    qDebug() << "GroundControlPointsModel::addPoint: point with ID" << id
             << "was not found?!";
    return;
  }

  auto const r = static_cast<int>(i - begin);
  this->beginInsertRows({}, r, r);
  d->points.insert(i, {id, gcp});
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
  auto const& index = this->find(id, COLUMN_ID);
  if (index.isValid())
  {
    emit this->dataChanged(index, index, {Qt::DecorationRole});
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
    for (auto const& i : d->helper->identifiers())
    {
      d->points.append({i, d->helper->groundControlPoint(i)});
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

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

#include <qtGet.h>
#include <qtStlUtil.h>

#include <QDebug>

#include <limits>

namespace kv = kwiver::vital;

using id_t = kv::ground_control_point_id_t;

namespace
{

constexpr auto IndexIsValid =
  QAbstractItemModel::CheckIndexOption::IndexIsValid;

enum
{
  COLUMN_ID,
  COLUMN_NAME,
  COLUMNS
};

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
  std::map<id_t, kv::ground_control_point_sptr> const* data = nullptr;
  QVector<gcp_ref> points;
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

  auto const gcpi = qtGet(*d->data, id);
  if (!gcpi)
  {
    qDebug() << "GroundControlPointsModel::addPoint: point with ID" << id
             << "was not found?!";
    return;
  }

  auto const r = static_cast<int>(i - begin);
  this->beginInsertRows({}, r, r);
  d->points.insert(i, {id, gcpi->second});
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
void GroundControlPointsModel::resetPoints()
{
  QTE_D();

  this->beginResetModel();

  d->points.clear();
  if (d->data)
  {
    for (auto const& i : *d->data)
    {
      d->points.append({i.first, i.second});
    }
  }

  this->endResetModel();
}

//-----------------------------------------------------------------------------
void GroundControlPointsModel::setPointData(
  std::map<id_t, kv::ground_control_point_sptr> const& data)
{
  QTE_D();

  d->data = &data;
  this->resetPoints();
}

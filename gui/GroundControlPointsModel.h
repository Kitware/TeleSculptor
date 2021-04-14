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

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSMODEL_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSMODEL_H_

#include <maptk/ground_control_point.h>

#include <qtGlobal.h>

#include <QAbstractItemModel>

class GroundControlPointsHelper;

class GroundControlPointsModelPrivate;

class GroundControlPointsModel : public QAbstractItemModel
{
  Q_OBJECT

public:
  GroundControlPointsModel(QObject* parent = nullptr);
  ~GroundControlPointsModel();

  kwiver::vital::ground_control_point_id_t id(QModelIndex const& index) const;
  QModelIndex find(kwiver::vital::ground_control_point_id_t,
                   int column = 0) const;

  int rowCount(QModelIndex const& parent) const override;
  int columnCount(QModelIndex const& parent) const override;

  QModelIndex index(int row, int column,
                    QModelIndex const& parent) const override;
  QModelIndex parent(QModelIndex const& child) const override;

  QVariant data(QModelIndex const& index, int role) const override;
  Qt::ItemFlags flags(QModelIndex const& index) const override;
  bool setData(QModelIndex const& index, QVariant const& value,
               int role) override;

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;

  void setDataSource(GroundControlPointsHelper*);

  void setRegisteredIcon(QIcon const& icon);

public slots:
  void addPoint(kwiver::vital::ground_control_point_id_t);
  void removePoint(kwiver::vital::ground_control_point_id_t);
  void modifyPoint(kwiver::vital::ground_control_point_id_t);

  void resetPoints();

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsModel)
  QTE_DECLARE_PRIVATE(GroundControlPointsModel)

  QTE_DISABLE_COPY(GroundControlPointsModel)
};

#endif

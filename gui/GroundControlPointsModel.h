// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_GROUNDCONTROLPOINTSMODEL_H_
#define TELESCULPTOR_GROUNDCONTROLPOINTSMODEL_H_

#include <maptk/ground_control_point.h>

#include <vital/vital_types.h>

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
  void setSurveyedIcon(QIcon const& icon);
  void setCameraIcon(QIcon const& icon);

public slots:
  void addPoint(kwiver::vital::ground_control_point_id_t);
  void removePoint(kwiver::vital::ground_control_point_id_t);
  void modifyPoint(kwiver::vital::ground_control_point_id_t);

  void resetPoints();

  void setActiveCamera(kwiver::vital::frame_id_t);

private:
  QTE_DECLARE_PRIVATE_RPTR(GroundControlPointsModel)
  QTE_DECLARE_PRIVATE(GroundControlPointsModel)

  QTE_DISABLE_COPY(GroundControlPointsModel)
};

#endif

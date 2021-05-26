// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VIDEOINPUT_H_
#define TELESCULPTOR_VIDEOINPUT_H_

#include <vital/types/local_geo_cs.h>

#include <vital/vital_types.h>
#include <vital/config/config_block_types.h>
#include <vital/logger/logger.h>
#include <vital/types/camera.h>
#include <vital/types/metadata.h>
#include <vital/types/metadata_map.h>

#include <qtGlobal.h>

#include <QMetaType>
#include <QtCore/QThread>

class VideoImportPrivate;

Q_DECLARE_METATYPE(std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>);

class VideoImport : public QThread
{
  Q_OBJECT

public:
  typedef kwiver::vital::config_block_sptr config_block_sptr;

  VideoImport(QObject* parent = nullptr);
  ~VideoImport() override;

  void setData(config_block_sptr const&,
               std::string const&,
               kwiver::vital::local_geo_cs& lgcs);

signals:
  /// Emitted when the tool execution is completed.
  void completed(std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>);
  /// Emitted when an intermediate update of the data is available to show progress.
  void updated(int);
  /// Update progress
  void progressChanged(QString, int);

public slots:
  void cancel();

protected:
  /// Execute the tool.
  ///
  /// This method must be overridden by tool implementations. The default
  /// implementation of execute() calls this method in a separate thread.
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(VideoImport)
  QTE_DECLARE_PRIVATE(VideoImport)
  QTE_DISABLE_COPY(VideoImport)
};

#endif
